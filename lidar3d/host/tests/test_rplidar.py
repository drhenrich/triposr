import struct
import unittest

from scan3d import rplidar


def build_capsule(start_angle_deg: float, distances, start_flag: bool = False) -> bytes:
    """Eine gueltige Dense-Capsule bauen (84 Byte, inklusive Pruefsumme)."""
    assert len(distances) == rplidar.DENSE_CABIN_COUNT
    raw_start = round(start_angle_deg * 64) & 0x7FFF
    if start_flag:
        raw_start |= 0x8000
    body = struct.pack("<H", raw_start) + struct.pack(
        f"<{rplidar.DENSE_CABIN_COUNT}H", *distances
    )
    checksum = 0
    for b in body:
        checksum ^= b
    b0 = (rplidar.EXP_SYNC_1 << 4) | (checksum & 0x0F)
    b1 = (rplidar.EXP_SYNC_2 << 4) | ((checksum >> 4) & 0x0F)
    return bytes([b0, b1]) + body


class TestCommands(unittest.TestCase):
    def test_simple_command(self):
        self.assertEqual(rplidar.cmd_stop(), b"\xa5\x25")
        self.assertEqual(rplidar.cmd_reset(), b"\xa5\x40")

    def test_express_scan_checksum(self):
        frame = rplidar.cmd_express_scan(0)
        self.assertEqual(frame[:3], b"\xa5\x82\x05")
        self.assertEqual(len(frame), 3 + 5 + 1)
        expected = 0
        for b in frame[:-1]:
            expected ^= b
        self.assertEqual(frame[-1], expected)

    def test_motor_rpm_payload(self):
        frame = rplidar.cmd_motor_rpm(600)
        self.assertEqual(frame[3:5], struct.pack("<H", 600))

    def test_get_lidar_conf_with_and_without_mode(self):
        self.assertEqual(rplidar.cmd_get_lidar_conf(rplidar.CONF_SCAN_MODE_TYPICAL)[2], 4)
        self.assertEqual(rplidar.cmd_get_lidar_conf(rplidar.CONF_SCAN_MODE_ANS_TYPE, 1)[2], 6)

    def test_response_descriptor(self):
        raw = b"\xa5\x5a" + struct.pack("<I", 84 | (1 << 30)) + bytes([0x85])
        desc = rplidar.parse_response_descriptor(raw)
        self.assertEqual(desc.length, 84)
        self.assertTrue(desc.is_stream)
        self.assertEqual(desc.data_type, rplidar.ANS_TYPE_MEASUREMENT_DENSE_CAPSULED)

    def test_response_descriptor_rejects_bad_header(self):
        with self.assertRaises(ValueError):
            rplidar.parse_response_descriptor(b"\x00" * 7)


class TestCapsuleParser(unittest.TestCase):
    def setUp(self):
        self.distances = tuple(range(1000, 1000 + rplidar.DENSE_CABIN_COUNT))

    def test_parses_capsule(self):
        parser = rplidar.CapsuleParser()
        raw = build_capsule(12.5, self.distances, start_flag=True)
        capsules = list(parser.feed(raw))
        self.assertEqual(len(capsules), 1)
        self.assertAlmostEqual(capsules[0].start_angle_deg, 12.5, places=3)
        self.assertTrue(capsules[0].start_flag)
        self.assertEqual(tuple(capsules[0].distances_mm), self.distances)

    def test_split_across_chunks(self):
        parser = rplidar.CapsuleParser()
        raw = build_capsule(0.0, self.distances)
        self.assertEqual(list(parser.feed(raw[:30])), [])
        self.assertEqual(len(list(parser.feed(raw[30:]))), 1)

    def test_resyncs_after_garbage(self):
        parser = rplidar.CapsuleParser()
        raw = b"\x11\x22\x33" + build_capsule(90.0, self.distances)
        capsules = list(parser.feed(raw))
        self.assertEqual(len(capsules), 1)
        self.assertGreaterEqual(parser.resyncs, 3)

    def test_rejects_bad_checksum(self):
        parser = rplidar.CapsuleParser()
        raw = bytearray(build_capsule(0.0, self.distances))
        raw[10] ^= 0xFF  # eine Distanz kippen -> Pruefsumme passt nicht mehr
        list(parser.feed(bytes(raw)))
        self.assertEqual(parser.checksum_errors, 1)

    def test_stream_of_many_capsules(self):
        parser = rplidar.CapsuleParser()
        blob = b"".join(
            build_capsule(i * 4.5, self.distances) for i in range(20)
        )
        self.assertEqual(len(list(parser.feed(blob))), 20)
        self.assertEqual(parser.resyncs, 0)


class TestCapsuleDecoder(unittest.TestCase):
    def setUp(self):
        self.distances = tuple([2000] * rplidar.DENSE_CABIN_COUNT)

    def _capsule(self, angle_deg):
        parser = rplidar.CapsuleParser()
        return next(iter(parser.feed(build_capsule(angle_deg, self.distances))))

    def test_first_capsule_yields_nothing(self):
        dec = rplidar.CapsuleDecoder()
        self.assertEqual(dec.push(self._capsule(0.0)), [])

    def test_angles_interpolated_between_capsules(self):
        dec = rplidar.CapsuleDecoder()
        dec.push(self._capsule(0.0))
        samples = dec.push(self._capsule(4.0))
        self.assertEqual(len(samples), rplidar.DENSE_CABIN_COUNT)
        # Winkel sind im Protokoll auf Q6 quantisiert -> 1/64 deg Toleranz.
        lsb = 1.0 / 64
        self.assertAlmostEqual(samples[0].angle_deg, 0.0, delta=lsb)
        # 4 deg auf 40 Messungen = 0.1 deg Schritt
        self.assertAlmostEqual(samples[10].angle_deg, 1.0, delta=lsb)
        self.assertAlmostEqual(samples[39].angle_deg, 3.9, delta=lsb)

    def test_wraparound_across_zero(self):
        dec = rplidar.CapsuleDecoder()
        dec.push(self._capsule(358.0))
        samples = dec.push(self._capsule(2.0))
        self.assertAlmostEqual(samples[0].angle_deg, 358.0, places=2)
        # Winkel muessen ueber 360 hinweg sauber umlaufen, nicht springen
        self.assertLess(samples[-1].angle_deg, 2.0)
        self.assertGreaterEqual(samples[-1].angle_deg, 0.0)
        self.assertTrue(any(s.new_revolution for s in samples))

    def _run(self, angles):
        dec = rplidar.CapsuleDecoder()
        samples = []
        for angle in angles:
            samples.extend(dec.push(self._capsule(angle)))
        return samples

    def test_full_revolution_sample_count(self):
        """10 Hz bei 32000 Messungen/s -> 3200 Messungen je Umdrehung."""
        step = 360.0 / 80  # 80 Capsules a 40 Messungen
        samples = self._run([(i * step) % 360.0 for i in range(161)])
        self.assertEqual(len(samples), 160 * rplidar.DENSE_CABIN_COUNT)
        markers = [i for i, s in enumerate(samples) if s.new_revolution]
        self.assertEqual(markers, [3200])

    def test_revolution_marker_lands_inside_capsule(self):
        """Startwinkel, die nicht auf 360 aufgehen: Marker mitten in der Capsule."""
        step = 7.0  # 360/7 ist nicht ganzzahlig -> eine Capsule ueberspannt die Grenze
        samples = self._run([(i * step) % 360.0 for i in range(120)])
        markers = [i for i, s in enumerate(samples) if s.new_revolution]
        self.assertEqual(len(markers), 2)
        # Genau eine Umdrehung zwischen zwei Markern, +/- eine Messung Rundung.
        self.assertAlmostEqual(markers[1] - markers[0], 360.0 / step * 40, delta=1)
        # Direkt vor dem Marker liegt das Ende der Umdrehung, danach der Anfang.
        self.assertGreater(samples[markers[0] - 1].angle_deg, 350.0)
        self.assertLess(samples[markers[0]].angle_deg, 10.0)

    def test_angle_increment_q16(self):
        dec = rplidar.CapsuleDecoder()
        dec.push(self._capsule(0.0))
        inc = dec.angle_increment_q16(self._capsule(4.0))
        self.assertAlmostEqual(inc / 65536.0, 0.1, places=3)


if __name__ == "__main__":
    unittest.main()
