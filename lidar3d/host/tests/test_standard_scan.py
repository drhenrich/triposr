"""Dekoder fuer den einfachen Scanmodus (RPLIDAR C1, Antworttyp 0x81)."""

import struct
import unittest

from scan3d import rplidar


def build_node(angle_deg: float, distance_mm: float, quality: int = 47,
               start: bool = False, break_check_bit: bool = False) -> bytes:
    """Eine 5-Byte-Messung bauen, wie sie der C1 im einfachen Modus schickt."""
    b0 = (quality << 2) | (0b01 if start else 0b10)
    angle_q6 = round(angle_deg * 64) & 0x7FFF
    b1 = ((angle_q6 & 0x7F) << 1) | (0 if break_check_bit else 1)
    b2 = (angle_q6 >> 7) & 0xFF
    return bytes([b0, b1, b2]) + struct.pack("<H", round(distance_mm * 4))


class TestStandardScanParser(unittest.TestCase):
    def test_decodes_single_node(self):
        parser = rplidar.StandardScanParser()
        samples = list(parser.feed(build_node(123.5, 2500.0, quality=47)))
        self.assertEqual(len(samples), 1)
        s = samples[0]
        self.assertAlmostEqual(s.angle_deg, 123.5, delta=1 / 64)
        self.assertAlmostEqual(s.distance_mm, 2500.0, delta=0.25)
        self.assertEqual(s.quality, 47)
        self.assertFalse(s.new_revolution)

    def test_start_flag(self):
        parser = rplidar.StandardScanParser()
        samples = list(parser.feed(build_node(0.0, 1000.0, start=True)))
        self.assertTrue(samples[0].new_revolution)

    def test_zero_distance_means_no_echo(self):
        parser = rplidar.StandardScanParser()
        samples = list(parser.feed(build_node(90.0, 0.0)))
        self.assertEqual(samples[0].distance_mm, 0.0)

    def test_split_across_chunks(self):
        parser = rplidar.StandardScanParser()
        raw = build_node(45.0, 1234.0)
        self.assertEqual(list(parser.feed(raw[:2])), [])
        samples = list(parser.feed(raw[2:]))
        self.assertEqual(len(samples), 1)
        self.assertAlmostEqual(samples[0].angle_deg, 45.0, delta=1 / 64)

    def test_stream_of_many_nodes(self):
        parser = rplidar.StandardScanParser()
        blob = b"".join(build_node(i * 0.72, 1000 + i) for i in range(500))
        samples = list(parser.feed(blob))
        self.assertEqual(len(samples), 500)
        self.assertEqual(parser.resyncs, 0)

    def test_resyncs_after_garbage(self):
        parser = rplidar.StandardScanParser()
        # 0x00 verletzt sowohl S != !S als auch das Pruefbit.
        raw = b"\x00\x00\x00" + build_node(200.0, 3000.0)
        samples = list(parser.feed(raw))
        self.assertEqual(len(samples), 1)
        self.assertGreaterEqual(parser.resyncs, 3)
        self.assertAlmostEqual(samples[0].angle_deg, 200.0, delta=1 / 64)

    def test_rejects_node_with_bad_check_bit(self):
        parser = rplidar.StandardScanParser()
        bad = build_node(10.0, 500.0, break_check_bit=True)
        good = build_node(20.0, 600.0)
        samples = list(parser.feed(bad + good))
        # Das kaputte Byte wird uebersprungen; die gute Messung kommt durch.
        self.assertTrue(any(abs(s.angle_deg - 20.0) < 1 for s in samples))
        self.assertGreater(parser.resyncs, 0)

    def test_angle_wraps_into_0_360(self):
        parser = rplidar.StandardScanParser()
        samples = list(parser.feed(build_node(359.9, 1000.0)))
        self.assertLess(samples[0].angle_deg, 360.0)
        self.assertGreater(samples[0].angle_deg, 359.0)

    def test_c1_revolution_size(self):
        """5000 Messungen/s bei 10 Hz -> 500 Punkte je Umdrehung, 0.72 Grad."""
        parser = rplidar.StandardScanParser()
        blob = b"".join(
            build_node((i * 0.72) % 360.0, 2000.0, start=(i == 0))
            for i in range(500)
        )
        samples = list(parser.feed(blob))
        self.assertEqual(len(samples), 500)
        self.assertEqual(sum(s.new_revolution for s in samples), 1)


class TestRevolutionAssembler(unittest.TestCase):
    def _stream(self, revolutions: int, per_rev: int = 500):
        blob = b"".join(
            build_node((i * 360.0 / per_rev) % 360.0, 2000.0, start=(i % per_rev == 0))
            for i in range(revolutions * per_rev)
        )
        return blob

    def test_drops_the_first_partial_revolution(self):
        parser = rplidar.StandardScanParser()
        assembler = rplidar.RevolutionAssembler()
        # Mitten in einer Umdrehung einsteigen: 100 Messungen ohne Marke davor.
        lead = b"".join(build_node(i * 0.72, 1000.0) for i in range(100))
        blob = lead + self._stream(2)
        revs = list(assembler.feed(parser.feed(blob)))
        # Zwei Marken begrenzen genau eine vollstaendige Umdrehung.
        self.assertEqual(len(revs), 1)
        self.assertEqual(len(revs[0]), 500)

    def test_yields_complete_revolutions(self):
        parser = rplidar.StandardScanParser()
        assembler = rplidar.RevolutionAssembler()
        revs = list(assembler.feed(parser.feed(self._stream(4))))
        self.assertEqual(len(revs), 3)  # die letzte ist noch offen
        self.assertTrue(all(len(r) == 500 for r in revs))
        self.assertEqual(assembler.revolutions, 3)

    def test_each_revolution_starts_at_its_marker(self):
        parser = rplidar.StandardScanParser()
        assembler = rplidar.RevolutionAssembler()
        revs = list(assembler.feed(parser.feed(self._stream(3))))
        for rev in revs:
            self.assertTrue(rev[0].new_revolution)
            self.assertFalse(any(s.new_revolution for s in rev[1:]))

    def test_reset_clears_state(self):
        parser = rplidar.StandardScanParser()
        assembler = rplidar.RevolutionAssembler()
        list(assembler.feed(parser.feed(self._stream(2))))
        assembler.reset()
        self.assertEqual(list(assembler.feed([])), [])


class TestCommands(unittest.TestCase):
    def test_scan_command(self):
        self.assertEqual(rplidar.cmd_scan(), b"\xa5\x20")

    def test_info_and_health(self):
        self.assertEqual(rplidar.cmd_get_device_info(), b"\xa5\x50")
        self.assertEqual(rplidar.cmd_get_health(), b"\xa5\x52")


if __name__ == "__main__":
    unittest.main()
