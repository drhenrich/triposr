import struct
import unittest

from scan3d import stream


def capsule_bytes(seq=0, flags=stream.FLAG_SWEEP_ACTIVE, yaw0=10.0, yaw1=10.0125,
                  alpha0=0.0, alpha_inc=0.1125, distances=None):
    if distances is None:
        distances = list(range(1000, 1000 + stream.CABIN_COUNT))
    return stream.encode_capsule(seq, flags, yaw0, yaw1, alpha0, alpha_inc, distances)


class TestFrameParser(unittest.TestCase):
    def test_roundtrip(self):
        raw = capsule_bytes(seq=7)
        frames = list(stream.FrameParser().feed(raw))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].type, stream.TYPE_CAPSULE)
        self.assertEqual(frames[0].seq, 7)
        self.assertEqual(len(frames[0].payload), stream.CAPSULE_PAYLOAD_SIZE)

    def test_frame_size_is_104_bytes(self):
        self.assertEqual(len(capsule_bytes()), stream.HEADER_SIZE + 96)

    def test_split_across_chunks(self):
        raw = capsule_bytes()
        parser = stream.FrameParser()
        self.assertEqual(list(parser.feed(raw[:5])), [])
        self.assertEqual(list(parser.feed(raw[5:60])), [])
        self.assertEqual(len(list(parser.feed(raw[60:]))), 1)

    def test_resync_after_garbage(self):
        parser = stream.FrameParser()
        frames = list(parser.feed(b"\xde\xad\xbe\xef" + capsule_bytes()))
        self.assertEqual(len(frames), 1)
        self.assertEqual(parser.resyncs, 4)

    def test_back_to_back_frames(self):
        blob = b"".join(capsule_bytes(seq=i) for i in range(50))
        frames = list(stream.FrameParser().feed(blob))
        self.assertEqual([f.seq for f in frames], list(range(50)))

    def test_oversized_length_is_rejected(self):
        bad = stream.HEADER.pack(stream.MAGIC, stream.TYPE_CAPSULE, 0, 0, 9999)
        parser = stream.FrameParser()
        frames = list(parser.feed(bad + capsule_bytes()))
        self.assertEqual(len(frames), 1)


class TestCapsuleFrame(unittest.TestCase):
    def test_decode_roundtrip(self):
        distances = list(range(500, 500 + stream.CABIN_COUNT))
        raw = capsule_bytes(seq=3, yaw0=12.5, yaw1=12.5125,
                            alpha0=45.0, alpha_inc=0.1125, distances=distances)
        frame = next(iter(stream.FrameParser().feed(raw)))
        cap = stream.decode_capsule(frame)
        self.assertEqual(cap.seq, 3)
        self.assertTrue(cap.sweep_active)
        self.assertAlmostEqual(cap.yaw_start_deg, 12.5, places=4)
        self.assertAlmostEqual(cap.yaw_end_deg, 12.5125, places=4)
        self.assertAlmostEqual(cap.alpha_start_deg, 45.0, places=3)
        self.assertAlmostEqual(cap.alpha_inc_deg, 0.1125, delta=1 / 65536)
        self.assertEqual(list(cap.distances_mm), distances)

    def test_samples_interpolate_yaw_and_alpha(self):
        raw = capsule_bytes(yaw0=10.0, yaw1=10.4, alpha0=0.0, alpha_inc=0.1)
        cap = stream.decode_capsule(next(iter(stream.FrameParser().feed(raw))))
        samples = list(cap.samples())
        self.assertEqual(len(samples), stream.CABIN_COUNT)
        self.assertAlmostEqual(samples[0][1], 0.0, places=3)
        self.assertAlmostEqual(samples[0][2], 10.0, places=3)
        self.assertAlmostEqual(samples[20][1], 2.0, places=3)
        self.assertAlmostEqual(samples[20][2], 10.2, places=3)

    def test_alpha_wraps_past_360(self):
        raw = capsule_bytes(alpha0=358.0, alpha_inc=0.1)
        cap = stream.decode_capsule(next(iter(stream.FrameParser().feed(raw))))
        alphas = [s[1] for s in cap.samples()]
        self.assertTrue(all(0.0 <= a < 360.0 for a in alphas))
        self.assertLess(alphas[-1], 2.0)

    def test_negative_alpha_increment(self):
        raw = capsule_bytes(alpha0=10.0, alpha_inc=-0.1)
        cap = stream.decode_capsule(next(iter(stream.FrameParser().feed(raw))))
        self.assertAlmostEqual(cap.alpha_inc_deg, -0.1, delta=1 / 65536)
        self.assertAlmostEqual(list(cap.samples())[10][1], 9.0, places=3)

    def test_wrong_type_rejected(self):
        frame = stream.Frame(type=stream.TYPE_STATUS, flags=0, seq=0, payload=b"")
        with self.assertRaises(ValueError):
            stream.decode_capsule(frame)

    def test_wrong_payload_size_rejected(self):
        frame = stream.Frame(type=stream.TYPE_CAPSULE, flags=0, seq=0, payload=b"\x00" * 10)
        with self.assertRaises(ValueError):
            stream.decode_capsule(frame)


class TestHelloAndStatus(unittest.TestCase):
    def test_hello(self):
        payload = stream._HELLO.pack(1, 600, -40500, 12000,
                                     0, round(180 * 65536))
        raw = stream.encode_frame(stream.TYPE_HELLO, 0, 0, payload)
        hello = stream.decode_hello(next(iter(stream.FrameParser().feed(raw))))
        self.assertEqual(hello.fw_version, 1)
        self.assertEqual(hello.lidar_rpm, 600)
        self.assertAlmostEqual(hello.offset_radial_mm, -40.5)
        self.assertAlmostEqual(hello.offset_axial_mm, 12.0)
        self.assertAlmostEqual(hello.yaw_max_deg, 180.0, places=4)

    def test_status(self):
        payload = stream._STATUS.pack(2, 1, 0, round(90.5 * 65536), 12345, 3, 0)
        raw = stream.encode_frame(stream.TYPE_STATUS, 0, 0, payload)
        st = stream.decode_status(next(iter(stream.FrameParser().feed(raw))))
        self.assertEqual(st.sweep_index, 2)
        self.assertEqual(st.state, 1)
        self.assertAlmostEqual(st.yaw_deg, 90.5, places=4)
        self.assertEqual(st.capsules, 12345)
        self.assertEqual(st.checksum_errors, 3)


class TestCollectSweep(unittest.TestCase):
    def _stream(self):
        hello = stream.encode_frame(
            stream.TYPE_HELLO, 0, 0,
            stream._HELLO.pack(1, 600, 0, 0, 0, round(180 * 65536)))
        idle = stream.encode_frame(
            stream.TYPE_STATUS, 0, 99, stream._STATUS.pack(0, 0, 0, 0, 0, 0, 0))
        running = stream.encode_frame(
            stream.TYPE_STATUS, 0, 98, stream._STATUS.pack(0, 1, 0, 0, 0, 0, 0))
        parked = capsule_bytes(seq=0, flags=0)  # vor dem Sweep, muss ignoriert werden
        active = b"".join(
            capsule_bytes(seq=i, yaw0=i * 0.1, yaw1=(i + 1) * 0.1) for i in range(1, 4))
        trailing = capsule_bytes(seq=50, flags=stream.FLAG_SWEEP_ACTIVE)
        return hello + idle + parked + running + active + idle + trailing

    def test_stops_at_idle_status_after_active_capsules(self):
        frames = stream.FrameParser().feed(self._stream())
        samples, hello = stream.collect_sweep(frames)
        self.assertIsNotNone(hello)
        self.assertEqual(hello.lidar_rpm, 600)
        # 3 aktive Capsules a 40 Messungen; die geparkte und die nach dem
        # Idle-Status zaehlen nicht mit.
        self.assertEqual(len(samples), 3 * stream.CABIN_COUNT)

    def test_yaw_increases_monotonically(self):
        frames = stream.FrameParser().feed(self._stream())
        samples, _ = stream.collect_sweep(frames)
        yaws = [s[2] for s in samples]
        self.assertEqual(yaws, sorted(yaws))


if __name__ == "__main__":
    unittest.main()
