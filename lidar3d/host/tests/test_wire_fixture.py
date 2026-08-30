"""Beide Protokollseiten gegen dieselbe Byte-Fixture pruefen.

Die Firmware prueft dieselbe Datei in firmware/test/native/test_main.cpp.
Aendert jemand nur eine Seite des Protokolls, schlaegt genau ein Test fehl.
"""

import os
import unittest

from scan3d import stream

FIXTURE = os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir, "tests", "wire_fixture.txt"
)


def load_fixture():
    entries = {}
    with open(FIXTURE, "r", encoding="ascii") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            name, hexbytes = line.split()
            entries[name] = bytes.fromhex(hexbytes)
    return entries


class TestWireFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = load_fixture()

    def test_fixture_has_all_frame_types(self):
        self.assertEqual(set(self.fixture), {"capsule", "hello", "status"})

    def test_capsule_bytes(self):
        distances = [1000 + 7 * i for i in range(stream.CABIN_COUNT)]
        raw = stream.encode_capsule(
            seq=0x1234,
            flags=stream.FLAG_SWEEP_ACTIVE | stream.FLAG_NEW_REVOLUTION,
            yaw_start_deg=42.5,
            yaw_end_deg=42.5125,
            alpha_start_deg=123.25,
            alpha_inc_deg=0.1125,
            distances_mm=distances,
        )
        self.assertEqual(raw, self.fixture["capsule"])
        self.assertEqual(len(raw), 104)

    def test_hello_bytes(self):
        payload = stream._HELLO.pack(1, 600, -40500, 12000, 0, round(180 * 65536))
        raw = stream.encode_frame(stream.TYPE_HELLO, 0, 1, payload)
        self.assertEqual(raw, self.fixture["hello"])

    def test_status_bytes(self):
        payload = stream._STATUS.pack(
            3, stream.TYPE_STATUS, 0, round(90.5 * 65536), 123456, 7, 2
        )
        raw = stream.encode_frame(stream.TYPE_STATUS, 0, 2, payload)
        self.assertEqual(raw, self.fixture["status"])

    def test_fixture_decodes_back(self):
        cap = stream.decode_capsule(
            next(iter(stream.FrameParser().feed(self.fixture["capsule"])))
        )
        self.assertAlmostEqual(cap.yaw_start_deg, 42.5, places=4)
        self.assertAlmostEqual(cap.alpha_start_deg, 123.25, places=3)
        self.assertEqual(cap.distances_mm[0], 1000)
        self.assertEqual(cap.distances_mm[-1], 1000 + 7 * 39)


if __name__ == "__main__":
    unittest.main()
