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

    #: Die acht Messungen des Scanframes: Winkel bewusst ungleichmaessig,
    #: genau so liefert der C1 sie.
    SCAN_SAMPLES = [(0.0, 1000), (0.72, 1005), (1.51, 1010), (2.19, 1015),
                    (2.95, 1020), (3.68, 1025), (4.39, 1030), (5.14, 1035)]

    def test_fixture_has_all_frame_types(self):
        self.assertEqual(set(self.fixture),
                         {"scan", "capsule", "hello", "status", "fault"})

    def test_scan_bytes(self):
        raw = stream.encode_scan(
            seq=4,
            flags=stream.FLAG_NEW_REVOLUTION | stream.FLAG_SWEEP_ACTIVE,
            yaw_start_deg=42.0,
            yaw_end_deg=42.5,
            samples=self.SCAN_SAMPLES,
        )
        self.assertEqual(raw, self.fixture["scan"])
        # 8 Header + 12 Kopf + 8 Messungen a 4 Byte
        self.assertEqual(len(raw), 52)

    def test_scan_decodes_back(self):
        scan = stream.decode_scan(
            next(iter(stream.FrameParser().feed(self.fixture["scan"])))
        )
        self.assertTrue(scan.new_revolution)
        self.assertTrue(scan.sweep_active)
        self.assertAlmostEqual(scan.yaw_start_deg, 42.0, places=4)
        self.assertAlmostEqual(scan.yaw_end_deg, 42.5, places=4)
        self.assertEqual(len(scan.distances_mm), 8)
        for got, (alpha, distance) in zip(zip(scan.angles_deg, scan.distances_mm),
                                          self.SCAN_SAMPLES):
            # Winkel sind Q6, quantisieren also auf 1/64 Grad.
            self.assertAlmostEqual(got[0], alpha, delta=1 / 64)
            self.assertEqual(got[1], distance)

    def test_scan_interpolates_only_the_yaw(self):
        scan = stream.decode_scan(
            next(iter(stream.FrameParser().feed(self.fixture["scan"])))
        )
        rows = list(scan.samples())
        self.assertAlmostEqual(rows[0][2], 42.0, places=4)
        self.assertAlmostEqual(rows[4][2], 42.25, places=4)
        # Der Scanwinkel kommt unveraendert aus den Bytes, nicht aus einem Raster.
        self.assertEqual([r[1] for r in rows], list(scan.angles_deg))

    def test_scan_with_mismatched_count_is_rejected(self):
        raw = bytearray(self.fixture["scan"])
        raw[8 + 8] = 9  # count auf 9 stellen, ohne Bytes anzuhaengen
        frame = next(iter(stream.FrameParser().feed(bytes(raw))))
        with self.assertRaises(ValueError):
            stream.decode_scan(frame)

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

    FAULT_TEXT = ("STS3215 antwortet nicht. Bus-ID, Baudrate (1 Mbaud), "
                  "Halbduplex und 12 V pruefen.")

    def test_fault_bytes(self):
        raw = stream.encode_fault(9, stream.FAULT_SERVO, self.FAULT_TEXT)
        self.assertEqual(raw, self.fixture["fault"])

    def test_fault_decodes_back(self):
        fault = stream.decode_fault(
            next(iter(stream.FrameParser().feed(self.fixture["fault"])))
        )
        self.assertEqual(fault.code, stream.FAULT_SERVO)
        self.assertEqual(fault.text, self.FAULT_TEXT)
        self.assertEqual(fault.seq, 9)

    def test_fault_text_is_capped(self):
        raw = stream.encode_fault(0, stream.FAULT_QUEUE, "x" * 400)
        fault = stream.decode_fault(next(iter(stream.FrameParser().feed(raw))))
        self.assertEqual(len(fault.text), stream.FAULT_MAX_TEXT_LEN)

    def test_fault_with_mismatched_length_is_rejected(self):
        raw = bytearray(self.fixture["fault"])
        raw[8 + 1] = 200  # Laengenbyte hochsetzen, ohne Text anzuhaengen
        frame = next(iter(stream.FrameParser().feed(bytes(raw))))
        with self.assertRaises(ValueError):
            stream.decode_fault(frame)

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
