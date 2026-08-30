import os
import struct
import tempfile
import unittest

from scan3d.ply import height_colors, write_ply


class TestWritePly(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.path = os.path.join(self.tmp, "out.ply")

    def _read(self):
        with open(self.path, "rb") as fh:
            data = fh.read()
        marker = b"end_header\n"
        idx = data.index(marker) + len(marker)
        return data[:idx].decode("ascii"), data[idx:]

    def test_writes_header_and_body(self):
        points = [(1.0, 2.0, 3.0), (-1.5, 0.0, 0.25)]
        write_ply(self.path, points)
        header, body = self._read()
        self.assertIn("element vertex 2", header)
        self.assertIn("format binary_little_endian 1.0", header)
        self.assertNotIn("red", header)
        self.assertEqual(len(body), 2 * 12)
        self.assertEqual(struct.unpack("<fff", body[:12]), (1.0, 2.0, 3.0))

    def test_writes_colors(self):
        points = [(0.0, 0.0, 0.0)]
        write_ply(self.path, points, [(10, 200, 30)])
        header, body = self._read()
        self.assertIn("property uchar green", header)
        self.assertEqual(len(body), 15)
        self.assertEqual(tuple(body[12:15]), (10, 200, 30))

    def test_empty_cloud(self):
        write_ply(self.path, [])
        header, body = self._read()
        self.assertIn("element vertex 0", header)
        self.assertEqual(body, b"")

    def test_mismatched_colors_rejected(self):
        with self.assertRaises(ValueError):
            write_ply(self.path, [(0.0, 0.0, 0.0)], [])


class TestHeightColors(unittest.TestCase):
    def test_ramp_spans_the_cloud(self):
        colors = height_colors([(0, 0, 0.0), (0, 0, 1.0), (0, 0, 2.0)])
        self.assertEqual(len(colors), 3)
        self.assertLess(colors[0][1], colors[-1][1])
        self.assertTrue(all(0 <= c <= 255 for rgb in colors for c in rgb))

    def test_flat_cloud_does_not_divide_by_zero(self):
        colors = height_colors([(0, 0, 1.0)] * 3)
        self.assertEqual(len(set(colors)), 1)

    def test_empty(self):
        self.assertEqual(height_colors([]), [])


if __name__ == "__main__":
    unittest.main()
