"""PLY-Leser, Kennzahlen und Befund des Viewer-Generators.

Der Befund entscheidet, ob ein Scan als brauchbar gilt - deshalb wird er
gegen zwei kuenstliche Wolken geprueft, bei denen die richtige Antwort
feststeht: ein Raum (Struktur vorhanden) und ein Rotationskoerper (Drehung
hat nichts beigetragen).
"""

import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

import make_viewer  # noqa: E402

from scan3d import ply  # noqa: E402
from scan3d.quality import analyse, verdict  # noqa: E402
from scan3d.geometry import to_cartesian  # noqa: E402
from scan3d.roomsim import RoomSimulator  # noqa: E402


def room_cloud(planes=36, per_plane=200):
    """Ein echter Raum, so wie ihn ein richtig montierter Scanner sieht."""
    sim = RoomSimulator(room_m=(6.0, 4.0, 2.6), noise_mm=0.0, dropout_rate=0.0)
    points = []
    for p in range(planes):
        yaw = p * 180.0 / planes
        for k in range(per_plane):
            alpha = 360.0 * k / per_plane
            distance = sim.distance_mm(alpha, yaw)
            if distance > 150:
                points.append(to_cartesian(distance, alpha, yaw))
    return points


def revolution_cloud(planes=36, per_plane=200, radius_mm=1500.0):
    """Der Fehlerfall: jede Ebene misst dasselbe, nur anders eingetragen.

    Genau das entsteht, wenn die Scanebene die Drehachse *nicht* enthaelt.
    """
    points = []
    for p in range(planes):
        yaw = p * 180.0 / planes
        for k in range(per_plane):
            alpha = 360.0 * k / per_plane
            points.append(to_cartesian(radius_mm, alpha, yaw))
    return points


class TestReadPly(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()

    def test_reads_binary_written_by_our_own_writer(self):
        path = os.path.join(self.dir, "a.ply")
        original = [(1.0, 2.0, 3.0), (-1.5, 0.25, -0.75)]
        ply.write_ply(path, original)
        back = make_viewer.read_ply(path)
        self.assertEqual(len(back), 2)
        for got, want in zip(back, original):
            for a, b in zip(got, want):
                self.assertAlmostEqual(a, b, places=5)

    def test_reads_binary_with_colour_columns(self):
        path = os.path.join(self.dir, "b.ply")
        points = [(0.5, -0.5, 1.25)]
        ply.write_ply(path, points, [(10, 20, 30)])
        back = make_viewer.read_ply(path)
        self.assertEqual(len(back), 1)
        self.assertAlmostEqual(back[0][2], 1.25, places=5)

    def test_reads_ascii(self):
        path = os.path.join(self.dir, "c.ply")
        with open(path, "w", encoding="ascii") as fh:
            fh.write("ply\nformat ascii 1.0\nelement vertex 2\n"
                     "property float x\nproperty float y\nproperty float z\n"
                     "end_header\n1 2 3\n4 5 6\n")
        back = make_viewer.read_ply(path)
        self.assertEqual(back, [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])

    def test_rejects_a_file_without_a_header(self):
        path = os.path.join(self.dir, "d.ply")
        with open(path, "wb") as fh:
            fh.write(b"nicht wirklich ply")
        with self.assertRaises(ValueError):
            make_viewer.read_ply(path)


class TestAnalyse(unittest.TestCase):
    def test_room_extent_matches_the_room(self):
        info = analyse(room_cloud())
        self.assertAlmostEqual(info["extent"][0], 6.0, delta=0.05)
        self.assertAlmostEqual(info["extent"][1], 4.0, delta=0.05)
        self.assertAlmostEqual(info["extent"][2], 2.6, delta=0.05)

    def test_counts_planes(self):
        info = analyse(room_cloud(planes=12))
        self.assertEqual(info["planes"], 12)

    def test_revolution_body_has_a_constant_contour(self):
        info = analyse(revolution_cloud())
        self.assertLess(info["sector_spread"], 0.01,
                        "ein Rotationskoerper streut praktisch nicht")
        self.assertLess(info["sector_ratio"], 1.05)
        self.assertAlmostEqual(info["median_radius"], 1.5, delta=0.05)

    def test_room_contour_varies_across_directions(self):
        info = analyse(room_cloud())
        self.assertGreater(info["sector_spread"], 0.25,
                           "ein Raum hat Ecken, die Kontur schwankt")
        self.assertGreater(info["sector_ratio"], 1.2)

    def test_coverage_never_exceeds_one(self):
        """Bei exakt 360.0 Grad entstuende sonst ein Sektor zu viel."""
        for cloud in (room_cloud(), revolution_cloud()):
            self.assertLessEqual(analyse(cloud)["sector_coverage"], 1.0)

    def test_half_covered_cloud_is_not_judged(self):
        """Der Fehler, der zu einer falschen Diagnose gefuehrt hat: nur die
        vordere Haelfte belegt, und die Kennzahl meldete Symmetrie."""
        half = [p for p in room_cloud() if p[1] >= 0]
        info = analyse(half)
        self.assertLess(info["sector_coverage"], 0.8)
        self.assertIsNone(info["sector_spread"])
        self.assertEqual(verdict(info)[0], "unklar")


class TestVerdict(unittest.TestCase):
    def test_flags_a_revolution_body_as_critical(self):
        level, title, text = verdict(
            analyse(revolution_cloud()))
        self.assertEqual(level, "kritisch")
        self.assertIn("Rotationskörper", title)
        # Die Erklaerung muss die Ursache nennen, nicht nur das Symptom.
        self.assertIn("Drehachse", text)

    def test_accepts_a_real_room(self):
        level, _, _ = verdict(analyse(room_cloud()))
        self.assertEqual(level, "plausibel")

    def test_reports_when_there_is_too_little_data(self):
        info = analyse([(0.0, 0.0, 5.0), (0.0, 0.0, -5.0)])
        self.assertIsNone(info["sector_spread"])
        self.assertEqual(verdict(info)[0], "unklar")


class TestBuild(unittest.TestCase):
    def test_page_has_no_leftover_placeholders(self):
        points = room_cloud(planes=6, per_plane=60)
        html = make_viewer.build(points, "T", analyse(points))
        self.assertNotIn("__", html.replace("__TITLE__", ""),
                         "alle Platzhalter muessen ersetzt sein")
        self.assertIn("<title>LiDAR-Scan T</title>", html)
        self.assertIn("cdnjs.cloudflare.com/ajax/libs/three.js", html)

    def test_points_survive_the_round_trip_into_the_page(self):
        import base64
        import re
        import struct
        points = room_cloud(planes=4, per_plane=40)
        html = make_viewer.build(points, "T", analyse(points))
        blob = base64.b64decode(re.search(r'const DATA = "([^"]+)"', html).group(1))
        self.assertEqual(len(blob) // 12, len(points))
        first = struct.unpack_from("<3f", blob, 0)
        for a, b in zip(first, points[0]):
            self.assertAlmostEqual(a, b, places=5)

    def test_page_is_pure_ascii(self):
        """Sonst zerlegt es die Umlaute.

        Beim Veroeffentlichen als Artifact bekommt die Seite einen Vorspann,
        der das <meta charset> weit hinter die ersten 1024 Byte schiebt - dort
        sucht der Browser es aber nur, und faellt sonst auf windows-1252
        zurueck. Die Seite darf sich deshalb nicht darauf verlassen.
        """
        points = room_cloud(planes=6, per_plane=60)
        html = make_viewer.build(points, "T", analyse(points))
        offending = [(i, c) for i, c in enumerate(html) if ord(c) >= 128]
        self.assertEqual(offending, [], "die Seite muss reines ASCII sein")

    def test_umlauts_survive_as_escapes(self):
        """Maskiert, aber nicht verloren - im Markup anders als im Skript."""
        points = room_cloud(planes=6, per_plane=60)
        html = make_viewer.build(points, "T", analyse(points))
        self.assertIn("Zur&#252;cksetzen", html)   # Zeichenreferenz im Markup
        self.assertIn("\\u00b0", html)             # Gradzeichen im <script>
        self.assertNotIn("&#252;cksetzen", html.split("<script")[-1])

    def test_to_ascii_leaves_plain_text_alone(self):
        self.assertEqual(make_viewer.to_ascii("<p>hallo</p>"), "<p>hallo</p>")

    def test_to_ascii_uses_the_right_escape_per_context(self):
        page = "<p>grün</p><script>var s = \"grün\";</script>"
        self.assertEqual(
            make_viewer.to_ascii(page),
            "<p>gr&#252;n</p><script>var s = \"gr\\u00fcn\";</script>")

    def test_page_carries_the_verdict(self):
        points = revolution_cloud(planes=8, per_plane=60)
        html = make_viewer.build(points, "T", analyse(points))
        self.assertIn('data-level="kritisch"', html)

    def test_level_key_stays_ascii_so_the_css_selector_matches(self):
        """Im <style> wirken keine Zeichenreferenzen - der Schluessel muss also
        ohne Umlaut auskommen, sonst faerbt sich der Streifen nicht."""
        for level in ("kritisch", "auffaellig", "plausibel"):
            self.assertIn(f'#finding[data-level="{level}"]', make_viewer.TEMPLATE)

    def test_to_ascii_refuses_non_ascii_in_the_stylesheet(self):
        with self.assertRaises(ValueError):
            make_viewer.to_ascii("<style>a::after{content:'grün'}</style>")

    def test_to_ascii_keeps_the_stylesheet_untouched(self):
        page = "<style>a{color:red}</style><p>grün</p>"
        self.assertEqual(make_viewer.to_ascii(page),
                         "<style>a{color:red}</style><p>gr&#252;n</p>")


if __name__ == "__main__":
    unittest.main()
