"""Schaetzung der Nulllage des Scanwinkels.

Hintergrund ist ein echter Fehlscan: der C1 stand richtig hochkant, aber
seine Winkelmarke zeigt hochkant zur Seite statt nach oben. Mit
``alpha_zero = 0`` wurde daraus der Abstand zur Decke als Radius verrechnet -
die Wolke war 5,7 m hoch, 1,7 m breit und sah wie ein runder Raum aus.

Die Tests fahren genau diesen Weg nach: bekannter Raum, kuenstlich verdreht,
Schaetzung muss den Versatz zurueckliefern und die Korrektur den Raum
wiederherstellen.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scan3d.alignment import (  # noqa: E402
    deviation_deg, estimate_alpha_zero, plane_coordinates, rotate_cloud)
from scan3d.geometry import to_cartesian  # noqa: E402
from scan3d.quality import analyse, verdict  # noqa: E402
from scan3d.roomsim import RoomSimulator  # noqa: E402

ROOM = (6.0, 4.0, 2.6)


def scan_room(alpha_offset_deg=0.0, planes=36, per_plane=400):
    """Einen Raum scannen, wobei der LiDAR seine Winkel verdreht zaehlt."""
    sim = RoomSimulator(room_m=ROOM, noise_mm=5.0, dropout_rate=0.0)
    points = []
    for p in range(planes):
        yaw = p * 180.0 / planes
        for k in range(per_plane):
            alpha = 360.0 * k / per_plane
            distance = sim.distance_mm(alpha, yaw)
            if distance > 150:
                # Der Scanner meldet den verdrehten Winkel, die Auswertung
                # nimmt ihn fuer bare Muenze - genau der Fehlerfall.
                points.append(to_cartesian(distance, alpha + alpha_offset_deg, yaw))
    return points


class TestPlaneCoordinates(unittest.TestCase):
    def test_round_trip_through_the_cloud(self):
        """Aus der Wolke muessen sich (u, w, psi) exakt zurueckholen lassen."""
        for yaw in (0.0, 37.0, 91.0, 173.0):
            # 0 und 180 Grad fehlen bewusst: dort liegt der Punkt auf der
            # Drehachse, und aus einem Punkt auf der Achse laesst sich der
            # Gierwinkel grundsaetzlich nicht zurueckholen.
            for alpha in (45.0, 120.0, 200.0, 300.0):
                point = to_cartesian(2500.0, alpha, yaw)
                (u, w, psi), = plane_coordinates([point])
                self.assertAlmostEqual(math.degrees(psi), yaw, places=3)
                self.assertAlmostEqual(u, 2.5 * math.sin(math.radians(alpha)),
                                       places=5)
                self.assertAlmostEqual(w, 2.5 * math.cos(math.radians(alpha)),
                                       places=5)

    def test_rotating_by_zero_changes_nothing(self):
        cloud = scan_room(planes=6, per_plane=60)
        for before, after in zip(cloud, rotate_cloud(cloud, 0.0)):
            for a, b in zip(before, after):
                self.assertAlmostEqual(a, b, places=6)


class TestEstimate(unittest.TestCase):
    def test_finds_the_offset_that_was_introduced(self):
        for offset in (0.0, 30.0, 90.0, 150.0):
            info = estimate_alpha_zero(scan_room(offset))
            self.assertTrue(info["confident"], f"Versatz {offset}")
            # Gesucht ist genau der Wert, der in alpha_zero gehoert - der
            # Winkel wird ja beim Umrechnen abgezogen. Verglichen wird modulo
            # 180, weil ein halber Umlauf die Wolke nur auf den Kopf stellt.
            self.assertLess(deviation_deg(info["alpha_zero_deg"], offset), 3.0,
                            f"Versatz {offset} nicht gefunden: {info}")

    def test_correcting_restores_the_room(self):
        cloud = scan_room(90.0)
        info = estimate_alpha_zero(cloud)
        fixed = rotate_cloud(cloud, info["alpha_zero_deg"])
        extent = analyse(fixed)["extent"]
        # Hoehe wieder Raumhoehe, Breite wieder Raumbreite.
        self.assertAlmostEqual(extent[2], ROOM[2], delta=0.1)
        self.assertAlmostEqual(max(extent[0], extent[1]), ROOM[0], delta=0.1)

    def test_a_revolution_body_offers_no_flat_surface(self):
        """Ohne Decke und Boden darf die Schaetzung nichts behaupten."""
        cloud = [to_cartesian(1500.0, 360.0 * k / 200, p * 5.0)
                 for p in range(36) for k in range(200)]
        self.assertFalse(estimate_alpha_zero(cloud)["confident"])

    def test_too_few_points_are_refused(self):
        info = estimate_alpha_zero([(0.0, 0.0, 1.0)] * 10)
        self.assertFalse(info["confident"])
        self.assertIsNone(info["alpha_zero_deg"])


class TestDeviation(unittest.TestCase):
    def test_half_a_turn_is_no_deviation(self):
        # 180 Grad stellen den Raum auf den Kopf, verfaelschen die Form aber
        # nicht - deshalb zaehlt der Versatz nur modulo 180.
        self.assertAlmostEqual(deviation_deg(180.0, 0.0), 0.0)
        self.assertAlmostEqual(deviation_deg(90.0, 0.0), 90.0)
        self.assertAlmostEqual(deviation_deg(170.0, 0.0), 10.0)
        self.assertAlmostEqual(deviation_deg(95.0, 90.0), 5.0)


class TestVerdictCatchesIt(unittest.TestCase):
    """Der Befund muss den Fall melden - vorher hiess er faelschlich 'plausibel'."""

    def test_flags_a_wrong_zero_position(self):
        level, headline, text = verdict(analyse(scan_room(90.0)))
        self.assertEqual(level, "kritisch")
        self.assertIn("Nulllage", headline)
        self.assertIn("alpha_zero", text)

    def test_leaves_a_correct_scan_alone(self):
        level, _, _ = verdict(analyse(scan_room(0.0)))
        self.assertEqual(level, "plausibel")

    def test_a_small_offset_is_tolerated(self):
        """Ein paar Grad Schiefstand sind normal und kein Fehler."""
        level, _, _ = verdict(analyse(scan_room(5.0)))
        self.assertEqual(level, "plausibel")


if __name__ == "__main__":
    unittest.main()
