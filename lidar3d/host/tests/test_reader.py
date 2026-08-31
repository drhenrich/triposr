"""Hintergrundleser und Raum-Simulator.

Das ist die Kette, auf der die Streamlit-App sitzt: Simulator -> Leser ->
Geometrie. Streamlit selbst laesst sich hier nicht starten, diese Schicht
darunter schon.
"""

import math
import unittest

from scan3d.geometry import MountGeometry, RangeFilter, to_cartesian
from scan3d.reader import LidarReader
from scan3d.roomsim import RoomSimulator


class TestRoomSimulator(unittest.TestCase):
    def test_revolution_has_marker_only_at_the_start(self):
        sim = RoomSimulator(samples_per_revolution=500, dropout_rate=0.0)
        rev = sim.revolution()
        self.assertEqual(len(rev), 500)
        self.assertTrue(rev[0].new_revolution)
        self.assertFalse(any(s.new_revolution for s in rev[1:]))

    def test_angles_cover_the_full_circle(self):
        sim = RoomSimulator(samples_per_revolution=500, dropout_rate=0.0)
        rev = sim.revolution()
        self.assertAlmostEqual(rev[0].angle_deg, 0.0)
        self.assertAlmostEqual(rev[-1].angle_deg, 360.0 * 499 / 500)

    def test_distances_match_the_room(self):
        """Ohne Rauschen muss der Strahl exakt die Wandflaeche treffen."""
        sim = RoomSimulator(room_m=(6.0, 4.0, 2.6), noise_mm=0.0, dropout_rate=0.0)
        # alpha = 0 zeigt entlang der Gierachse nach oben -> halbe Raumhoehe.
        self.assertAlmostEqual(sim.distance_mm(0.0, 0.0), 1300.0, places=3)
        # alpha = 90 bei yaw = 0 zeigt entlang +X -> halbe Raumlaenge.
        self.assertAlmostEqual(sim.distance_mm(90.0, 0.0), 3000.0, places=3)
        # alpha = 90 bei yaw = 90 zeigt entlang +Y -> halbe Raumbreite.
        self.assertAlmostEqual(sim.distance_mm(90.0, 90.0), 2000.0, places=3)

    def test_dropouts_report_no_echo(self):
        sim = RoomSimulator(dropout_rate=1.0)
        self.assertTrue(all(s.distance_mm == 0.0 for s in sim.revolution()))

    def test_is_reproducible_for_a_given_seed(self):
        a = RoomSimulator(seed=7).revolution()
        b = RoomSimulator(seed=7).revolution()
        self.assertEqual([s.distance_mm for s in a], [s.distance_mm for s in b])


class TestLidarReader(unittest.TestCase):
    def test_delivers_revolutions_from_the_simulator(self):
        reader = LidarReader(simulate=True, scan_hz=50.0)
        reader.start()
        try:
            revolution = reader.wait_for_revolution(timeout=3.0)
            self.assertIsNotNone(revolution)
            self.assertEqual(len(revolution), 500)
            stats = reader.stats()
            self.assertGreaterEqual(stats.revolutions, 1)
            self.assertIsNone(stats.error)
            self.assertEqual(stats.info.get("model"), "Simulation")
        finally:
            reader.stop()
        self.assertFalse(reader.running)

    def test_stop_is_idempotent(self):
        reader = LidarReader(simulate=True, scan_hz=50.0)
        reader.start()
        reader.stop()
        reader.stop()
        self.assertFalse(reader.running)

    def test_serial_failure_is_reported_not_raised(self):
        """Ein toter Port darf die Oberflaeche nicht mitreissen."""
        reader = LidarReader(port="/dev/does-not-exist", simulate=False)
        reader.start()
        try:
            deadline_reached = reader.wait_for_revolution(timeout=2.0)
            self.assertIsNone(deadline_reached)
            self.assertIsNotNone(reader.stats().error)
        finally:
            reader.stop()

    def test_statistics_count_valid_measurements(self):
        reader = LidarReader(simulate=True, scan_hz=50.0)
        reader.start()
        try:
            reader.wait_for_revolution(timeout=3.0)
            stats = reader.stats()
            self.assertGreater(stats.samples, 0)
            self.assertGreater(stats.valid, 0)
            self.assertLessEqual(stats.valid, stats.samples)
        finally:
            reader.stop()


class TestSimulatedCloudIsGeometricallySound(unittest.TestCase):
    """Ende zu Ende: Simulator -> Geometrie -> Punktwolke."""

    def test_room_reconstructs_to_its_own_dimensions(self):
        room = (6.0, 4.0, 2.6)
        sim = RoomSimulator(room_m=room, samples_per_revolution=500,
                            noise_mm=0.0, dropout_rate=0.0)
        rng = RangeFilter(min_mm=150.0, max_mm=12000.0)
        mount = MountGeometry()

        points = []
        for plane in range(36):            # 180 Grad in 5-Grad-Schritten
            yaw = plane * 5.0
            for i in range(500):
                alpha = 360.0 * i / 500
                distance = sim.distance_mm(alpha, yaw)
                if not rng.accepts(distance):
                    continue
                points.append(to_cartesian(distance, alpha, yaw, mount))

        self.assertGreater(len(points), 10000)
        for axis, extent in enumerate(room):
            values = [p[axis] for p in points]
            self.assertAlmostEqual(max(values), extent / 2, places=3)
            self.assertAlmostEqual(min(values), -extent / 2, places=3)


if __name__ == "__main__":
    unittest.main()
