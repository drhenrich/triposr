import math
import unittest

from scan3d.geometry import (
    MountGeometry,
    RangeFilter,
    project,
    sweep_plan,
    to_cartesian,
    voxel_downsample,
)


class TestToCartesian(unittest.TestCase):
    def test_straight_up_is_plus_z(self):
        x, y, z = to_cartesian(1000.0, 0.0, 0.0)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(z, 1.0, places=9)

    def test_straight_down_is_minus_z(self):
        _, _, z = to_cartesian(1000.0, 180.0, 123.0)
        self.assertAlmostEqual(z, -1.0, places=9)

    def test_horizontal_follows_yaw(self):
        x, y, z = to_cartesian(2000.0, 90.0, 0.0)
        self.assertAlmostEqual((x, y, z)[0], 2.0, places=9)
        self.assertAlmostEqual(z, 0.0, places=9)

        x, y, _ = to_cartesian(2000.0, 90.0, 90.0)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 2.0, places=9)

    def test_180_degree_yaw_covers_the_back_half(self):
        """Kernpunkt des Aufbaus: 180 deg Gieren decken die volle Kugel ab.

        alpha=270 bei yaw=0 zeigt in dieselbe Richtung wie alpha=90 bei yaw=180.
        """
        a = to_cartesian(1000.0, 270.0, 0.0)
        b = to_cartesian(1000.0, 90.0, 180.0)
        for u, v in zip(a, b):
            self.assertAlmostEqual(u, v, places=9)

    def test_radial_offset_shifts_before_rotation(self):
        mount = MountGeometry(offset_radial_mm=50.0)
        x, y, z = to_cartesian(1000.0, 90.0, 0.0, mount)
        self.assertAlmostEqual(x, 1.05, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(z, 0.0, places=9)

    def test_axial_offset_shifts_z(self):
        mount = MountGeometry(offset_axial_mm=-25.0)
        _, _, z = to_cartesian(1000.0, 0.0, 0.0, mount)
        self.assertAlmostEqual(z, 0.975, places=9)

    def test_alpha_sign_mirrors_the_scan_plane(self):
        mount = MountGeometry(alpha_sign=-1)
        x, _, z = to_cartesian(1000.0, 90.0, 0.0, mount)
        self.assertAlmostEqual(x, -1.0, places=9)
        self.assertAlmostEqual(z, 0.0, places=9)

    def test_alpha_zero_rotates_the_scan_plane(self):
        mount = MountGeometry(alpha_zero_deg=90.0)
        _, _, z = to_cartesian(1000.0, 90.0, 0.0, mount)
        self.assertAlmostEqual(z, 1.0, places=9)

    def test_invalid_signs_rejected(self):
        with self.assertRaises(ValueError):
            MountGeometry(alpha_sign=0)
        with self.assertRaises(ValueError):
            MountGeometry(yaw_sign=2)

    def test_flat_wall_stays_flat(self):
        """Wand bei x = 3 m: alle Messungen muessen exakt auf x = 3 landen."""
        for alpha in (60.0, 75.0, 90.0, 105.0, 120.0):
            r = 3000.0 / math.sin(math.radians(alpha))
            x, _, _ = to_cartesian(r, alpha, 0.0)
            self.assertAlmostEqual(x, 3.0, places=6)

    def test_uncalibrated_radial_offset_bends_a_flat_wall(self):
        """Begruendung fuer die Kalibrierung: 40 mm Versatz kruemmt die Wand."""
        mount = MountGeometry(offset_radial_mm=40.0)
        xs = []
        for alpha in (60.0, 90.0, 120.0):
            r = 3000.0 / math.sin(math.radians(alpha))
            xs.append(to_cartesian(r, alpha, 0.0, mount)[0])
        self.assertAlmostEqual(xs[1] - xs[0], 0.0, delta=1e-9)  # symmetrisch
        self.assertGreater(max(xs) - 3.0, 0.039)


class TestRangeFilter(unittest.TestCase):
    def test_rejects_zero_and_out_of_range(self):
        f = RangeFilter(min_mm=150.0, max_mm=30000.0)
        self.assertFalse(f.accepts(0))
        self.assertFalse(f.accepts(100))
        self.assertFalse(f.accepts(30001))
        self.assertTrue(f.accepts(150))
        self.assertTrue(f.accepts(30000))


class TestProject(unittest.TestCase):
    def test_filters_and_converts(self):
        samples = [
            (0.0, 0.0, 0.0),  # kein Echo
            (1000.0, 0.0, 0.0),
            (50.0, 0.0, 0.0),  # in der Blindzone
        ]
        points = list(project(samples))
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][2], 1.0, places=9)


class TestSweepPlan(unittest.TestCase):
    def test_180_degrees_at_one_degree_steps(self):
        info = sweep_plan(180.0, 1.0)
        self.assertEqual(info["planes"], 180.0)
        # Je Ebene eine Umdrehung (100 ms) plus Fahrt und Einrasten (50 ms).
        self.assertAlmostEqual(info["seconds_per_plane"], 0.15)
        self.assertAlmostEqual(info["duration_s"], 27.0)
        self.assertAlmostEqual(info["samples_per_plane"], 3200.0)
        self.assertAlmostEqual(info["total_samples"], 576000.0)
        self.assertAlmostEqual(info["in_plane_resolution_deg"], 0.1125)

    def test_overhead_is_configurable(self):
        """Ohne Zuschlag bleibt die reine Messzeit uebrig."""
        info = sweep_plan(180.0, 1.0, plane_overhead_s=0.0)
        self.assertAlmostEqual(info["duration_s"], 18.0)

    def test_finer_step_takes_proportionally_longer(self):
        coarse = sweep_plan(180.0, 1.0)
        fine = sweep_plan(180.0, 0.5)
        self.assertAlmostEqual(fine["duration_s"], 2 * coarse["duration_s"])

    def test_rejects_zero_step(self):
        with self.assertRaises(ValueError):
            sweep_plan(180.0, 0.0)


class TestVoxelDownsample(unittest.TestCase):
    def test_collapses_neighbours(self):
        points = [(0.0, 0.0, 0.0), (0.01, 0.01, 0.01), (1.0, 0.0, 0.0)]
        self.assertEqual(len(voxel_downsample(points, 0.05)), 2)

    def test_handles_negative_coordinates(self):
        points = [(-0.01, 0.0, 0.0), (-0.02, 0.0, 0.0), (0.01, 0.0, 0.0)]
        # -0.01 und -0.02 liegen im selben Voxel [-0.05, 0), 0.01 in [0, 0.05)
        self.assertEqual(len(voxel_downsample(points, 0.05)), 2)

    def test_rejects_zero_size(self):
        with self.assertRaises(ValueError):
            voxel_downsample([], 0.0)


if __name__ == "__main__":
    unittest.main()
