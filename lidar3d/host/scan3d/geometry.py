"""Umrechnung (Distanz, Scanwinkel, Gierwinkel) -> kartesische Punkte.

Aufbau: der 2D-LiDAR ist so montiert, dass seine Scanebene die Gierachse
enthaelt (Scanebene senkrecht). Ein Schrittmotor dreht ihn um diese Achse.
Damit ist eine Messung nichts anderes als eine Kugelkoordinate:

    alpha  = Polarwinkel, kommt vom LiDAR (0 deg = entlang +Z / nach oben)
    psi    = Azimut, kommt vom Schrittzaehler
    r      = Distanz

    x = r*sin(alpha)*cos(psi)
    y = r*sin(alpha)*sin(psi)
    z = r*cos(alpha)

Weil der LiDAR pro Umdrehung volle 360 Grad in seiner Ebene abdeckt, genuegt
ein Gierbereich von 180 Grad fuer die komplette Kugel. Deshalb braucht der
Aufbau keinen Schleifring.

In der Praxis sitzt das optische Zentrum nicht exakt auf der Gierachse. Die
beiden Versaetze (radial und axial) gehen vor der Drehung ein - siehe
docs/02-geometrie.md fuer die Kalibrierung.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Iterator, List, Tuple

Point = Tuple[float, float, float]


@dataclass(frozen=True)
class MountGeometry:
    """Einbaulage des LiDAR relativ zur Gierachse. Alle Laengen in mm."""

    #: Abstand des optischen Zentrums von der Gierachse, senkrecht dazu.
    offset_radial_mm: float = 0.0
    #: Hoehe des optischen Zentrums entlang der Gierachse ueber dem Ursprung.
    offset_axial_mm: float = 0.0
    #: LiDAR-Winkel, der nach oben (+Z) zeigt.
    alpha_zero_deg: float = 0.0
    #: -1, wenn der LiDAR-Winkel entgegen der gewuenschten Richtung laeuft.
    alpha_sign: int = 1
    #: Gierwinkel, der als Azimut 0 gilt.
    yaw_zero_deg: float = 0.0
    #: -1, wenn der Schrittmotor entgegen der gewuenschten Richtung dreht.
    yaw_sign: int = 1

    def __post_init__(self) -> None:
        if self.alpha_sign not in (1, -1):
            raise ValueError("alpha_sign muss +1 oder -1 sein")
        if self.yaw_sign not in (1, -1):
            raise ValueError("yaw_sign muss +1 oder -1 sein")


@dataclass(frozen=True)
class RangeFilter:
    """Gueltigkeitsfenster fuer Messungen. Laengen in mm."""

    min_mm: float = 150.0  # unterhalb der Blindzone (Datenblatt: 50 mm) plus Reserve
    max_mm: float = 30000.0

    def accepts(self, distance_mm: float) -> bool:
        return distance_mm > 0 and self.min_mm <= distance_mm <= self.max_mm


def to_cartesian(
    distance_mm: float,
    alpha_deg: float,
    yaw_deg: float,
    mount: MountGeometry = MountGeometry(),
) -> Point:
    """Eine Messung in Meter-Weltkoordinaten (Z = Gierachse, nach oben)."""
    a = math.radians(mount.alpha_sign * (alpha_deg - mount.alpha_zero_deg))
    # Koordinaten in der Scanebene: u radial von der Achse weg, w entlang der Achse.
    u = mount.offset_radial_mm + distance_mm * math.sin(a)
    w = mount.offset_axial_mm + distance_mm * math.cos(a)

    psi = math.radians(mount.yaw_sign * (yaw_deg - mount.yaw_zero_deg))
    return (
        u * math.cos(psi) / 1000.0,
        u * math.sin(psi) / 1000.0,
        w / 1000.0,
    )


def project(
    samples: Iterable[Tuple[float, float, float]],
    mount: MountGeometry = MountGeometry(),
    rng: RangeFilter = RangeFilter(),
) -> Iterator[Point]:
    """(distance_mm, alpha_deg, yaw_deg)-Tripel in Punkte umrechnen und filtern."""
    for distance_mm, alpha_deg, yaw_deg in samples:
        if not rng.accepts(distance_mm):
            continue
        yield to_cartesian(distance_mm, alpha_deg, yaw_deg, mount)


def sweep_plan(
    yaw_span_deg: float,
    yaw_step_deg: float,
    scan_hz: float = 10.0,
    samples_per_second: float = 32000.0,
) -> dict:
    """Dauer, Punktzahl und Winkelaufloesung eines Sweeps vorausberechnen.

    ``yaw_step_deg`` ist der gewuenschte Abstand zwischen zwei Scanebenen.
    """
    if yaw_step_deg <= 0:
        raise ValueError("yaw_step_deg muss positiv sein")
    planes = yaw_span_deg / yaw_step_deg
    duration_s = planes / scan_hz
    samples_per_plane = samples_per_second / scan_hz
    return {
        "planes": planes,
        "duration_s": duration_s,
        "yaw_rate_deg_s": yaw_span_deg / duration_s if duration_s else 0.0,
        "samples_per_plane": samples_per_plane,
        "total_samples": planes * samples_per_plane,
        "in_plane_resolution_deg": 360.0 / samples_per_plane,
    }


def voxel_downsample(points: Iterable[Point], voxel_m: float) -> List[Point]:
    """Ein Punkt pro Voxel (erster Treffer gewinnt).

    Ohne das ist die Punktwolke an den Polen der Gierachse extrem dicht und am
    Aequator duenn - genau das erzeugt die typische Sanduhr-Form.
    """
    if voxel_m <= 0:
        raise ValueError("voxel_m muss positiv sein")
    seen = set()
    out: List[Point] = []
    for p in points:
        key = (
            math.floor(p[0] / voxel_m),
            math.floor(p[1] / voxel_m),
            math.floor(p[2] / voxel_m),
        )
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
    return out
