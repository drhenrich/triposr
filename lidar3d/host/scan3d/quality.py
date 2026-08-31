"""Taugt der Scan etwas? Kennzahlen und ein Befund dazu.

Die wichtigste Frage bei diesem Aufbau ist nicht die Genauigkeit, sondern ob
die Drehung ueberhaupt etwas beigetragen hat. Sitzt der LiDAR falsch herum -
Scanebene senkrecht zur Drehachse statt durch sie hindurch -, misst jede
Ebene denselben Ring, und heraus kommt ein Rotationskoerper. Der sieht auf
den ersten Blick nach einer Punktwolke aus, enthaelt aber keine Raumform.

Genau darauf zielt ``sector_spread``: der Abstand zur Drehachse, verglichen
ueber die Himmelsrichtungen. Bei einem Raum schwankt er (Ecken), bei einem
Rotationskoerper nicht.
"""

from __future__ import annotations

import math
import statistics as stat
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float, float]

#: Unterhalb dieser Streuung (in m) ist die Wolke praktisch rund.
REVOLUTION_THRESHOLD_M = 0.05
#: Darunter ist sie zumindest auffaellig gleichmaessig.
SUSPICIOUS_THRESHOLD_M = 0.25


SECTOR_WIDTH_DEG = 15
#: Unter dieser Abdeckung laesst sich ueber die Form nichts sagen.
MIN_COVERAGE = 0.8


def analyse(points: Sequence[Point]) -> Dict:
    """Kennzahlen einer Punktwolke in Weltkoordinaten (Z = Drehachse).

    Die Form wird ueber die **Kontur** beurteilt: je Himmelsrichtung der
    groesste Abstand zur Drehachse, ueber alle Hoehen. Eine fruehere Fassung
    nahm dafuer den Median in einer duennen waagerechten Scheibe - das ging
    schief, weil so eine Scheibe leicht nur die vordere Haelfte des
    Gierbereichs enthaelt und dann Symmetrie meldet, wo keine ist. Die Kontur
    ueber alle Hoehen hat dieses Problem nicht, und ``sector_coverage`` sagt,
    ob ueberhaupt genug Richtungen belegt sind.
    """
    if not points:
        raise ValueError("die Wolke ist leer")

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    radii = sorted(math.hypot(math.hypot(x, y), z) for x, y, z in points)

    # Der Azimut ist auf 0..180 gefaltet: der LiDAR misst in seiner Ebene
    # volle 360 Grad, der radiale Anteil darf also negativ werden.
    planes = {round(math.degrees(math.atan2(y, x)) % 180) for x, y, _ in points}

    sector_count = 360 // SECTOR_WIDTH_DEG
    reach: Dict[int, float] = {}
    for x, y, _ in points:
        # Der Modulo kann bei winzigen negativen Winkeln exakt 360.0 liefern;
        # ohne das zweite Modulo entstuende ein Sektor zu viel.
        sector = (int(math.degrees(math.atan2(y, x)) % 360)
                  // SECTOR_WIDTH_DEG) % sector_count
        distance = math.hypot(x, y)
        if distance > reach.get(sector, 0.0):
            reach[sector] = distance

    coverage = len(reach) / sector_count
    values = list(reach.values())
    spread = stat.pstdev(values) if coverage >= MIN_COVERAGE else None
    ratio = (max(values) / min(values)
             if coverage >= MIN_COVERAGE and min(values) > 1e-6 else None)

    return {
        "count": len(points),
        "planes": len(planes),
        "extent": [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)],
        "bounds": [[min(xs), max(xs)], [min(ys), max(ys)], [min(zs), max(zs)]],
        "median_radius": radii[len(radii) // 2],
        "max_radius": radii[-1],
        "far_share": sum(1 for r in radii if r > 4.0) / len(radii),
        "sector_coverage": coverage,
        "sector_spread": spread,
        "sector_ratio": ratio,
        "sector_reach": [reach.get(k) for k in range(sector_count)],
    }


def verdict(info: Dict) -> Tuple[str, str, str]:
    """(Stufe, Ueberschrift, Erklaerung). Stufe: unklar/kritisch/auffaellig/plausibel."""
    spread: Optional[float] = info["sector_spread"]
    ratio: Optional[float] = info["sector_ratio"]
    coverage: float = info["sector_coverage"]

    if spread is None:
        return ("unklar", "Zu wenige Richtungen belegt",
                f"Nur {coverage * 100:.0f} % der Himmelsrichtungen enthalten "
                "Punkte. Über die Form lässt sich so nichts sagen — es fehlen "
                "Scanebenen, oder der Gierbereich war zu klein. 180° decken die "
                "volle Kugel ab.")

    if spread < REVOLUTION_THRESHOLD_M and (ratio is None or ratio < 1.1):
        return ("kritisch", "Die Wolke ist ein Rotationskörper",
                f"Die Kontur reicht in jede Himmelsrichtung gleich weit "
                f"(Schwankung {spread * 1000:.0f} mm). Ein echter Raum ist nicht "
                "rund. Jede Scanebene hat praktisch dasselbe gemessen — die "
                "Drehung hat nichts beigetragen. Meist liegt das an der "
                "Einbaulage: die Scanebene muss die Drehachse enthalten. Liegt "
                "das Gerät flach und dreht sich um die Hochachse, misst es immer "
                "wieder denselben waagerechten Ring.")

    if spread < SUSPICIOUS_THRESHOLD_M:
        return ("auffällig", "Sehr gleichmäßige Kontur",
                f"Die Kontur schwankt nur um {spread * 1000:.0f} mm "
                f"(weiteste zu engste Richtung {ratio:.1f}:1). Das kann an einer "
                "engen, symmetrischen Umgebung liegen. Die Draufsicht klärt es: "
                "eine Raumkontur hat Ecken.")

    return ("plausibel", "Die Wolke zeigt Struktur",
            f"Die Kontur reicht je nach Richtung {ratio:.1f}-mal unterschiedlich "
            f"weit (Schwankung {spread * 1000:.0f} mm), bei "
            f"{coverage * 100:.0f} % belegten Richtungen. Die Drehung hat also "
            "verschiedene Geometrie erfasst, wie man es von einem Raum erwartet.")
