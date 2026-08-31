"""Nulllage des LiDAR-Winkels aus einer fertigen Wolke schaetzen.

Ein 2D-LiDAR zaehlt seine Winkel ab einer Marke am Gehaeuse. Liegt das
Geraet flach, zeigt diese Marke waagerecht nach vorn. Stellt man es fuer den
3D-Scan hochkant, zeigt sie nicht nach oben, sondern zur Seite - beim C1 um
rund 90 Grad versetzt.

Wer diesen Versatz nicht eintraegt, verrechnet den Abstand zur Decke als
Radius. Um die Achse gedreht wird daraus ein Zylinder mit genau diesem
Radius: der "Rundraum". Gleichzeitig landet die tatsaechliche Raumtiefe in
der Hoehe, die Wolke wird also viel zu hoch und viel zu schmal.

Geschaetzt wird ueber die einzige Eigenschaft, die in jedem Zimmer gilt:
Decke und Boden sind waagerecht. Bei richtiger Nulllage fallen deshalb sehr
viele Punkte auf exakt dieselbe Hoehe; bei falscher verschmieren sie.

Uebrig bleibt eine echte Mehrdeutigkeit: 0 Grad und 180 Grad Versatz
erzeugen dieselbe Wolke, nur auf den Kopf gestellt. Das laesst sich aus den
Messwerten allein nicht entscheiden - deshalb liefert die Schaetzung beide
Werte und sagt dazu, wo die groesste ebene Flaeche jeweils liegt.
"""

from __future__ import annotations

import collections
import math
from typing import Dict, Iterable, List, Sequence, Tuple

Point = Tuple[float, float, float]

#: Klassenbreite der Hoehenzaehlung. Etwas groesser als das Rauschen des C1,
#: damit eine ebene Decke wirklich in eine einzige Klasse faellt.
BIN_M = 0.02

#: Ab hier gilt die Nulllage als klar falsch eingestellt.
TOLERANCE_DEG = 15.0

#: So viele Punkte genuegen fuer die Suche - der Rest kostet nur Zeit.
SAMPLE_SIZE = 8000
#: Schrittweite des groben Durchgangs. Eine ebene Decke ist ueber mehrere Grad
#: hinweg noch als Haeufung erkennbar, feiner muss der erst werden.
COARSE_STEP_DEG = 5.0


def plane_coordinates(points: Iterable[Point]) -> List[Tuple[float, float, float]]:
    """Aus der Wolke die Koordinaten in der Scanebene zurueckgewinnen.

    Rueckgabe je Punkt: (u, w, psi) - u radial von der Drehachse weg, w
    entlang der Achse, psi der Gierwinkel im Bogenmass. Das geht nur, weil
    die Drehung um die Z-Achse laeuft: der Radius bleibt erhalten, und das
    Vorzeichen von u steckt darin, ob der Punkt auf der Ebenenrichtung oder
    auf deren Gegenrichtung liegt.
    """
    out: List[Tuple[float, float, float]] = []
    for x, y, z in points:
        phi = math.degrees(math.atan2(y, x)) % 360.0
        plane = phi % 180.0
        # Liegt der Punkt in der Ebenenrichtung selbst oder gegenueber?
        forward = abs(((phi - plane + 180.0) % 360.0) - 180.0) < 0.5
        radius = math.hypot(x, y)
        out.append((radius if forward else -radius, z, math.radians(plane)))
    return out


def rotate_cloud(points: Iterable[Point], delta_deg: float) -> List[Point]:
    """Die Wolke so umrechnen, als waere alpha_zero um delta groesser gewesen.

    Jede Scanebene wird in sich gedreht, der Gierwinkel bleibt unberuehrt.
    Damit laesst sich eine bereits aufgenommene Datei nachtraeglich richtig
    stellen, ohne den Scan zu wiederholen.
    """
    a = math.radians(delta_deg)
    ca, sa = math.cos(a), math.sin(a)
    out: List[Point] = []
    for u, w, psi in plane_coordinates(points):
        radial = u * ca - w * sa
        axial = u * sa + w * ca
        out.append((radial * math.cos(psi), radial * math.sin(psi), axial))
    return out


def _flat_surface(plane_coords: Sequence[Tuple[float, float, float]],
                  delta_deg: float) -> Tuple[int, float]:
    """Groesste ebene waagerechte Flaeche bei diesem Versatz: (Punkte, Hoehe)."""
    a = math.radians(delta_deg)
    ca, sa = math.cos(a), math.sin(a)
    heights = collections.Counter(
        int(round((u * sa + w * ca) / BIN_M)) for u, w, _ in plane_coords)
    if not heights:
        return 0, 0.0
    key, count = heights.most_common(1)[0]
    return count, key * BIN_M


def estimate_alpha_zero(points: Sequence[Point], step_deg: float = 1.0) -> Dict:
    """Den wahrscheinlichsten Winkelversatz bestimmen.

    Ergebnis:
        alpha_zero_deg   bester Versatz in [0, 180)
        mirrored_deg     derselbe Wert plus 180 - dieselbe Wolke, auf dem Kopf
        surface_points   Punkte auf der groessten ebenen Flaeche
        surface_height_m Hoehe dieser Flaeche ueber dem Sensor (bei alpha_zero_deg)
        sharpness        wie deutlich der beste Versatz herausragt (1 = gar nicht)
        confident        True, wenn die Schaetzung tragfaehig ist
    """
    coords = plane_coordinates(points)
    if len(coords) < 200 or step_deg <= 0:
        return {"alpha_zero_deg": None, "mirrored_deg": None,
                "surface_points": 0, "surface_height_m": None,
                "sharpness": None, "confident": False}

    # Zwei Durchgaenge auf einer Stichprobe: erst grob den ganzen Halbkreis,
    # dann fein um den besten Wert herum. Die Suche laeuft bei jedem Neuaufbau
    # der Seite, und eine volle Wolke Punkt fuer Punkt 180-mal durchzurechnen
    # waere dafuer viel zu langsam.
    sample = coords[::max(1, len(coords) // SAMPLE_SIZE)]

    coarse = max((_flat_surface(sample, i * COARSE_STEP_DEG)[0], i * COARSE_STEP_DEG)
                 for i in range(int(180.0 / COARSE_STEP_DEG)))[1]
    span = int(COARSE_STEP_DEG / step_deg)
    fine = [(coarse + k * step_deg) % 180.0 for k in range(-span, span + 1)]
    best = max((_flat_surface(sample, d)[0], d) for d in fine)[1]

    # Vergleichsmass fuer die Deutlichkeit: wie gut schneidet ein beliebiger
    # falscher Winkel ab? Ohne echte ebene Flaeche liegt der beste kaum
    # darueber, und dann darf die Schaetzung nichts behaupten.
    counts = sorted(_flat_surface(sample, i * COARSE_STEP_DEG)[0]
                    for i in range(int(180.0 / COARSE_STEP_DEG)))
    typical = counts[len(counts) // 2] or 1
    sharpness = _flat_surface(sample, best)[0] / typical

    # Die gemeldeten Zahlen kommen aus der ganzen Wolke, nicht der Stichprobe.
    best_count, height = _flat_surface(coords, best)
    return {
        "alpha_zero_deg": best,
        "mirrored_deg": best + 180.0,
        "surface_points": best_count,
        "surface_height_m": height,
        "sharpness": sharpness,
        # Eine echte Decke haelt einen spuerbaren Anteil der Wolke und hebt
        # sich klar von den falschen Versaetzen ab.
        "confident": sharpness >= 3.0 and best_count >= 0.02 * len(coords),
    }


def deviation_deg(estimated_deg: float, configured_deg: float = 0.0) -> float:
    """Abweichung der Einstellung von der Schaetzung, 0 bis 90 Grad.

    Modulo 180, weil ein Versatz von 180 Grad die Wolke nur auf den Kopf
    stellt und die Raumform nicht verfaelscht.
    """
    delta = (estimated_deg - configured_deg) % 180.0
    return min(delta, 180.0 - delta)
