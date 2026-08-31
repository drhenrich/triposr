"""Synthetischer LiDAR in einem Quader-Raum.

Damit laeuft die Streamlit-App ohne angeschlossene Hardware, und die
Geometriekette laesst sich gegen eine bekannte Wahrheit pruefen: der Raum ist
ein achsenparalleler Quader, also muss die Punktwolke exakt dessen Masse
zurueckliefern.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Tuple

from .rplidar import Sample


@dataclass
class RoomSimulator:
    """Der Sensor sitzt im Ursprung, der Raum ist um ihn herum zentriert."""

    #: Raummasse in Metern (x, y, z).
    room_m: Tuple[float, float, float] = (6.0, 4.0, 2.6)
    #: Messungen je Umdrehung. C1: 5000/s bei 10 Hz = 500.
    samples_per_revolution: int = 500
    #: Streuung der Distanzmessung in mm.
    noise_mm: float = 15.0
    #: Anteil der Messungen ohne Echo (dunkle Flaechen, Glas, Kanten).
    dropout_rate: float = 0.02
    seed: int = 1

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def distance_mm(self, alpha_deg: float, yaw_deg: float) -> float:
        """Strahl aus dem Ursprung gegen den Quader schneiden. 0 = kein Echo."""
        if self._rng.random() < self.dropout_rate:
            return 0.0

        a = math.radians(alpha_deg)
        psi = math.radians(yaw_deg)
        # Scanebene enthaelt die Gierachse (Z), genau wie im echten Aufbau.
        dx = math.sin(a) * math.cos(psi)
        dy = math.sin(a) * math.sin(psi)
        dz = math.cos(a)

        half = (self.room_m[0] / 2, self.room_m[1] / 2, self.room_m[2] / 2)
        t = math.inf
        for d, h in zip((dx, dy, dz), half):
            if abs(d) > 1e-9:
                t = min(t, (h if d > 0 else -h) / d)
        if not math.isfinite(t):
            return 0.0
        return max(0.0, t * 1000.0 + self._rng.gauss(0.0, self.noise_mm))

    def revolution(self, yaw_deg: float = 0.0) -> List[Sample]:
        """Eine vollstaendige Umdrehung, erste Messung mit Umlaufmarke."""
        n = self.samples_per_revolution
        out: List[Sample] = []
        for i in range(n):
            alpha = 360.0 * i / n
            out.append(
                Sample(
                    angle_deg=alpha,
                    distance_mm=self.distance_mm(alpha, yaw_deg),
                    new_revolution=(i == 0),
                    quality=47,
                )
            )
        return out
