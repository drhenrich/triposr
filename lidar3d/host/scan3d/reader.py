"""Liest Umdrehungen im Hintergrund, damit die Oberflaeche nicht blockiert.

Bewusst ohne Streamlit-Bezug: so laesst sich der Leser gegen den simulierten
Raum testen, ohne eine Oberflaeche zu starten.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .roomsim import RoomSimulator
from .rplidar import Sample


@dataclass
class ReaderStats:
    revolutions: int = 0
    samples: int = 0
    valid: int = 0
    resyncs: int = 0
    rate_hz: float = 0.0
    error: Optional[str] = None
    info: dict = field(default_factory=dict)


class LidarReader:
    """Eigener Thread, der laufend die jeweils letzte Umdrehung bereithaelt.

    Die Oberflaeche fragt nur ab; sie wartet nie auf die serielle
    Schnittstelle.
    """

    def __init__(self, port: str = "", baudrate: int = 460_800,
                 simulate: bool = True, motor_rpm: int = 0,
                 room: Tuple[float, float, float] = (6.0, 4.0, 2.6),
                 scan_hz: float = 10.0):
        self.port = port
        self.baudrate = baudrate
        self.simulate = simulate
        self.motor_rpm = motor_rpm
        self.room = room
        self.scan_hz = scan_hz

        self._lock = threading.Lock()
        self._latest: Optional[List[Sample]] = None
        self._stats = ReaderStats()
        self._stamps: deque = deque(maxlen=30)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lidar = None

    # -- Steuerung ---------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=2.0)

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # -- Abfrage -----------------------------------------------------------

    def latest(self) -> Optional[List[Sample]]:
        with self._lock:
            return self._latest

    def stats(self) -> ReaderStats:
        with self._lock:
            return ReaderStats(
                revolutions=self._stats.revolutions,
                samples=self._stats.samples,
                valid=self._stats.valid,
                resyncs=self._stats.resyncs,
                rate_hz=self._stats.rate_hz,
                error=self._stats.error,
                info=dict(self._stats.info),
            )

    def wait_for_revolution(self, timeout: float = 2.0) -> Optional[List[Sample]]:
        """Auf die erste Umdrehung warten. Vor allem fuer Tests und Skripte."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            revolution = self.latest()
            if revolution is not None:
                return revolution
            if self.stats().error:
                return None
            time.sleep(0.01)
        return None

    # -- Innenleben --------------------------------------------------------

    def _publish(self, revolution: List[Sample], resyncs: int = 0) -> None:
        now = time.monotonic()
        with self._lock:
            self._latest = revolution
            self._stats.revolutions += 1
            self._stats.samples += len(revolution)
            self._stats.valid += sum(1 for s in revolution if s.distance_mm > 0)
            self._stats.resyncs = resyncs
            self._stamps.append(now)
            if len(self._stamps) >= 2:
                span = self._stamps[-1] - self._stamps[0]
                if span > 0:
                    self._stats.rate_hz = (len(self._stamps) - 1) / span

    def _fail(self, message: str) -> None:
        with self._lock:
            self._stats.error = message

    def _run(self) -> None:
        if self.simulate:
            self._run_simulated()
        else:
            self._run_serial()

    def _run_simulated(self) -> None:
        sim = RoomSimulator(room_m=self.room)
        with self._lock:
            self._stats.info = {"model": "Simulation", "firmware": "-"}
        period = 1.0 / self.scan_hz if self.scan_hz > 0 else 0.0
        while not self._stop.is_set():
            started = time.monotonic()
            self._publish(sim.revolution(yaw_deg=0.0))
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                self._stop.wait(remaining)

    def _run_serial(self) -> None:
        try:
            from .serial_source import SerialLidar
        except Exception as exc:  # pragma: no cover - Umgebungsabhaengig
            self._fail(f"pyserial fehlt: {exc}")
            return

        try:
            self._lidar = SerialLidar(self.port, self.baudrate)
        except Exception as exc:
            self._fail(f"Port {self.port} liess sich nicht oeffnen: {exc}")
            return

        try:
            # Geraeteinfo ist nur Diagnose - ein Fehler hier darf den Scan
            # nicht verhindern.
            try:
                info = self._lidar.device_info()
                status, code = self._lidar.health()
                info["health"] = {0: "gut", 1: "Warnung", 2: "Fehler"}.get(status, status)
                info["health_code"] = code
                with self._lock:
                    self._stats.info = info
            except Exception as exc:
                with self._lock:
                    self._stats.info = {"warnung": f"Geraeteinfo nicht lesbar: {exc}"}

            if self.motor_rpm > 0:
                self._lidar.set_motor_rpm(self.motor_rpm)
                time.sleep(1.0)

            self._lidar.start_standard_scan()

            for revolution in self._lidar.revolutions():
                if self._stop.is_set():
                    break
                self._publish(revolution, resyncs=self._lidar.resyncs)
        except Exception as exc:
            self._fail(str(exc))
        finally:
            try:
                if self._lidar is not None:
                    self._lidar.close()
            except Exception:
                pass
            self._lidar = None
