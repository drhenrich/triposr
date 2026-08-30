"""Direktbetrieb: RPLIDAR S2 haengt per USB-Adapter am PC.

Nuetzlich zum Einfahren, bevor die Firmware fertig ist: der ESP32 dreht nur
die Achse mit konstanter Rate, der PC liest den LiDAR und rechnet den
Gierwinkel aus der Zeit. Braucht ``pyserial``.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterator, Optional, Tuple

from . import rplidar

DEFAULT_BAUDRATE = 1_000_000


@dataclass
class LinearYaw:
    """Gierwinkel als lineare Funktion der Zeit.

    Gilt, solange der Schrittmotor mit konstanter Frequenz laeuft - genau das
    macht die Firmware im Sweep (LEDC erzeugt eine feste STEP-Frequenz).
    """

    start_deg: float
    rate_deg_s: float
    t0: Optional[float] = None

    def start(self, now: Optional[float] = None) -> None:
        self.t0 = time.monotonic() if now is None else now

    def at(self, t: float) -> float:
        if self.t0 is None:
            raise RuntimeError("LinearYaw.start() wurde nicht aufgerufen")
        return self.start_deg + self.rate_deg_s * (t - self.t0)


class SerialLidar:
    """Duenner Wrapper um den S2 an einem seriellen Port."""

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE, timeout: float = 1.0):
        try:
            import serial  # type: ignore
        except ImportError as exc:  # pragma: no cover - Umgebungsabhaengig
            raise RuntimeError("pyserial fehlt: pip install pyserial") from exc
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self._parser = rplidar.CapsuleParser()
        self._decoder = rplidar.CapsuleDecoder()

    def close(self) -> None:
        try:
            self._ser.write(rplidar.cmd_stop())
            time.sleep(0.05)
        finally:
            self._ser.close()

    def __enter__(self) -> "SerialLidar":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _read_descriptor(self) -> rplidar.ResponseDescriptor:
        raw = self._ser.read(7)
        return rplidar.parse_response_descriptor(raw)

    def get_conf(self, conf_type: int, mode: Optional[int] = None) -> bytes:
        self._ser.write(rplidar.cmd_get_lidar_conf(conf_type, mode))
        desc = self._read_descriptor()
        if desc.data_type != rplidar.ANS_TYPE_GET_LIDAR_CONF:
            raise RuntimeError(f"unerwarteter Antworttyp 0x{desc.data_type:02X}")
        body = self._ser.read(desc.length)
        if len(body) != desc.length:
            raise RuntimeError("Antwort unvollstaendig")
        return body[4:]  # die ersten 4 Byte spiegeln den angefragten Typ

    def typical_scan_mode(self) -> int:
        return int.from_bytes(self.get_conf(rplidar.CONF_SCAN_MODE_TYPICAL)[:2], "little")

    def scan_mode_answer_type(self, mode: int) -> int:
        return self.get_conf(rplidar.CONF_SCAN_MODE_ANS_TYPE, mode)[0]

    def set_motor_rpm(self, rpm: int) -> None:
        self._ser.write(rplidar.cmd_motor_rpm(rpm))

    def start_dense_scan(self, mode: Optional[int] = None) -> int:
        """Express-Scan im typischen Modus starten; prueft auf Dense-Capsules."""
        self._ser.write(rplidar.cmd_stop())
        time.sleep(0.05)
        self._ser.reset_input_buffer()

        if mode is None:
            mode = self.typical_scan_mode()
        ans = self.scan_mode_answer_type(mode)
        if ans != rplidar.ANS_TYPE_MEASUREMENT_DENSE_CAPSULED:
            raise RuntimeError(
                f"Modus {mode} liefert Antworttyp 0x{ans:02X}, erwartet 0x85 "
                "(dense capsuled). Anderen Modus waehlen."
            )

        self._ser.write(rplidar.cmd_express_scan(mode))
        desc = self._read_descriptor()
        if desc.data_type != rplidar.ANS_TYPE_MEASUREMENT_DENSE_CAPSULED:
            raise RuntimeError(f"Scan liefert Typ 0x{desc.data_type:02X}, erwartet 0x85")
        self._parser = rplidar.CapsuleParser()
        self._decoder = rplidar.CapsuleDecoder()
        return mode

    def samples(self, yaw: LinearYaw) -> Iterator[Tuple[float, float, float]]:
        """(distance_mm, alpha_deg, yaw_deg) - laeuft, bis der Aufrufer abbricht."""
        while True:
            chunk = self._ser.read(self._ser.in_waiting or 84)
            if not chunk:
                continue
            now = time.monotonic()
            for capsule in self._parser.feed(chunk):
                yaw_deg = yaw.at(now)
                for sample in self._decoder.push(capsule):
                    yield (float(sample.distance_mm), sample.angle_deg, yaw_deg)

    @property
    def checksum_errors(self) -> int:
        return self._parser.checksum_errors
