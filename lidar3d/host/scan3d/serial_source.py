"""Direktbetrieb: der LiDAR haengt per USB-Adapter am PC.

Zwei Geraete, zwei Betriebsarten:

* **RPLIDAR C1** - 5000 Messungen/s bei 460800 Baud. Das sind 25 kB/s von
  46 kB/s Leitungskapazitaet, also genuegt der einfache Scanmodus mit 5 Byte
  je Messung. Das ist der Standardfall (``start_standard_scan``).
* **RPLIDAR S2** - 32000 Messungen/s bei 1 MBaud. 160 kB/s passen nicht durch
  100 kB/s, deshalb zwingend der dense-capsuled Modus
  (``start_dense_scan``).

Braucht ``pyserial``.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple

from . import rplidar

#: RPLIDAR C1 ab Werk.
BAUDRATE_C1 = 460_800
#: RPLIDAR S2 und S3.
BAUDRATE_S2 = 1_000_000
DEFAULT_BAUDRATE = BAUDRATE_C1


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
        self._standard = rplidar.StandardScanParser()
        self._assembler = rplidar.RevolutionAssembler()

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

    def device_info(self) -> dict:
        """Modell, Firmware- und Hardwarestand. Guter erster Verbindungstest."""
        self._ser.write(rplidar.cmd_get_device_info())
        desc = self._read_descriptor()
        body = self._ser.read(desc.length)
        if len(body) != desc.length or desc.length < 20:
            raise RuntimeError("Geraeteinfo unvollstaendig")
        return {
            "model": body[0],
            "firmware": f"{body[2]}.{body[1]}",
            "hardware": body[3],
            "serial": body[4:20].hex(),
        }

    def health(self) -> Tuple[int, int]:
        """(Status, Fehlercode). Status 0 = gut, 1 = Warnung, 2 = Fehler."""
        self._ser.write(rplidar.cmd_get_health())
        desc = self._read_descriptor()
        body = self._ser.read(desc.length)
        if len(body) < 3:
            raise RuntimeError("Health-Antwort unvollstaendig")
        return body[0], int.from_bytes(body[1:3], "little")

    def start_standard_scan(self) -> None:
        """Einfacher Scanmodus, 5 Byte je Messung - der Weg fuer den C1."""
        self._ser.write(rplidar.cmd_stop())
        time.sleep(0.05)
        self._ser.reset_input_buffer()

        self._ser.write(rplidar.cmd_scan())
        desc = self._read_descriptor()
        if desc.data_type != rplidar.ANS_TYPE_MEASUREMENT:
            raise RuntimeError(
                f"Scan liefert Typ 0x{desc.data_type:02X}, erwartet 0x81")
        if desc.length != rplidar.STANDARD_NODE_SIZE:
            raise RuntimeError(
                f"Messung ist {desc.length} statt "
                f"{rplidar.STANDARD_NODE_SIZE} Byte lang")
        self._standard = rplidar.StandardScanParser()
        self._assembler = rplidar.RevolutionAssembler()

    def revolutions(self) -> Iterator[List[rplidar.Sample]]:
        """Vollstaendige Umdrehungen; laeuft, bis der Aufrufer abbricht.

        Nur nach ``start_standard_scan`` verwenden.
        """
        while True:
            chunk = self._ser.read(self._ser.in_waiting or rplidar.STANDARD_NODE_SIZE)
            if not chunk:
                continue
            for revolution in self._assembler.feed(self._standard.feed(chunk)):
                yield revolution

    @property
    def resyncs(self) -> int:
        return self._standard.resyncs

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
