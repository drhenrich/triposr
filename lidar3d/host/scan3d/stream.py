"""Streamprotokoll ESP32-S3 -> Host (WLAN/TCP).

Die Firmware haengt an jede Dense-Capsule den Gierwinkel und schickt das
Ergebnis als Frame mit 8-Byte-Header weiter. Format siehe docs/03-protokolle.md;
Gegenstueck in der Firmware: firmware/src/stream_proto.h.

Warum WLAN und nicht BLE: 32000 Messungen/s ergeben rund 80 kB/s, also ~640 kbit/s
Nutzlast. BLE schafft realistisch 100-200 kbit/s und scheidet damit aus.
"""

from __future__ import annotations

import socket
import struct
from dataclasses import dataclass
from typing import Iterator, List, Optional, Sequence, Tuple

MAGIC = 0x4E57  # 'NW'
HEADER = struct.Struct("<HBBHH")  # magic, type, flags, seq, payload_len
HEADER_SIZE = HEADER.size  # 8

TYPE_HELLO = 0
TYPE_CAPSULE = 1  # S2: 40 Messungen auf gleichmaessigem Winkelraster
TYPE_STATUS = 2
TYPE_SCAN = 3     # C1: Messungen mit eigenem Winkel je Stueck
TYPE_FAULT = 4    # Hochlauf misslungen, mit Klartext

#: Gruende, aus denen der Hochlauf scheitern kann. Der Scanner bleibt dabei
#: im Netz erreichbar und meldet den Grund - sonst braeuchte es das serielle
#: Kabel, um ueberhaupt zu sehen, dass die Firmware laeuft.
FAULT_NONE = 0
FAULT_SERVO = 1        # STS3215 antwortet nicht
FAULT_LIDAR_PORT = 2   # UART zum LiDAR liess sich nicht oeffnen
FAULT_LIDAR_SCAN = 3   # kein Scanmodus liess sich starten
FAULT_QUEUE = 4        # Speicher fuer die Frame-Queue fehlt
FAULT_MAX_TEXT_LEN = 160

FLAG_NEW_REVOLUTION = 1 << 0
FLAG_SWEEP_ACTIVE = 1 << 1
FLAG_SWEEP_REVERSED = 1 << 2

CABIN_COUNT = 40
#: yaw_start_q16 u32, yaw_end_q16 u32, alpha_inc_q16 i32, alpha_q6 u16, reserved u16
_CAPSULE_HEAD = struct.Struct("<IIiHH")
CAPSULE_PAYLOAD_SIZE = _CAPSULE_HEAD.size + 2 * CABIN_COUNT  # 96

#: yaw_start_q16 u32, yaw_end_q16 u32, count u16, reserved u16
_SCAN_HEAD = struct.Struct("<IIHH")
SCAN_HEAD_SIZE = _SCAN_HEAD.size  # 12
SCAN_SAMPLE_SIZE = 4              # angle_q6 u16, distance_mm u16
SCAN_MAX_SAMPLES = 32

#: fw_version u16, lidar_rpm u16, offset_radial_um i32, offset_axial_um i32,
#: yaw_min_q16 u32, yaw_max_q16 u32
_HELLO = struct.Struct("<HHiiII")

#: sweep_index u16, state u8, reserved u8, yaw_q16 u32, capsules u32,
#: checksum_errors u32, dropped_frames u32
_STATUS = struct.Struct("<HBBIIII")


@dataclass(frozen=True)
class Frame:
    type: int
    flags: int
    seq: int
    payload: bytes


@dataclass(frozen=True)
class CapsuleFrame:
    """Eine Capsule mit bereits zugeordnetem Gierwinkel."""

    seq: int
    flags: int
    yaw_start_deg: float
    yaw_end_deg: float
    alpha_start_deg: float
    alpha_inc_deg: float
    distances_mm: Sequence[int]

    @property
    def sweep_active(self) -> bool:
        return bool(self.flags & FLAG_SWEEP_ACTIVE)

    def samples(self) -> Iterator[Tuple[float, float, float]]:
        """(distance_mm, alpha_deg, yaw_deg) je Messung.

        Gierwinkel wird ueber die Capsule linear interpoliert. Bei 10 deg/s
        Gierrate und 1.25 ms Capsule-Dauer sind das 0.0125 deg Spanne - die
        Interpolation ist Feinschliff, kein Muss.
        """
        n = len(self.distances_mm)
        yaw_span = self.yaw_end_deg - self.yaw_start_deg
        # Nulldurchgang der Gierachse abfangen (sollte im Sweep nicht auftreten).
        if yaw_span > 180.0:
            yaw_span -= 360.0
        elif yaw_span < -180.0:
            yaw_span += 360.0
        for i, dist in enumerate(self.distances_mm):
            alpha = (self.alpha_start_deg + i * self.alpha_inc_deg) % 360.0
            yaw = self.yaw_start_deg + yaw_span * i / n
            yield (float(dist), alpha, yaw)


@dataclass(frozen=True)
class ScanFrame:
    """Eine Gruppe Messungen aus dem einfachen Scanmodus des C1.

    Anders als bei der Capsule traegt jede Messung ihren eigenen Winkel: der C1
    verteilt seine Messungen nicht exakt gleichmaessig, und ein interpoliertes
    Raster wuerde diese Information wegwerfen. Ein Frame gehoert immer zu genau
    einer Umdrehung, deshalb gilt FLAG_NEW_REVOLUTION fuer den ganzen Frame.
    """

    seq: int
    flags: int
    yaw_start_deg: float
    yaw_end_deg: float
    angles_deg: Sequence[float]
    distances_mm: Sequence[int]

    @property
    def sweep_active(self) -> bool:
        return bool(self.flags & FLAG_SWEEP_ACTIVE)

    @property
    def new_revolution(self) -> bool:
        return bool(self.flags & FLAG_NEW_REVOLUTION)

    def samples(self) -> Iterator[Tuple[float, float, float]]:
        """(distance_mm, alpha_deg, yaw_deg) je Messung.

        Interpoliert wird nur der Gierwinkel - der Scanwinkel steht gemessen da.
        """
        n = len(self.distances_mm)
        yaw_span = self.yaw_end_deg - self.yaw_start_deg
        if yaw_span > 180.0:
            yaw_span -= 360.0
        elif yaw_span < -180.0:
            yaw_span += 360.0
        for i, dist in enumerate(self.distances_mm):
            yield (float(dist), self.angles_deg[i], self.yaw_start_deg + yaw_span * i / n)


@dataclass(frozen=True)
class FaultFrame:
    """Der Scanner laeuft, aber der Hochlauf ist gescheitert."""

    seq: int
    code: int
    text: str


@dataclass(frozen=True)
class HelloFrame:
    fw_version: int
    lidar_rpm: int
    offset_radial_mm: float
    offset_axial_mm: float
    yaw_min_deg: float
    yaw_max_deg: float


@dataclass(frozen=True)
class StatusFrame:
    sweep_index: int
    state: int
    yaw_deg: float
    capsules: int
    checksum_errors: int
    dropped_frames: int


def encode_frame(frame_type: int, flags: int, seq: int, payload: bytes) -> bytes:
    return HEADER.pack(MAGIC, frame_type, flags, seq & 0xFFFF, len(payload)) + payload


def encode_capsule(
    seq: int,
    flags: int,
    yaw_start_deg: float,
    yaw_end_deg: float,
    alpha_start_deg: float,
    alpha_inc_deg: float,
    distances_mm: Sequence[int],
) -> bytes:
    """Nur fuer Tests und den Simulator - die Firmware baut das in C++."""
    if len(distances_mm) != CABIN_COUNT:
        raise ValueError(f"genau {CABIN_COUNT} Distanzen erwartet")
    payload = _CAPSULE_HEAD.pack(
        round(yaw_start_deg * 65536) & 0xFFFFFFFF,
        round(yaw_end_deg * 65536) & 0xFFFFFFFF,
        round(alpha_inc_deg * 65536),
        round(alpha_start_deg * 64) & 0xFFFF,
        0,
    ) + struct.pack(f"<{CABIN_COUNT}H", *distances_mm)
    return encode_frame(TYPE_CAPSULE, flags, seq, payload)


def decode_capsule(frame: Frame) -> CapsuleFrame:
    if frame.type != TYPE_CAPSULE:
        raise ValueError(f"kein Capsule-Frame (type={frame.type})")
    if len(frame.payload) != CAPSULE_PAYLOAD_SIZE:
        raise ValueError(f"Capsule-Payload hat {len(frame.payload)} statt {CAPSULE_PAYLOAD_SIZE} Byte")
    yaw_start, yaw_end, alpha_inc, alpha_q6, _ = _CAPSULE_HEAD.unpack_from(frame.payload)
    distances = struct.unpack_from(f"<{CABIN_COUNT}H", frame.payload, _CAPSULE_HEAD.size)
    return CapsuleFrame(
        seq=frame.seq,
        flags=frame.flags,
        yaw_start_deg=yaw_start / 65536.0,
        yaw_end_deg=yaw_end / 65536.0,
        alpha_start_deg=alpha_q6 / 64.0,
        alpha_inc_deg=alpha_inc / 65536.0,
        distances_mm=distances,
    )


def encode_scan(
    seq: int,
    flags: int,
    yaw_start_deg: float,
    yaw_end_deg: float,
    samples: Sequence[Tuple[float, int]],
) -> bytes:
    """(alpha_deg, distance_mm)-Paare als Scanframe. Fuer Tests und Simulator."""
    if len(samples) > SCAN_MAX_SAMPLES:
        raise ValueError(f"hoechstens {SCAN_MAX_SAMPLES} Messungen je Frame")
    payload = bytearray(_SCAN_HEAD.pack(
        round(yaw_start_deg * 65536) & 0xFFFFFFFF,
        round(yaw_end_deg * 65536) & 0xFFFFFFFF,
        len(samples),
        0,
    ))
    for alpha_deg, distance_mm in samples:
        payload += struct.pack("<HH", round(alpha_deg * 64) & 0xFFFF, distance_mm)
    return encode_frame(TYPE_SCAN, flags, seq, bytes(payload))


def decode_scan(frame: Frame) -> ScanFrame:
    if frame.type != TYPE_SCAN:
        raise ValueError(f"kein Scan-Frame (type={frame.type})")
    if len(frame.payload) < SCAN_HEAD_SIZE:
        raise ValueError("Scan-Payload zu kurz")
    yaw_start, yaw_end, count, _ = _SCAN_HEAD.unpack_from(frame.payload)
    expected = SCAN_HEAD_SIZE + SCAN_SAMPLE_SIZE * count
    if count > SCAN_MAX_SAMPLES or len(frame.payload) != expected:
        raise ValueError(
            f"Scan-Payload hat {len(frame.payload)} Byte, erwartet {expected} "
            f"fuer {count} Messungen")
    raw = struct.unpack_from(f"<{2 * count}H", frame.payload, SCAN_HEAD_SIZE)
    return ScanFrame(
        seq=frame.seq,
        flags=frame.flags,
        yaw_start_deg=yaw_start / 65536.0,
        yaw_end_deg=yaw_end / 65536.0,
        angles_deg=tuple(raw[2 * i] / 64.0 for i in range(count)),
        distances_mm=tuple(raw[2 * i + 1] for i in range(count)),
    )


def encode_fault(seq: int, code: int, text: str) -> bytes:
    """Nur fuer Tests und den Simulator - die Firmware baut das in C++."""
    raw = text.encode("ascii", "replace")[:FAULT_MAX_TEXT_LEN]
    return encode_frame(TYPE_FAULT, 0, seq,
                        bytes([code, len(raw)]) + raw)


def decode_fault(frame: Frame) -> FaultFrame:
    if frame.type != TYPE_FAULT:
        raise ValueError(f"kein Fault-Frame (type={frame.type})")
    if len(frame.payload) < 2:
        raise ValueError("Fault-Payload zu kurz")
    code, length = frame.payload[0], frame.payload[1]
    if len(frame.payload) != 2 + length:
        raise ValueError(
            f"Fault-Payload hat {len(frame.payload)} Byte, erwartet {2 + length}")
    return FaultFrame(seq=frame.seq, code=code,
                      text=frame.payload[2:].decode("ascii", "replace"))


def decode_hello(frame: Frame) -> HelloFrame:
    ver, rpm, radial_um, axial_um, yaw_min, yaw_max = _HELLO.unpack_from(frame.payload)
    return HelloFrame(
        fw_version=ver,
        lidar_rpm=rpm,
        offset_radial_mm=radial_um / 1000.0,
        offset_axial_mm=axial_um / 1000.0,
        yaw_min_deg=yaw_min / 65536.0,
        yaw_max_deg=yaw_max / 65536.0,
    )


def decode_status(frame: Frame) -> StatusFrame:
    sweep, state, _, yaw_q16, capsules, cs_err, dropped = _STATUS.unpack_from(frame.payload)
    return StatusFrame(
        sweep_index=sweep,
        state=state,
        yaw_deg=yaw_q16 / 65536.0,
        capsules=capsules,
        checksum_errors=cs_err,
        dropped_frames=dropped,
    )


class FrameParser:
    """Byte-Stream -> Frames, mit Resynchronisierung auf das Magic."""

    MAX_PAYLOAD = 4096

    def __init__(self) -> None:
        self._buf = bytearray()
        self.resyncs = 0

    def feed(self, data: bytes) -> Iterator[Frame]:
        self._buf.extend(data)
        while True:
            frame = self._try_pop()
            if frame is None:
                return
            yield frame

    def _try_pop(self) -> Optional[Frame]:
        buf = self._buf
        while True:
            if len(buf) < HEADER_SIZE:
                return None
            magic, ftype, flags, seq, length = HEADER.unpack_from(buf)
            if magic != MAGIC or length > self.MAX_PAYLOAD:
                del buf[0]
                self.resyncs += 1
                continue
            if len(buf) < HEADER_SIZE + length:
                return None
            payload = bytes(buf[HEADER_SIZE : HEADER_SIZE + length])
            del buf[: HEADER_SIZE + length]
            return Frame(type=ftype, flags=flags, seq=seq, payload=payload)


class TcpSource:
    """Verbindung zum Scanner; liefert Frames, bis der Sweep endet."""

    def __init__(self, host: str, port: int = 5005, timeout: float = 10.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._parser = FrameParser()

    def __enter__(self) -> "TcpSource":
        self._sock = socket.create_connection((self.host, self.port), self.timeout)
        self._sock.settimeout(self.timeout)
        return self

    def __exit__(self, *exc) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def frames(self) -> Iterator[Frame]:
        assert self._sock is not None, "TcpSource als Kontextmanager verwenden"
        while True:
            chunk = self._sock.recv(16384)
            if not chunk:
                return
            for frame in self._parser.feed(chunk):
                yield frame


def collect_sweep(frames: Iterator[Frame]) -> Tuple[List[Tuple[float, float, float]], Optional[HelloFrame]]:
    """Frames bis zum Sweep-Ende einsammeln und zu Messtripeln aufloesen.

    Ein Sweep gilt als beendet, sobald nach aktiven Capsules ein Status-Frame
    mit ``state == 0`` (idle) eintrifft.
    """
    hello: Optional[HelloFrame] = None
    samples: List[Tuple[float, float, float]] = []
    saw_active = False
    for frame in frames:
        if frame.type == TYPE_HELLO:
            hello = decode_hello(frame)
        elif frame.type == TYPE_CAPSULE:
            capsule = decode_capsule(frame)
            if not capsule.sweep_active:
                continue
            saw_active = True
            samples.extend(capsule.samples())
        elif frame.type == TYPE_STATUS:
            if saw_active and decode_status(frame).state == 0:
                break
    return samples, hello
