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
TYPE_CAPSULE = 1
TYPE_STATUS = 2

FLAG_NEW_REVOLUTION = 1 << 0
FLAG_SWEEP_ACTIVE = 1 << 1
FLAG_SWEEP_REVERSED = 1 << 2

CABIN_COUNT = 40
#: yaw_start_q16 u32, yaw_end_q16 u32, alpha_inc_q16 i32, alpha_q6 u16, reserved u16
_CAPSULE_HEAD = struct.Struct("<IIiHH")
CAPSULE_PAYLOAD_SIZE = _CAPSULE_HEAD.size + 2 * CABIN_COUNT  # 96

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
