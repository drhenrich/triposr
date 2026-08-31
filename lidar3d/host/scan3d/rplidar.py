"""RPLIDAR S2 Protokoll: Kommandos und Dekoder fuer "dense capsuled" Scandaten.

Reine Standardbibliothek, damit der Dekoder ohne numpy testbar ist und
1:1 als Referenz fuer die C++-Firmware dient (siehe firmware/src/rplidar.cpp).

Warum "dense capsuled" und nicht der einfache SCAN-Modus:
Der S2 liefert 32000 Messungen/s, die UART laeuft mit 1 Mbaud (~100 kB/s netto).
Der einfache SCAN-Modus braucht 5 Byte pro Messung = 160 kB/s und passt damit
nicht durch die Leitung. Eine Dense-Capsule packt 40 Messungen in 84 Byte
(2.1 Byte/Messung = ~67 kB/s) und passt.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable, Iterator, List, Optional, Sequence

# ---------------------------------------------------------------------------
# Kommandos (aus sl_lidar_cmd.h des Slamtec SDK)
# ---------------------------------------------------------------------------

SYNC_BYTE = 0xA5
SYNC_BYTE2 = 0x5A

CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_EXPRESS_SCAN = 0x82
CMD_GET_DEVICE_INFO = 0x50
CMD_GET_DEVICE_HEALTH = 0x52
CMD_GET_LIDAR_CONF = 0x84
CMD_HQ_MOTOR_SPEED_CTRL = 0xA8

# Antworttypen
ANS_TYPE_DEVINFO = 0x04
ANS_TYPE_DEVHEALTH = 0x06
ANS_TYPE_MEASUREMENT = 0x81
ANS_TYPE_MEASUREMENT_CAPSULED = 0x82
ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA = 0x84
ANS_TYPE_MEASUREMENT_DENSE_CAPSULED = 0x85
ANS_TYPE_GET_LIDAR_CONF = 0x20

# GET_LIDAR_CONF Typen
CONF_SCAN_MODE_COUNT = 0x70
CONF_SCAN_MODE_US_PER_SAMPLE = 0x71
CONF_SCAN_MODE_MAX_DISTANCE = 0x74
CONF_SCAN_MODE_ANS_TYPE = 0x75
CONF_SCAN_MODE_TYPICAL = 0x7C
CONF_SCAN_MODE_NAME = 0x7F

# Dense-Capsule Layout
DENSE_CABIN_COUNT = 40
DENSE_CAPSULE_SIZE = 2 + 2 + 2 * DENSE_CABIN_COUNT  # 84 Byte
EXP_SYNC_1 = 0xA  # oberes Nibble von Byte 0
EXP_SYNC_2 = 0x5  # oberes Nibble von Byte 1


def _with_checksum(payload: bytes) -> bytes:
    """Slamtec-Pruefsumme: XOR ueber alle vorangehenden Bytes."""
    cs = 0
    for b in payload:
        cs ^= b
    return payload + bytes([cs])


def cmd_simple(command: int) -> bytes:
    """Kommando ohne Payload."""
    return bytes([SYNC_BYTE, command])


def cmd_with_payload(command: int, payload: bytes) -> bytes:
    """Kommando mit Payload; Laengenbyte und Pruefsumme werden ergaenzt."""
    if len(payload) > 255:
        raise ValueError("payload zu lang")
    return _with_checksum(bytes([SYNC_BYTE, command, len(payload)]) + payload)


def cmd_scan() -> bytes:
    """Einfacher Scanmodus, 5 Byte je Messung.

    Beim C1 reicht der: 5000 Messungen/s x 5 Byte sind 25 kB/s, durch
    460800 Baud passen rund 46 kB/s. Beim S2 mit 32000 Messungen/s waere
    das nicht gegangen - daher dort der dense-capsuled Modus.
    """
    return cmd_simple(CMD_SCAN)


def cmd_get_device_info() -> bytes:
    return cmd_simple(CMD_GET_DEVICE_INFO)


def cmd_get_health() -> bytes:
    return cmd_simple(CMD_GET_DEVICE_HEALTH)


def cmd_stop() -> bytes:
    return cmd_simple(CMD_STOP)


def cmd_reset() -> bytes:
    return cmd_simple(CMD_RESET)


def cmd_express_scan(mode: int) -> bytes:
    """EXPRESS_SCAN. Payload: working_mode u8, working_flags u16, param u16."""
    return cmd_with_payload(CMD_EXPRESS_SCAN, struct.pack("<BHH", mode, 0, 0))


def cmd_get_lidar_conf(conf_type: int, mode: Optional[int] = None) -> bytes:
    payload = struct.pack("<I", conf_type)
    if mode is not None:
        payload += struct.pack("<H", mode)
    return cmd_with_payload(CMD_GET_LIDAR_CONF, payload)


def cmd_motor_rpm(rpm: int) -> bytes:
    """Drehzahl des LiDAR-Kopfes setzen. 10 Hz Scanrate = 600 rpm."""
    return cmd_with_payload(CMD_HQ_MOTOR_SPEED_CTRL, struct.pack("<H", rpm))


@dataclass(frozen=True)
class ResponseDescriptor:
    length: int
    mode: int
    data_type: int

    @property
    def is_stream(self) -> bool:
        return self.mode == 1


def parse_response_descriptor(raw: bytes) -> ResponseDescriptor:
    """Antwort-Deskriptor: A5 5A, dann 30 bit Laenge + 2 bit Mode, dann Typ."""
    if len(raw) != 7:
        raise ValueError(f"Deskriptor muss 7 Byte lang sein, ist {len(raw)}")
    if raw[0] != SYNC_BYTE or raw[1] != SYNC_BYTE2:
        raise ValueError("ungueltiger Deskriptor-Header")
    word = struct.unpack("<I", raw[2:6])[0]
    return ResponseDescriptor(
        length=word & 0x3FFFFFFF, mode=(word >> 30) & 0x03, data_type=raw[6]
    )


# ---------------------------------------------------------------------------
# Dense-Capsule Dekoder
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DenseCapsule:
    """Eine rohe Dense-Capsule: Startwinkel plus 40 Distanzen in mm."""

    start_angle_q6: int  # Q6-Grad, Bit 15 bereits entfernt
    start_flag: bool  # Bit 15: Beginn einer neuen Umdrehung
    distances_mm: Sequence[int]

    @property
    def start_angle_deg(self) -> float:
        return self.start_angle_q6 / 64.0


@dataclass(frozen=True)
class Sample:
    """Eine einzelne Messung in der Scanebene des LiDAR."""

    angle_deg: float  # 0..360, LiDAR-eigener Winkel
    distance_mm: float  # 0 == kein Echo
    new_revolution: bool
    #: Signalguete 0..63. Nur der einfache Scanmodus liefert sie.
    quality: int = 0


STANDARD_NODE_SIZE = 5


class StandardScanParser:
    """Byte-Strom -> Messungen im einfachen Scanmodus (Antworttyp 0x81).

    Eine Messung sind 5 Byte:

        Byte 0: Bit 0 = S (Umlaufmarke), Bit 1 = !S, Bit 2..7 = Guete
        Byte 1: Bit 0 = C (Pruefbit, immer 1), Bit 1..7 = Winkel Q6, low
        Byte 2: Winkel Q6, high
        Byte 3..4: Distanz Q2 (u16 LE), 0 = kein Echo

    Eine Pruefsumme gibt es nicht - die Gueltigkeit haengt allein an S != !S
    und C == 1. Das reicht zum Resynchronisieren, weil ein falsch
    ausgerichteter Strom diese beiden Bedingungen schnell verletzt.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.resyncs = 0
        self.invalid_nodes = 0

    def reset(self) -> None:
        self._buf.clear()

    @staticmethod
    def _node_looks_valid(b0: int, b1: int) -> bool:
        start = b0 & 0x01
        start_inv = (b0 >> 1) & 0x01
        check = b1 & 0x01
        return start != start_inv and check == 1

    def feed(self, data: bytes) -> Iterator[Sample]:
        self._buf.extend(data)
        buf = self._buf
        while len(buf) >= STANDARD_NODE_SIZE:
            if not self._node_looks_valid(buf[0], buf[1]):
                del buf[0]
                self.resyncs += 1
                self.invalid_nodes += 1
                continue

            node = bytes(buf[:STANDARD_NODE_SIZE])
            del buf[:STANDARD_NODE_SIZE]

            quality = node[0] >> 2
            new_revolution = bool(node[0] & 0x01)
            angle_q6 = (node[1] >> 1) | (node[2] << 7)
            distance_q2 = struct.unpack_from("<H", node, 3)[0]
            yield Sample(
                angle_deg=(angle_q6 / 64.0) % 360.0,
                distance_mm=distance_q2 / 4.0,
                new_revolution=new_revolution,
                quality=quality,
            )


class RevolutionAssembler:
    """Messungen -> vollstaendige Umdrehungen.

    Eine Umdrehung endet, sobald die naechste Umlaufmarke kommt. Die erste,
    angebrochene Umdrehung wird verworfen, damit jede gelieferte Umdrehung
    wirklich vollstaendig ist.
    """

    def __init__(self) -> None:
        self._current: List[Sample] = []
        self._started = False
        self.revolutions = 0

    def reset(self) -> None:
        self._current = []
        self._started = False

    def feed(self, samples: Iterable[Sample]) -> Iterator[List[Sample]]:
        for sample in samples:
            if sample.new_revolution:
                if self._started and self._current:
                    self.revolutions += 1
                    yield self._current
                self._current = []
                self._started = True
            if self._started:
                self._current.append(sample)


class CapsuleParser:
    """Byte-Stream -> DenseCapsule.

    Sucht das Sync-Muster (oberes Nibble 0xA / 0x5), prueft die Pruefsumme und
    resynchronisiert bei Fehlern byteweise.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.checksum_errors = 0
        self.resyncs = 0

    def feed(self, data: bytes) -> Iterator[DenseCapsule]:
        self._buf.extend(data)
        while True:
            capsule = self._try_pop()
            if capsule is None:
                return
            yield capsule

    def _try_pop(self) -> Optional[DenseCapsule]:
        buf = self._buf
        while True:
            if len(buf) < DENSE_CAPSULE_SIZE:
                return None
            if (buf[0] >> 4) != EXP_SYNC_1 or (buf[1] >> 4) != EXP_SYNC_2:
                del buf[0]
                self.resyncs += 1
                continue

            frame = bytes(buf[:DENSE_CAPSULE_SIZE])
            expected = (frame[0] & 0x0F) | ((frame[1] & 0x0F) << 4)
            actual = 0
            for b in frame[2:]:
                actual ^= b
            if expected != actual:
                self.checksum_errors += 1
                del buf[0]
                self.resyncs += 1
                continue

            del buf[:DENSE_CAPSULE_SIZE]
            raw_start = struct.unpack_from("<H", frame, 2)[0]
            distances = struct.unpack_from(f"<{DENSE_CABIN_COUNT}H", frame, 4)
            return DenseCapsule(
                start_angle_q6=raw_start & 0x7FFF,
                start_flag=bool(raw_start & 0x8000),
                distances_mm=distances,
            )


class CapsuleDecoder:
    """DenseCapsule -> 40 Messungen.

    Der Winkel einer Messung ergibt sich erst aus dem Startwinkel der *naechsten*
    Capsule: die 40 Messungen werden linear zwischen beiden Startwinkeln
    interpoliert. Der Dekoder haelt deshalb immer eine Capsule zurueck.
    Das entspricht ``_dense_capsuleToNormal`` im Slamtec SDK.
    """

    def __init__(self) -> None:
        self._prev: Optional[DenseCapsule] = None
        self._pending_revolution = False

    def reset(self) -> None:
        self._prev = None
        self._pending_revolution = False

    def angle_increment_q16(self, current: DenseCapsule) -> Optional[int]:
        """Winkelschritt pro Messung in Q16-Grad, oder None ohne Vorgaenger."""
        if self._prev is None:
            return None
        cur_q8 = current.start_angle_q6 << 2
        prev_q8 = self._prev.start_angle_q6 << 2
        diff_q8 = cur_q8 - prev_q8
        if prev_q8 > cur_q8:
            diff_q8 += 360 << 8
        return (diff_q8 << 8) // DENSE_CABIN_COUNT

    def push(self, current: DenseCapsule) -> List[Sample]:
        """Naechste Capsule einspeisen, Messungen der vorigen zurueckgeben."""
        inc_q16 = self.angle_increment_q16(current)
        if inc_q16 is None or self._prev is None:
            self._prev = current
            return []

        start_q16 = self._prev.start_angle_q6 << 10
        rev_index = self._revolution_index(current, start_q16, inc_q16)

        angle_q16 = start_q16
        out: List[Sample] = []
        for i, dist in enumerate(self._prev.distances_mm):
            angle_q6 = (angle_q16 >> 10) % (360 << 6)
            out.append(
                Sample(
                    angle_deg=angle_q6 / 64.0,
                    distance_mm=dist,
                    new_revolution=(i == rev_index),
                )
            )
            angle_q16 += inc_q16
        self._prev = current
        return out

    def _revolution_index(
        self, current: DenseCapsule, start_q16: int, inc_q16: int
    ) -> Optional[int]:
        """Index der Messung, bei der die 360-Grad-Grenze ueberschritten wird.

        Die Grenze liegt zwischen zwei Capsules genau dann, wenn der Startwinkel
        kleiner wird. Faellt sie rechnerisch hinter die letzte Messung dieser
        Capsule (Rundung, oder Capsules exakt auf der Grenze), wird sie auf die
        erste Messung der naechsten Capsule vorgetragen.
        """
        rev_index: Optional[int] = None
        if self._pending_revolution:
            rev_index = 0
            self._pending_revolution = False

        assert self._prev is not None
        wrapped = self._prev.start_angle_q6 > current.start_angle_q6
        if not wrapped or inc_q16 <= 0:
            return rev_index

        remaining_q16 = (360 << 16) - start_q16
        index = -(-remaining_q16 // inc_q16)  # aufrunden
        if index >= DENSE_CABIN_COUNT:
            self._pending_revolution = True
        elif rev_index is None:
            rev_index = index
        return rev_index
