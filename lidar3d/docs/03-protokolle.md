# Protokolle

Zwei Protokolle sind im Spiel: das Slamtec-Protokoll zwischen LiDAR und ESP32,
und das eigene Frameprotokoll zwischen ESP32 und Host.

---

## 1. RPLIDAR → ESP32

UART, 8N1, 3,3 V TTL. **C1: 460.800 Baud. S2: 1.000.000 Baud.**

### Welcher Scanmodus, und warum

Das entscheidet allein die Bandbreite:

| Gerät | Messungen/s | Modus | Byte je Messung | Last | Leitung | passt? |
|---|---|---|---|---|---|---|
| C1 | 5.000 | SCAN (0x20) | 5 | 25 kB/s | ~46 kB/s | ja |
| S2 | 32.000 | SCAN (0x20) | 5 | 160 kB/s | ~100 kB/s | **nein** |
| S2 | 32.000 | dense capsuled (0x82) | 2,1 | 67 kB/s | ~100 kB/s | ja |

Beim C1 ist der einfache Modus also nicht nur ausreichend, sondern besser: er
schickt zu **jeder Messung ihren eigenen Winkel** mit. Beim S2 wäre das zu viel,
deshalb enthält eine Dense-Capsule 84 Byte für 40 Messungen und der Winkel wird
zwischen den Startwinkeln aufeinanderfolgender Capsules interpoliert.

Der Unterschied ist keine Formsache: die Winkel des C1 sind **nicht exakt
gleichmäßig verteilt**. Ein interpoliertes Raster über sie zu legen würde
Messinformation wegwerfen. Deshalb reicht die Firmware sie einzeln weiter.

Die Firmware versucht beim Start zuerst den einfachen Modus und weicht nur auf
die Capsules aus, wenn er sich nicht starten lässt — siehe `firmware/src/rplidar.h`.

### Startsequenz für den einfachen Modus (C1)

1. `STOP`, kurz warten, Eingangspuffer leeren.
2. `SCAN` (`A5 20`), Deskriptor prüfen: `data_type == 0x81`, `length == 5`.
3. Ab hier laufen 5-Byte-Messungen ohne weitere Anforderung durch.

Keine Modusabfrage nötig — den einfachen Modus hat jeder RPLIDAR.

### Aufbau einer Messung (5 Byte, Antworttyp 0x81)

| Byte | Bit | Inhalt |
|---|---|---|
| 0 | 0 | `S` — Beginn einer Umdrehung |
| 0 | 1 | `!S` — muss das Gegenteil von `S` sein |
| 0 | 2..7 | `quality` |
| 1 | 0 | `check` — muss 1 sein |
| 1 | 1..7 | `angle_q6`, untere 7 Bit |
| 2 | 0..7 | `angle_q6`, obere 8 Bit |
| 3..4 | | `distance_q2` (Viertelmillimeter), little endian |

Die beiden Prüfbits sind das einzige Raster, das der Modus hergibt: geht ein
Byte verloren, wird byteweise resynchronisiert, bis `S != !S` und `check == 1`
wieder zusammenpassen. Implementiert in `host/scan3d/rplidar.py`
(`StandardScanParser`) und `firmware/src/standard_scan.h`.

### Kommandos

Ohne Payload: `A5 <cmd>`.
Mit Payload: `A5 <cmd> <len> <payload…> <XOR aller vorherigen Bytes>`.

| Kommando | Byte | Payload |
|---|---|---|
| SCAN | 0x20 | — |
| STOP | 0x25 | — |
| RESET | 0x40 | — |
| EXPRESS_SCAN | 0x82 | `working_mode u8, working_flags u16, param u16` |
| GET_LIDAR_CONF | 0x84 | `type u32` (+ `mode u16` bei modusbezogenen Abfragen) |
| HQ_MOTOR_SPEED_CTRL | 0xA8 | `rpm u16` — 600 rpm = 10 Hz Scanrate |

C1 und S2 regeln ihren Motor selbst; ein PWM-Pin wie bei der A-Serie entfällt.

### Antwort-Deskriptor

```
A5 5A | length:30 mode:2 (u32 LE) | data_type u8
```

`mode == 1` bedeutet: es folgt ein kontinuierlicher Datenstrom.

### Startsequenz für die Dense-Capsules (S2)

1. `STOP`, kurz warten, Eingangspuffer leeren.
2. `GET_LIDAR_CONF(0x7C)` → typische Modusnummer.
3. `GET_LIDAR_CONF(0x75, mode)` → Antworttyp dieses Modus. **Muss 0x85 sein**
   (dense capsuled); alles andere passt nicht durch die UART, und die Firmware
   bricht mit einer Meldung ab, statt später an unerklärlichen Aussetzern zu
   scheitern.
4. `EXPRESS_SCAN(mode)`, Deskriptor prüfen (`data_type == 0x85`).
5. Ab hier laufen 84-Byte-Capsules ohne weitere Anforderung durch.

### Aufbau einer Dense-Capsule

| Offset | Größe | Inhalt |
|---|---|---|
| 0 | 1 | oberes Nibble `0xA` (Sync), unteres = Prüfsumme Bits 0–3 |
| 1 | 1 | oberes Nibble `0x5` (Sync), unteres = Prüfsumme Bits 4–7 |
| 2 | 2 | `start_angle_q6` — Bits 0–14 Winkel in 1/64°, Bit 15 = Umlaufmarke |
| 4 | 80 | 40 × `u16` Distanz in **mm**; 0 = kein Echo |

Prüfsumme = XOR über die Bytes 2…83.

### Winkel je Messung

Der Winkel ergibt sich erst aus der **nächsten** Capsule. Der Dekoder hält
deshalb immer eine Capsule zurück:

```
prev_q8 = prev.start_angle_q6 << 2
cur_q8  = cur.start_angle_q6  << 2
diff_q8 = cur_q8 - prev_q8
if prev_q8 > cur_q8: diff_q8 += 360 << 8      # Umlauf
inc_q16 = (diff_q8 << 8) / 40                 # Schritt je Messung

angle_q16 = prev.start_angle_q6 << 10
for i in 0..39:
    angle[i] = (angle_q16 >> 10) / 64.0
    angle_q16 += inc_q16
```

Das entspricht `_dense_capsuleToNormal` im Slamtec-SDK. Implementiert in
`host/scan3d/rplidar.py` und `firmware/src/dense_capsule.h`; beide liefern
dieselben Winkel.

**Umlaufmarke.** Das SDK erkennt den 360°-Durchgang über einen Modulo-Trick,
der eine Lücke hat: endet eine Capsule exakt auf der Grenze, wird der Durchgang
nie gemeldet. Beide Implementierungen hier rechnen stattdessen den Index direkt
aus und tragen ihn auf die nächste Capsule vor, falls er hinter deren letzte
Messung fällt. Getestet in `test_full_revolution_sample_count` (Capsules exakt
auf der Grenze) und `test_revolution_marker_lands_inside_capsule`.

---

## 2. ESP32 → Host

TCP, Port 5005. Alles little endian.

### Header, 8 Byte

| Offset | Typ | Feld |
|---|---|---|
| 0 | u16 | Magic `0x4E57` (`'NW'`) |
| 2 | u8 | Typ: 0 = HELLO, 1 = CAPSULE, 2 = STATUS, 3 = SCAN |
| 3 | u8 | Flags |
| 4 | u16 | Sequenznummer |
| 6 | u16 | Länge der Nutzlast |

Flags: Bit 0 = Umlaufmarke, Bit 1 = Sweep läuft, Bit 2 = Rückfahrt.

Nur Frames mit gesetztem Bit 1 gehören zum Sweep; alles andere darf der Host
verwerfen. Welcher Datenframe kommt — CAPSULE beim S2, SCAN beim C1 —,
entscheidet die Firmware nach angeschlossenem Gerät; Host und iOS-App
verarbeiten beide.

### CAPSULE, 96 Byte Nutzlast (104 Byte gesamt)

| Offset | Typ | Feld |
|---|---|---|
| 0 | u32 | `yaw_start_q16` — Gierwinkel der ersten Messung, Grad × 65536 |
| 4 | u32 | `yaw_end_q16` — Gierwinkel nach der letzten Messung |
| 8 | i32 | `alpha_inc_q16` — Winkelschritt je Messung, Grad × 65536 |
| 12 | u16 | `alpha_q6` — Scanwinkel der ersten Messung, Grad × 64 |
| 14 | u16 | reserviert |
| 16 | 80 | 40 × u16 Distanz in mm |

Der Host interpoliert Gier- und Scanwinkel über die 40 Messungen linear.

**Beim aktuellen Aufbau sind `yaw_start` und `yaw_end` immer gleich**, weil die
Achse während der Messung stillsteht (Schritt und Halt, siehe
`02-geometrie.md`). Die beiden Felder bleiben trotzdem getrennt: so bleibt das
Protokoll für einen Aufbau mit Dauerfahrt verwendbar, ohne es zu ändern, und
der Host muss nichts unterscheiden — die Interpolation über eine Spanne von
null ergibt schlicht überall denselben Winkel.

Bandbreite: 800 Capsules/s × 104 Byte ≈ **83 kB/s** (~666 kbit/s). Über WLAN
unproblematisch, über BLE (realistisch 100–200 kbit/s) nicht machbar. Deshalb
WLAN.

### SCAN, 12 + 4·n Byte Nutzlast (n ≤ 32)

| Offset | Typ | Feld |
|---|---|---|
| 0 | u32 | `yaw_start_q16` — Gierwinkel der ersten Messung, Grad × 65536 |
| 4 | u32 | `yaw_end_q16` — Gierwinkel nach der letzten Messung |
| 8 | u16 | `count` — Anzahl Messungen, höchstens 32 |
| 10 | u16 | reserviert |
| 12 | 4·n | je Messung: `angle_q6 u16` (Grad × 64), `distance_mm u16` |

Der Gegenpart zu CAPSULE für den einfachen Scanmodus. Interpoliert wird hier
**nur der Gierwinkel** — der Scanwinkel steht ja gemessen da. Genau dafür gibt
es diesen Frametyp: die Winkel des C1 sind nicht gleichmäßig verteilt und
ließen sich aus Startwinkel und Schrittweite nicht rekonstruieren.

Ein Frame endet spätestens nach 32 Messungen und immer an einer Umlaufmarke.
Damit gehört er nie zu zwei Umdrehungen, und Bit 0 der Flags gilt für den
ganzen Frame.

Bandbreite: 5000 Messungen/s × 4 Byte plus Rahmen ≈ **23 kB/s**.

### HELLO, 20 Byte Nutzlast

`fw_version u16`, `lidar_rpm u16`, `offset_radial_um i32`,
`offset_axial_um i32`, `yaw_min_q16 u32`, `yaw_max_q16 u32`

Wird beim Verbindungsaufbau gesendet, damit der Host die Einbaulage nicht
doppelt konfiguriert halten muss.

### STATUS, 20 Byte Nutzlast

`sweep_index u16`, `state u8`, reserviert u8, `yaw_q16 u32`, `capsules u32`,
`checksum_errors u32`, `dropped_frames u32`

`state`: 0 = idle, 1 = Anfahrt auf die erste Ebene, 2 = Sweep, 3 = Rückfahrt. Alle 200 ms und bei
jedem Zustandswechsel. Ein STATUS mit `state == 0` nach aktiven Capsules
beendet den Sweep — daran erkennt `collect_sweep()` das Ende.

`dropped_frames > 0` heißt: das WLAN kam nicht mit und die Queue lief über. Die
Wolke hat dann Lücken.

### Kommandos vom Host

Einzelne Bytes: `'S'` startet einen Sweep, `'X'` bricht ab. Bei
`AUTO_START_ON_CONNECT = 1` (Standard) beginnt der Sweep schon beim Verbinden.

---

## Beide Seiten zusammenhalten

`tests/wire_fixture.txt` enthält je einen Beispielframe der drei Typen als
Hex. Firmware (`firmware/test/native/test_main.cpp`) und Host
(`host/tests/test_wire_fixture.py`) prüfen ihre Encoder gegen dieselbe Datei.
Ändert jemand nur eine Seite, schlägt genau ein Test fehl statt Punktwolken
still zu verbiegen.
