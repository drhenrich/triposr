# Protokolle

Zwei Protokolle sind im Spiel: das Slamtec-Protokoll zwischen S2 und ESP32,
und das eigene Frameprotokoll zwischen ESP32 und Host.

---

## 1. RPLIDAR S2 → ESP32

UART, **1.000.000 Baud, 8N1, 3,3 V TTL**.

### Warum der dense-capsuled Modus zwingend ist

| Modus | Byte je Messung | bei 32.000/s | passt durch 1 MBaud (~100 kB/s)? |
|---|---|---|---|
| SCAN (0x20) | 5 | 160 kB/s | **nein** |
| dense capsuled (0x82) | 2,1 | 67 kB/s | ja |

Eine Dense-Capsule ist 84 Byte lang und enthält 40 Messungen. Der Winkel wird
nicht mitgeschickt, sondern zwischen den Startwinkeln aufeinanderfolgender
Capsules interpoliert — daher die Ersparnis.

### Kommandos

Ohne Payload: `A5 <cmd>`.
Mit Payload: `A5 <cmd> <len> <payload…> <XOR aller vorherigen Bytes>`.

| Kommando | Byte | Payload |
|---|---|---|
| STOP | 0x25 | — |
| RESET | 0x40 | — |
| EXPRESS_SCAN | 0x82 | `working_mode u8, working_flags u16, param u16` |
| GET_LIDAR_CONF | 0x84 | `type u32` (+ `mode u16` bei modusbezogenen Abfragen) |
| HQ_MOTOR_SPEED_CTRL | 0xA8 | `rpm u16` — 600 rpm = 10 Hz Scanrate |

Der S2 regelt seinen Motor selbst; ein PWM-Pin wie bei der A-Serie entfällt.

### Antwort-Deskriptor

```
A5 5A | length:30 mode:2 (u32 LE) | data_type u8
```

`mode == 1` bedeutet: es folgt ein kontinuierlicher Datenstrom.

### Startsequenz der Firmware

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
| 2 | u8 | Typ: 0 = HELLO, 1 = CAPSULE, 2 = STATUS |
| 3 | u8 | Flags |
| 4 | u16 | Sequenznummer |
| 6 | u16 | Länge der Nutzlast |

Flags: Bit 0 = Umlaufmarke, Bit 1 = Sweep läuft, Bit 2 = Rückfahrt.

Nur Capsules mit gesetztem Bit 1 gehören zum Sweep; alles andere darf der Host
verwerfen.

### CAPSULE, 96 Byte Nutzlast (104 Byte gesamt)

| Offset | Typ | Feld |
|---|---|---|
| 0 | u32 | `yaw_start_q16` — Gierwinkel der ersten Messung, Grad × 65536 |
| 4 | u32 | `yaw_end_q16` — Gierwinkel nach der letzten Messung |
| 8 | i32 | `alpha_inc_q16` — Winkelschritt je Messung, Grad × 65536 |
| 12 | u16 | `alpha_q6` — Scanwinkel der ersten Messung, Grad × 64 |
| 14 | u16 | reserviert |
| 16 | 80 | 40 × u16 Distanz in mm |

Der Host interpoliert Gier- und Scanwinkel über die 40 Messungen linear. Die
Gierspanne einer Capsule beträgt bei 10°/s nur 0,0125° — die Interpolation ist
Feinschliff, kein Muss.

Bandbreite: 800 Capsules/s × 104 Byte ≈ **83 kB/s** (~666 kbit/s). Über WLAN
unproblematisch, über BLE (realistisch 100–200 kbit/s) nicht machbar. Deshalb
WLAN.

### HELLO, 20 Byte Nutzlast

`fw_version u16`, `lidar_rpm u16`, `offset_radial_um i32`,
`offset_axial_um i32`, `yaw_min_q16 u32`, `yaw_max_q16 u32`

Wird beim Verbindungsaufbau gesendet, damit der Host die Einbaulage nicht
doppelt konfiguriert halten muss.

### STATUS, 20 Byte Nutzlast

`sweep_index u16`, `state u8`, reserviert u8, `yaw_q16 u32`, `capsules u32`,
`checksum_errors u32`, `dropped_frames u32`

`state`: 0 = idle, 1 = Homing, 2 = Sweep, 3 = Rückfahrt. Alle 200 ms und bei
jedem Zustandswechsel. Ein STATUS mit `state == 0` nach aktiven Capsules
beendet den Sweep — daran erkennt `collect_sweep()` das Ende.

`dropped_frames > 0` heißt: das WLAN kam nicht mit und die Queue lief über. Die
Wolke hat dann Lücken.

### Kommandos vom Host

Einzelne Bytes: `'S'` startet Homing und Sweep, `'X'` bricht ab. Bei
`AUTO_START_ON_CONNECT = 1` (Standard) beginnt der Sweep schon beim Verbinden.

---

## Beide Seiten zusammenhalten

`tests/wire_fixture.txt` enthält je einen Beispielframe der drei Typen als
Hex. Firmware (`firmware/test/native/test_main.cpp`) und Host
(`host/tests/test_wire_fixture.py`) prüfen ihre Encoder gegen dieselbe Datei.
Ändert jemand nur eine Seite, schlägt genau ein Test fehl statt Punktwolken
still zu verbiegen.
