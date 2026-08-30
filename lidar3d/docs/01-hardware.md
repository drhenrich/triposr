# Hardware

## Stückliste

| Teil | Anmerkung |
|---|---|
| RPLIDAR S2 | 190 g, 77 × 77 × 38,85 mm, 5 V, >2 W, TTL-UART 3,3 V @ 1 MBaud |
| ESP32-S3 DevKitC-1 | zwei Kerne, genug RAM für die Queue, 3,3-V-UART |
| TMC2209 Steppertreiber | Schrittmodul mit UART-Anschluss (BigTreeTech/Fysetc) |
| NEMA17 Schrittmotor | 200 Vollschritte, ~1,0 A Nennstrom, 40–48 mm |
| GT2-Riemen + 20T/60T Riemenscheiben | 3:1 Untersetzung |
| Rillenkugellager 6808 o. ä. | großer Innendurchmesser, damit das Kabel mittig durchgeht |
| Optischer Endschalter oder Hallsensor | Referenzpunkt der Gierachse |
| Netzteil 12 V / ≥3 A | |
| Step-Down 12 V → 5 V, ≥2 A | für LiDAR und ESP32 |
| Elko 470–1000 µF am 5-V-Zweig | fängt den Anlaufstrom des LiDAR-Motors ab |

## Mechanik

Der springende Punkt: **die Scanebene des LiDAR muss die Drehachse
enthalten.** Der S2 wird also um 90° gekippt montiert — seine Scanebene steht
senkrecht, die Drehachse ist senkrecht. Liegt der LiDAR flach, dreht man ihn
nur um sich selbst und gewinnt keine dritte Dimension.

```
              Drehachse (senkrecht)
                    │
        ┌───────────┼───────────┐
        │      ╲    │    ╱      │   Scanebene des S2,
        │        ╲  │  ╱        │   senkrecht, 360 Grad
        │          ╲│╱          │
        │      ─────●─────      │   optisches Zentrum
        │          ╱│╲          │
        │        ╱  │  ╲        │
        └───────────┼───────────┘
                    │
              ╔═════╧═════╗
              ║  60T      ║  Riemenscheibe, 3:1
              ╚═══════════╝
                 NEMA17
```

**Kein Schleifring.** Weil der LiDAR in seiner Ebene schon 360° misst, genügt
ein Gierbereich von 180° für die volle Kugel (Herleitung in
`02-geometrie.md`). Das Kabel bekommt eine Schlaufe mit etwas Reserve und
läuft durch das hohle Lager nach unten. Bei ~1 A Versorgungsstrom wäre ein
Schleifring das teuerste und unzuverlässigste Bauteil im ganzen Aufbau.

**Warum 3:1 und nicht direkt gekoppelt.** Zwei Gründe. Erstens Auflösung:
200 Vollschritte × 16 Microsteps × 3 = 9600 Schritte pro Umdrehung, also
0,0375° pro Schritt. Zweitens — und wichtiger — Vibration. 190 g auf einem
Ausleger sind eine träge Masse; die Rastmomente eines direkt gekoppelten
Motors gehen ungedämpft in den Sensor und verrauschen die Distanzmessung.
Der Riemen dämpft, und der Motor läuft bei gleicher Gierrate dreimal
schneller, also weiter weg von seinen Resonanzen im unteren Drehzahlbereich.

Bei 10°/s Gierrate ergibt das eine STEP-Frequenz von
10 / 0,0375 = **266,7 Hz** — sehr langsam und mit StealthChop praktisch
lautlos.

**Auswuchten.** Das optische Zentrum des S2 möglichst nah an die Drehachse
setzen. Jeder radiale Versatz erzeugt Unwucht *und* muss später
herauskalibriert werden (`02-geometrie.md`). Ein Gegengewicht auf der
Rückseite des Auslegers ist billiger als eine steifere Achse.

## Verkabelung

### RPLIDAR S2 ↔ ESP32-S3

Der S2 spricht 3,3-V-TTL, der ESP32-S3 auch — **kein Pegelwandler nötig**.

| S2 | ESP32-S3 | Anmerkung |
|---|---|---|
| GND | GND | gemeinsame Masse, großzügig dimensioniert |
| 5 V | **nicht** vom ESP32 | eigener Step-Down, siehe unten |
| TX | GPIO 18 (RX) | |
| RX | GPIO 17 (TX) | |

Die genaue Steckerbelegung steht im Datenblatt, das dem Gerät beiliegt; der
S2 wird üblicherweise mit einem Adapterboard ausgeliefert, das GND, 5 V, TX
und RX herausführt. Vor dem ersten Anschließen mit dem Datenblatt abgleichen —
diese Belegung hier nicht aus dem Kopf annehmen.

### TMC2209 ↔ ESP32-S3

| TMC2209 | ESP32-S3 |
|---|---|
| STEP | GPIO 5 |
| DIR | GPIO 6 |
| EN | GPIO 7 (low-aktiv) |
| UART (PDN) | GPIO 15 / 16 über 1 kΩ |
| VM / GND | 12 V |
| MS1, MS2 | GND (Adresse 0b00) |

Microstepping wird über UART gesetzt, nicht über MS1/MS2 — die beiden Pins
legen nur die UART-Adresse fest.

Endschalter an GPIO 4 gegen GND, interner Pullup ist in der Firmware aktiviert.

## Stromversorgung

Das ist die Stelle, an der solche Aufbauten am häufigsten scheitern.

| Verbraucher | Strom bei 5 V |
|---|---|
| RPLIDAR S2 | >2 W laut Datenblatt, also ≥0,4 A dauerhaft; beim Anlauf des Motors deutlich mehr |
| ESP32-S3 mit WLAN | 0,1–0,3 A, Spitzen beim Senden |

Regeln:

* Der LiDAR bekommt **seinen eigenen 5-V-Zweig** direkt vom Step-Down, nicht
  den 5-V-Pin des ESP32-Devkits und schon gar nicht dessen 3,3-V-Regler.
* Elko 470–1000 µF direkt am 5-V-Eingang des LiDAR. Der Anlaufstrom des
  Motors reißt sonst die Spannung ein, und der LiDAR bootet mitten im Sweep neu.
* Der Motor läuft an 12 V, der TMC2209 mit 600 mA RMS. Das reicht für die
  Last und hält den Treiber kühl.
* Masse sternförmig auf einen Punkt. Ein 1-MBaud-UART verzeiht keine
  Masseschleifen.

## Inbetriebnahme, der Reihe nach

1. **Nur der LiDAR am PC**, über den USB-Adapter:
   `python3 -m scan3d serial /dev/ttyUSB0 --duration 5 --yaw-rate 0`
   Bei `--yaw-rate 0` bleibt der Gierwinkel 0 und die Punktwolke ist ein
   flacher Ring — genau richtig, um Reichweite und Ausfälle zu prüfen. Die
   Pruefsummenfehler in der Ausgabe müssen bei ~0 liegen.
2. **Nur die Achse.** LiDAR abziehen, Firmware flashen, Homing und Sweep
   beobachten. Die Achse muss gleichmäßig und leise laufen; ruckelt sie,
   stimmt der Motorstrom oder die Untersetzung nicht.
3. **Beides zusammen**, aber ohne Drehung (`YAW_SWEEP_RATE_DEG_S` klein).
   Prüfen, dass `dropped_frames` im Status 0 bleibt.
4. **Kalibrieren**, siehe `02-geometrie.md`.
5. **Erster echter Sweep.**
