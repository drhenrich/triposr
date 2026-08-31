# Hardware

## Stückliste

| Teil | Anmerkung |
|---|---|
| RPLIDAR S2 | 190 g, 77 × 77 × 38,85 mm, 5 V, >2 W, TTL-UART 3,3 V @ 1 MBaud |
| ESP32-S3 DevKitC-1 | zwei Kerne, genug RAM für die Queue, 3,3-V-UART |
| Feetech STS3215 Busservo | Ausführung **12 V / 30 kg·cm / magnetische Kodierung** (C018), 1:345, 12-bit-Absolutencoder, TTL halbduplex, ab Werk ID 1 und 1 MBaud |
| Bustreiber für den Servo | Feetech FE-URT-1 oder ein 74HC241 — oder der ESP32 schaltet die Richtung selbst, siehe unten |
| Rillenkugellager 6808 o. ä. | großer Innendurchmesser, damit das Kabel mittig durchgeht |
| Netzteil 12 V / ≥3 A | |
| Step-Down 12 V → 5 V, ≥2 A | für LiDAR und ESP32 |
| Elko 470–1000 µF am 5-V-Zweig | fängt den Anlaufstrom des LiDAR-Motors ab |

Was gegenüber einem Schrittmotoraufbau **entfällt**: Steppertreiber, NEMA17,
Riemen, beide Riemenscheiben und der Endschalter. Der Encoder des STS3215 ist
absolut — es gibt keine Referenzfahrt und keinen verlorenen Schritt.

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
              ║  Lager    ║  nimmt Gewicht und Moment auf
              ╠═══════════╣
              ║  STS3215  ║  liefert nur das Drehmoment
              ╚═══════════╝
```

**Kein Schleifring.** Weil der LiDAR in seiner Ebene schon 360° misst, genügt
ein Gierbereich von 180° für die volle Kugel (Herleitung in
`02-geometrie.md`). Das Kabel bekommt eine Schlaufe mit etwas Reserve und
läuft durch das hohle Lager nach unten. Bei ~1 A Versorgungsstrom wäre ein
Schleifring das teuerste und unzuverlässigste Bauteil im ganzen Aufbau.

**Der Servo trägt nicht, er dreht nur.** Das ist die wichtigste mechanische
Regel hier. 190 g auf einem Ausleger erzeugen ein Kippmoment, das nichts auf
der Servo-Abtriebswelle zu suchen hat — die ist auf Drehmoment ausgelegt, nicht
auf Querkraft. Das Rillenkugellager nimmt Gewicht und Moment auf, der Servo
greift nur an. Ohne Lager verschleißt das Getriebe, und das Spiel wächst.

**Schritt und Halt statt Dauerfahrt.** Die Achse dreht nicht kontinuierlich,
sondern fährt Ebene für Ebene an, rastet ein und lässt genau eine
LiDAR-Umdrehung aufnehmen. Der Grund: 10°/s wären 3,7 % der Leerlaufdrehzahl
des Servos (270°/s bei 12 V), und so weit unten regelt ein 1:345-Getriebe
schlecht. Begründung und Konsequenzen stehen in `firmware/src/yaw_axis.h`.

Nebeneffekt: Getriebevibration stört nicht, weil während der Messung
stillgestanden wird. Bei einem Schrittmotor mit Dauerfahrt wäre das der
wichtigste Störeinfluss gewesen.

**Auflösung.** 4096 Zählwerte auf 360° sind 0,0879° je Zählwert. Bei 1°
Ebenenabstand ist das reichlich; relevant würde es erst unter ~0,2°.

**Getriebespiel.** Das ist der Punkt, den man beim STS3215 messen sollte. Zwei
Entschärfungen: der Encoder sitzt auf der Abtriebswelle, misst also den echten
Ausgangswinkel und nicht die Motorstellung; und der Sweep läuft nur in eine
Richtung, Spiel schlägt allein bei der Rückfahrt zu. Die Firmware übernimmt
zusätzlich den **gemessenen** Winkel je Ebene, nicht den befohlenen — eine
bleibende Regelabweichung landet damit in den Daten statt im Fehler.

**Auswuchten.** Das optische Zentrum des S2 möglichst nah an die Drehachse
setzen. Jeder radiale Versatz erzeugt Unwucht *und* muss später
herauskalibriert werden (`02-geometrie.md`). Ein Gegengewicht auf der
Rückseite des Auslegers ist billiger als eine steifere Achse.

## Verkabelung

### RPLIDAR C1 ↔ ESP32-S3

Der C1 spricht 3,3-V-TTL, der ESP32-S3 auch — **kein Pegelwandler nötig**.
Der USB-C-Adapter bleibt weg; er wird nur gebraucht, wenn der LiDAR an einem
PC hängt.

| Kabel | Signal | an den ESP32-S3 | Anmerkung |
|---|---|---|---|
| ● rot | VCC | **nicht** vom ESP32 | 5 V (4,8–5,2 V), eigener Zweig — siehe Stromversorgung |
| ● gelb | TX | **GPIO 18** (RX) | der LiDAR sendet, der ESP32 empfängt |
| ● grün | RX | **GPIO 17** (TX) | der ESP32 sendet, der LiDAR empfängt |
| ● schwarz | GND | GND | gemeinsame Masse, großzügig dimensioniert |

**Die Datenleitungen werden überkreuzt.** Gelb an RX, grün an TX — was der
eine sendet, empfängt der andere. Vertauscht kommt kein einziges Byte an, und
zwar völlig geräuschlos: der LiDAR dreht sich trotzdem, weil der Motor
unabhängig läuft.

Feste Baudrate **460.800**, in `config.h` als `LIDAR_BAUDRATE` eingetragen.
Der C1 kann nichts anderes.

> **Achtung, Pinbelegung prüfen.** GPIO 17 ist hier der LiDAR-TX. Ältere
> Aufbauten und viele Beispielsketche legen den **Servo** auf 16/17
> (`SerialServo.begin(1000000, SERIAL_8N1, 16, 17)`). Dann kollidiert der
> Servo-TX mit dem LiDAR-TX, und einer von beiden schweigt. In `config.h`
> liegt der Servo auf **15/16** — entweder danach verdrahten, oder die
> `#define`s an die eigene Verdrahtung anpassen. Zwei UARTs dürfen sich
> keinen Pin teilen.

### STS3215 ↔ ESP32-S3

Der Servo hat einen **halbduplexen** Ein-Draht-Bus: Senden und Empfangen
teilen sich dieselbe Leitung. Zwei Wege, das anzuschließen:

**a) Der ESP32 schaltet selbst** (`SERVO_DIR_PIN 7` in `config.h`). Die
Firmware setzt die UART in den RS485-Halbduplexmodus; der Pin steuert dann
einen Transceiver, und der Empfang ist während des Sendens abgeschaltet — das
eigene Echo landet gar nicht erst im Puffer.

**b) Ein Adapterboard übernimmt es** (Feetech FE-URT-1 o. ä.). Dann
`SERVO_DIR_PIN` auf `-1` setzen.

| Signal | ESP32-S3 |
|---|---|
| Servo-Bus TX | GPIO 15 |
| Servo-Bus RX | GPIO 16 |
| Richtungsumschaltung (DE/RE) | GPIO 7, oder −1 bei externem Adapter |
| Servo-Versorgung | 12 V, gemeinsame Masse |

Bus-Baudrate 1 MBaud, Servo-ID 1 (Werkseinstellung).

Beides ist in `config.h` einstellbar. **Register unter Adresse 40 nicht
blind beschreiben** — dort liegen ID (5) und Baudrate (6) im EPROM, ein
Fehlgriff macht den Servo unerreichbar. Die Firmware fasst sie nicht an.

## Stromversorgung

Das ist die Stelle, an der solche Aufbauten am häufigsten scheitern.

| Verbraucher | Strom bei 5 V |
|---|---|
| RPLIDAR S2 | >2 W laut Datenblatt, also ≥0,4 A dauerhaft; beim Anlauf des Motors deutlich mehr |
| ESP32-S3 mit WLAN | 0,1–0,3 A, Spitzen beim Senden |

Regeln:

* Der LiDAR bekommt **seinen eigenen 5-V-Zweig** direkt vom Step-Down, nicht
  den 5-V-Pin des ESP32-Devkits und schon gar nicht dessen 3,3-V-Regler. Das
  rote Kabel gehört dorthin, nicht an das Board.
* Elko 470–1000 µF direkt am 5-V-Eingang des LiDAR. Der Anlaufstrom des
  Motors reißt sonst die Spannung ein, und der LiDAR bootet mitten im Sweep neu.

> **`rst:0xf (BROWNOUT_RST)`** im seriellen Log heißt genau das: die Spannung
> ist unter die Schwelle gefallen und der ESP32 hat sich neu gestartet. Beim
> Hochlauf mag das nur lästig sein — mitten im Sweep ist die halbe Wolke weg,
> und ohne diesen Hinweis sucht man den Fehler in der Software.
>
> Die Firmware liest den Grund des letzten Neustarts aus und zeigt ihn an:
> auf der seriellen Konsole als `ACHTUNG:` und auf der Webseite als Hinweis
> im Meldungskasten. Abhilfe ist immer dieselbe: eigener 5-V-Zweig für den
> LiDAR, Elko dicht am Stecker, kurze und dicke Masseleitung.
* Der Servo läuft an 12 V. Im Leerlauf zieht er nur 180 mA, blockiert aber
  2,7 A — das Netzteil muss die Spitze beim Anfahren liefern können, sonst
  bricht die Spannung ein und der ESP32 startet neu.
* Masse sternförmig auf einen Punkt. Ein 1-MBaud-UART verzeiht keine
  Masseschleifen — und hier laufen zwei davon.

## Inbetriebnahme, der Reihe nach

1. **Nur der LiDAR am PC**, über den USB-Adapter:
   `python3 -m scan3d serial /dev/ttyUSB0 --duration 5 --yaw-rate 0`
   Bei `--yaw-rate 0` bleibt der Gierwinkel 0 und die Punktwolke ist ein
   flacher Ring — genau richtig, um Reichweite und Ausfälle zu prüfen. Die
   Pruefsummenfehler in der Ausgabe müssen bei ~0 liegen.
2. **Nur der Servo.** LiDAR abziehen, Firmware flashen. Antwortet der Servo
   nicht, bricht die Firmware mit einer Meldung ab, statt eine Wolke mit
   erfundenen Winkeln aufzunehmen. Prüfen: Bus-ID (Werk: 1), 1 MBaud,
   Halbduplex-Verdrahtung, 12 V.

   Die Startmeldung nennt die ausgelesene **Servo-Modellnummer**. Kommt dort
   eine 0 oder etwas Unerwartetes, hängt am Bus nicht das, was man denkt —
   das ist der schnellste Test, ob die Verdrahtung stimmt.
3. **Getriebespiel messen.** Servo auf eine Position fahren, von Hand leicht
   in beide Richtungen drücken und die zurückgemeldete Position ablesen. Die
   Differenz ist das Spiel; sie geht als Rauschen in den Gierwinkel ein.
4. **Beides zusammen**, aber mit grobem Ebenenabstand
   (`YAW_PLANE_STEP_DEG 10.0`) für einen schnellen Durchlauf. Prüfen, dass
   `dropped_frames` im Status 0 bleibt.
5. **Kalibrieren**, siehe `02-geometrie.md`.
6. **Erster echter Sweep.**
