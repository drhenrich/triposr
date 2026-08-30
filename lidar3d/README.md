# lidar3d — 3D-Scanner aus einem RPLIDAR S2

Ein 2D-LiDAR wird von einem Servo um eine senkrechte Achse gedreht.
Jede Messung ist damit eine Kugelkoordinate, und aus dem 2D-Scanner wird ein
3D-Scanner. Der Aufbau steht still und scannt seine Umgebung — es wird kein
ARCore und kein Kameratracking gebraucht.

Das ist der gleiche Ansatz wie in der Vorlage von `northworkslab`, aber mit
einem RPLIDAR S2 statt eines kleinen LD19/LD06-Moduls. Der S2 ist deutlich
stärker — und genau das ändert einige Entscheidungen im Aufbau.

## Was der S2 am Konzept ändert

| | LD19/LD06-Klasse | RPLIDAR S2 |
|---|---|---|
| Reichweite | ~12 m | 30 m (weiß), ≥10 m (schwarz) |
| Messrate | ~4.500/s | **32.000/s** |
| Gewicht | ~40 g | **190 g** |
| Schnittstelle | 230 kBaud | **1 MBaud TTL 3,3 V** |
| Versorgung | 5 V, <1 W | 5 V, >2 W |
| Maße | ~38 mm | 77 × 77 × 38,85 mm |

Vier Konsequenzen, die den Entwurf bestimmen:

1. **Der einfache SCAN-Modus reicht nicht.** 32.000 Messungen/s × 5 Byte sind
   160 kB/s, durch 1 MBaud passen aber nur ~100 kB/s. Deshalb muss der
   *dense-capsuled* Express-Modus benutzt werden: 40 Messungen in 84 Byte,
   also 2,1 Byte pro Messung bzw. ~67 kB/s. Das passt.
2. **BLE scheidet aus.** Der Datenstrom zum Handy sind ~640 kbit/s. BLE schafft
   realistisch 100–200 kbit/s. Gestreamt wird deshalb über **TCP** — wahlweise
   über das USB-C-Kabel oder über WLAN (siehe unten).
3. **190 g brauchen ein Lager, nicht mehr Motor.** Die Gierachse ist ein
   Feetech STS3215 (12 V, 30 kg·cm, 1:345). Sein Drehmoment ist reichlich; das
   Kippmoment der 190 g nimmt ein Rillenkugellager auf, nicht die
   Servo-Abtriebswelle.
4. **Kein Schleifring nötig.** Der LiDAR misst in seiner Ebene bereits volle
   360°. Ein Gierbereich von **180° deckt deshalb die komplette Kugel ab**.
   Eine Kabelschlaufe genügt; das spart den Schleifring, der bei ~1 A
   Versorgungsstrom sonst der teuerste und unzuverlässigste Teil wäre.

## Schritt und Halt statt Dauerfahrt

Die Achse dreht **nicht** kontinuierlich. Sie fährt Ebene für Ebene an, rastet
ein und lässt genau eine LiDAR-Umdrehung aufnehmen.

Der Anlass war der Servo: 10°/s wären 3,7 % seiner Leerlaufdrehzahl, und so
weit unten regelt ein 1:345-Getriebe schlecht. Der Gewinn geht aber weit
darüber hinaus:

* Der Gierwinkel wird am **Absolutencoder gemessen**, nicht aus der Zeit
  hochgerechnet. Verlorene Schritte gibt es als Fehlerbild nicht mehr.
* **Kein Endschalter, keine Referenzfahrt** — der Encoder ist absolut.
* **Keine Zeitsynchronisation.** Bei Rotationsscannern mit Dauerfahrt ist die
  Zuordnung von Messzeitpunkt zu Drehwinkel die Hauptfehlerquelle. Hier stellt
  sich die Frage nicht.
* Während der Messung steht die Achse still: keine Bewegungsunschärfe, keine
  Getriebevibration im Sensor.

```
$ python3 -m scan3d plan --span 180 --step 1
Gierbereich          180.0 deg
Ebenenabstand        1.000 deg
Scanebenen           180
Sweep-Dauer          27.0 s
Zeit je Ebene        150 ms
Messungen je Ebene   3200
Aufloesung in Ebene  0.1125 deg
Punkte gesamt        576000
```

27 Sekunden für 576.000 Punkte — der Preis gegenüber 18 s bei Dauerfahrt. Bei
0,5° Ebenenabstand sind es 54 s und 1,15 Mio. Punkte.

## iPhone am USB-C

Zielaufbau ist ein iPhone 17 Pro am Kabel, mit der Punktwolke live auf dem
Display. Der entscheidende Punkt vorweg:

**Ein serielles USB-Gerät kann eine iPhone-App nicht ansprechen.** iOS reicht
generische USB-Peripherie nicht durch, `ExternalAccessory` verlangt
MFi-Zertifizierung, und DriverKit gibt es auf macOS und iPadOS — aber nicht
auf dem iPhone. Der USB-C-Anschluss ändert daran nichts.

**Ein USB-*Netzwerk*gerät dagegen schon.** Seit iOS 17 unterstützen iPhones
USB-Ethernet-Adapter über die Geräteklasse CDC-NCM, mit Bordmitteln und ohne
MFi. Der ESP32-S3 meldet sich also als Netzwerkinterface an, vergibt dem
iPhone per DHCP eine Adresse, und der TCP-Server ist darüber erreichbar.

Der Gewinn: **am Protokoll ändert sich nichts.** Derselbe Stream, derselbe
Decoder, dieselbe Ansicht — egal ob Kabel oder WLAN. Und das iPhone behält am
Kabel sein eigenes WLAN und Mobilfunknetz, statt sich in das Netz des Scanners
einbuchen zu müssen.

| Weg | Scanner | iPhone |
|---|---|---|
| USB-C (NCM) | `192.168.7.1:5005` | `192.168.7.2` per DHCP |
| WLAN (AP) | `192.168.4.1:5005` | `192.168.4.2` per DHCP |

Zwei Fallstricke, die einen Abend kosten können, stehen ausführlich in
[`docs/04-ios-usb.md`](docs/04-ios-usb.md): iOS fragt DHCP **genau einmal**
beim Link-Up und wiederholt es nie (die Firmware zieht den Link deshalb erst
hoch, wenn alles andere steht), und ohne
`NSLocalNetworkUsageDescription` in der `Info.plist` blockiert iOS jede
Verbindung — stillschweigend.

Und: **das iPhone versorgt den Scanner nicht.** Der S2 will >2 W, der
Servo 12 V. Das Kabel überträgt nur Daten, der Scanner hat sein eigenes
Netzteil.

## Aufbau des Repos

```
firmware/          ESP32-S3, PlatformIO
  src/dense_capsule.h   S2-Dekoder      (hardwarefrei, nativ getestet)
  src/angle_util.h      Winkelfestkomma (hardwarefrei, nativ getestet)
  src/stream_proto.h    Frame-Layout    (hardwarefrei, nativ getestet)
  src/rplidar_s2.*      UART-Anbindung
  src/feetech_bus.h     STS3215-Busprotokoll (hardwarefrei, nativ getestet)
  src/sweep_plan.h      Ebenenaufteilung     (hardwarefrei, nativ getestet)
  src/feetech_servo.*   Servo am Halbduplex-Bus
  src/yaw_axis.*        Schritt-und-Halt-Automat
  src/usb_ncm.*         USB-C als USB-Ethernet zum iPhone
  src/main.cpp          Tasks, Netz, TCP
  test/native/          Logiktests mit g++, ohne Hardware
ios/               iPhone-App mit Echtzeit-3D
  LidarKit/         Swift Package: Protokoll, Geometrie, TCP (testbar)
  ScannerApp/       SwiftUI + Metal-Renderer
host/              Python, nur Standardbibliothek im Kern
  scan3d/           Dekoder, Geometrie, PLY-Export, CLI
  tests/            65 Tests
tests/wire_fixture.txt  gemeinsame Byte-Fixture aller drei Protokollseiten
docs/              Hardware, Geometrie/Kalibrierung, Protokolle, iOS/USB-C
```

## Loslegen

**Ohne Hardware** — die ganze Kette einmal durchspielen:

```bash
cd host
python3 -m scan3d simulate --step 1 --color -o test.ply
```

Erzeugt einen simulierten 6 × 4 × 2,6 m Raum mit dem echten Sweep-Muster.
Die PLY-Datei öffnet sich in CloudCompare, MeshLab oder Blender.

**Nur der LiDAR am PC** — zum Einfahren, bevor die Firmware läuft:

```bash
pip install pyserial
python3 -m scan3d serial /dev/ttyUSB0 --duration 20 --yaw-rate 10 -o scan.ply
```

**Vollständiger Aufbau:**

```bash
cd firmware && pio run -e usb -t upload    # mit USB-C; -e wifi baut nur WLAN
cd ../host && python3 -m scan3d capture 192.168.7.1 --color -o scan.ply
```

Für die Live-Ansicht auf dem iPhone siehe [`ios/README.md`](ios/README.md) —
Xcode-Projekt in fünf Schritten, die Quellen liegen fertig da.

Sobald sich ein Client verbindet, fährt die Achse einen Sweep.

## Tests

```bash
cd host && python3 -m unittest discover -s tests -t .   # 65 Tests
make -C firmware/test/native                            # 125 Prüfungen
cd ios/LidarKit && swift test                           # nur auf dem Mac
```

Alle drei Implementierungen des Protokolls — Python, C++ und Swift — prüfen
ihr Byte-Layout gegen dieselbe Datei `tests/wire_fixture.txt`. Ändert jemand
nur eine Seite, schlägt genau ein Test fehl, statt dass Punktwolken still
verbogen werden.

## Stand

**Getestet und grün** ist alles, was ohne Hardware prüfbar ist: der
Capsule-Dekoder, die Winkelinterpolation, das Feetech-Busprotokoll, die
Ebenenaufteilung, die Encoderumrechnung, die Geometrie und das Frameprotokoll
(Python und C++, byteweise gegeneinander).

**Nicht kompiliert** — in dieser Umgebung fehlten die Toolchains:

| Teil | warum ungeprüft |
|---|---|
| `firmware/src/rplidar_s2.cpp`, `feetech_servo.cpp`, `yaw_axis.cpp`, `main.cpp` | keine PlatformIO-Toolchain |
| `firmware/src/usb_ncm.cpp` und der IDF-Build (`env:usb`) | dito; zusätzlich hat sich die `esp_tinyusb`-API zwischen IDF-Versionen mehrfach geändert — vor dem Flashen gegen das Beispiel `tusb_ncm` der eigenen Version abgleichen |
| `ios/` (alles) | kein Swift, kein Xcode |

Die Swift-Seite bringt aber eigene Tests mit: ein `swift test` auf dem Mac
prüft den Decoder gegen dieselbe Byte-Fixture wie die anderen beiden
Implementierungen — bevor Hardware im Spiel ist.

Vor dem ersten Einschalten `docs/01-hardware.md` lesen, besonders den
Abschnitt zur Stromversorgung.

## Weiter

* `docs/01-hardware.md` — Stückliste, Mechanik, Verkabelung, Strombedarf
* `docs/02-geometrie.md` — Mathematik, Abdeckung, Kalibrierung
* `docs/03-protokolle.md` — S2-Protokoll und Streamformat
* `docs/04-ios-usb.md` — iPhone am USB-C: was geht und was nicht
* `ios/README.md` — App bauen und bedienen
