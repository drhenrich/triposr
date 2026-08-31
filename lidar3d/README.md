# lidar3d — 3D-Scanner aus einem 2D-LiDAR

Ein 2D-LiDAR wird von einem Servo um eine senkrechte Achse gedreht.
Jede Messung ist damit eine Kugelkoordinate, und aus dem 2D-Scanner wird ein
3D-Scanner. Der Aufbau steht still und scannt seine Umgebung — es wird kein
ARCore und kein Kameratracking gebraucht.

Das ist der gleiche Ansatz wie in der Vorlage von `northworkslab`.
Gearbeitet wird mit einem **RPLIDAR C1**; die Hostseite kann daneben auch den
größeren S2, weil der Entwurf zunächst darauf ausgelegt war.

## Der verwendete Sensor

| | RPLIDAR C1 | RPLIDAR S2 |
|---|---|---|
| Reichweite | 12 m (weiß) | 30 m (weiß) |
| Messrate | **5.000/s** | 32.000/s |
| Winkelauflösung | ~0,72° | 0,1125° |
| Scanrate | 8–12 Hz | 8–15 Hz |
| Schnittstelle | **460.800 Baud TTL** | 1 MBaud TTL |
| Gewicht | ~110 g | 190 g |
| Schutzklasse | IP54 | IP65 |

Daraus folgt für den Entwurf:

1. **Der einfache SCAN-Modus genügt.** 5.000 Messungen/s × 5 Byte sind 25 kB/s,
   durch 460.800 Baud passen rund 46 kB/s. Beim S2 wäre das nicht gegangen
   (160 kB/s durch 100 kB/s) — dort ist der *dense-capsuled* Express-Modus
   zwingend. Beide Dekoder liegen vor und sind getestet.
2. **BLE scheidet aus.** Auch die 5.000 Messungen/s sind noch rund 200 kbit/s
   Nutzlast, und das bei BLEs realistischer Obergrenze. Gestreamt wird über
   **TCP** — wahlweise über das USB-C-Kabel oder über WLAN (siehe unten).
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
Messungen je Ebene   500
Aufloesung in Ebene  0.7200 deg
Punkte gesamt        90000
```

27 Sekunden für 90.000 Punkte. Die Dauer hängt an der **Scanrate** (10 Hz), nicht
an der Messrate — mit einem S2 wären es dieselben 27 s, nur mit 576.000 Punkten.
Bei 0,5° Ebenenabstand verdoppelt sich beides.

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

Und: **das iPhone versorgt den Scanner nicht.** Der LiDAR braucht 5 V, der
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
  scan3d/           Dekoder (C1 und S2), Geometrie, PLY-Export, CLI, Leser
  app/              Streamlit-Oberflaeche: Live 2D, 3D aufnehmen, Diagnose
  tests/            100 Tests
tests/wire_fixture.txt  gemeinsame Byte-Fixture aller drei Protokollseiten
docs/              Hardware, Geometrie/Kalibrierung, Protokolle, iOS/USB-C
```

## Loslegen

> **Achtung:** Das Projekt liegt auf dem Branch
> `claude/rplidar-s2-robot-q5c9aq`, nicht auf `main`. Auf `main` gibt es den
> Ordner `lidar3d/` nicht.
>
> ```bash
> git fetch origin
> git checkout claude/rplidar-s2-robot-q5c9aq
> ```

**Der C1 am USB — mit Oberfläche.** Das ist der schnellste Weg zu sichtbaren
Daten. Alle Pfade hier vom **Wurzelverzeichnis des Repos** aus:

```bash
pip install -r lidar3d/host/requirements-app.txt
streamlit run lidar3d/host/app/streamlit_app.py
```

Die App setzt ihren Suchpfad selbst — sie läuft aus jedem Verzeichnis, solange
der Pfad zur Datei stimmt.

Startet mit simuliertem Raum, läuft also sofort auch ohne Hardware. In der
Seitenleiste umschalten, sobald der C1 dranhängt (Port, 460800 Baud). Details in
[`host/app/README.md`](host/app/README.md).

**Ohne Oberfläche, nur Kommandozeile:**

```bash
cd lidar3d/host
python3 -m scan3d simulate --step 1 --color -o test.ply     # ohne Hardware
python3 -m scan3d serial /dev/ttyUSB0 --duration 20 -o scan.ply
```

`serial` nimmt standardmäßig 460800 Baud und den einfachen Scanmodus (C1);
für den S2 `--baudrate 1000000 --mode dense`.

**Vollständiger Aufbau:**

```bash
cd lidar3d/firmware && pio run -e usb -t upload   # mit USB-C; -e wifi nur WLAN
cd ../host && python3 -m scan3d capture 192.168.7.1 --color -o scan.ply
```

Für die Live-Ansicht auf dem iPhone siehe [`ios/README.md`](ios/README.md) —
Xcode-Projekt in fünf Schritten, die Quellen liegen fertig da.

Sobald sich ein Client verbindet, fährt die Achse einen Sweep.

## Tests

```bash
cd lidar3d/host && python3 -m unittest discover -s tests -t .   # 100 Tests
make -C lidar3d/firmware/test/native                           # 129 Prüfungen
cd lidar3d/ios/LidarKit && swift test                          # nur auf dem Mac
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
