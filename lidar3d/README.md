# lidar3d — 3D-Scanner aus einem RPLIDAR S2

Ein 2D-LiDAR wird von einem Schrittmotor um eine senkrechte Achse gedreht.
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
   realistisch 100–200 kbit/s. Die Firmware streamt daher über **WLAN/TCP**.
3. **190 g wollen eine Untersetzung.** Der Entwurf nutzt einen NEMA17 mit
   GT2-Riemen 20T→60T (3:1). Das dämpft Vibrationen — der wichtigste
   Störeinfluss auf die Distanzmessung während der Fahrt — und ergibt
   0,0375° pro Microstep.
4. **Kein Schleifring nötig.** Der LiDAR misst in seiner Ebene bereits volle
   360°. Ein Gierbereich von **180° deckt deshalb die komplette Kugel ab**.
   Eine Kabelschlaufe genügt; das spart den Schleifring, der bei ~1 A
   Versorgungsstrom sonst der teuerste und unzuverlässigste Teil wäre.

## Ein Sweep in Zahlen

```
$ python3 -m scan3d plan --span 180 --step 1
Gierbereich          180.0 deg
Ebenenabstand        1.000 deg
Scanebenen           180
Sweep-Dauer          18.0 s
Gierrate             10.000 deg/s
Messungen je Ebene   3200
Aufloesung in Ebene  0.1125 deg
Punkte gesamt        576000
```

18 Sekunden für 576.000 Punkte. Bei 0,5° Ebenenabstand sind es 36 s und
1,15 Mio. Punkte.

Nebenbei fällt hier ein beruhigendes Ergebnis ab: bei 10°/s entspricht **1 ms
Latenz nur 0,01° Gierfehler**. Die Zeitsynchronisation zwischen LiDAR-Paket und
Schrittzähler ist damit unkritisch — was bei diesem Aufbautyp sonst das größte
Genauigkeitsproblem ist.

## Aufbau des Repos

```
firmware/          ESP32-S3, PlatformIO
  src/dense_capsule.h   S2-Dekoder      (hardwarefrei, nativ getestet)
  src/yaw_model.h       Gier-Festkomma  (hardwarefrei, nativ getestet)
  src/stream_proto.h    Frame-Layout    (hardwarefrei, nativ getestet)
  src/rplidar_s2.*      UART-Anbindung
  src/yaw_axis.*        TMC2209 + LEDC-Schrittimpulse
  src/main.cpp          Tasks, WLAN, TCP
  test/native/          Logiktests mit g++, ohne Hardware
host/              Python, nur Standardbibliothek im Kern
  scan3d/           Dekoder, Geometrie, PLY-Export, CLI
  tests/            64 Tests
tests/wire_fixture.txt  gemeinsame Byte-Fixture beider Protokollseiten
docs/              Hardware, Geometrie/Kalibrierung, Protokolle
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
cd firmware && pio run -t upload
# Handy oder PC mit dem WLAN "lidar3d" verbinden
cd ../host && python3 -m scan3d capture 192.168.4.1 --color -o scan.ply
```

Sobald sich ein Client verbindet, referenziert die Achse auf den Endschalter
und fährt einen Sweep.

## Tests

```bash
cd host && python3 -m unittest discover -s tests -t .   # 64 Tests
make -C firmware/test/native                            # 49 Prüfungen
```

Beide Seiten prüfen das Frame-Layout gegen dieselbe Datei
`tests/wire_fixture.txt`. Ändert jemand nur eine Seite des Protokolls,
schlägt genau ein Test fehl.

## Stand

Getestet ist alles, was ohne die Hardware testbar ist: der Capsule-Dekoder,
die Winkelinterpolation, die Gier-Festkommamathematik, die Geometrie und beide
Seiten des Frameprotokolls.

**Nicht verifiziert** sind die Teile, die echte Hardware brauchen: die
UART-Anbindung an den S2 (`rplidar_s2.cpp`), die TMC2209-Ansteuerung
(`yaw_axis.cpp`) und `main.cpp`. Die sind geschrieben, aber weder kompiliert
noch auf einem Gerät gelaufen — für PlatformIO fehlte in dieser Umgebung die
Toolchain. Vor dem ersten Einschalten also `docs/01-hardware.md` lesen,
besonders den Abschnitt zur Stromversorgung.

Eine Android-App ist bewusst nicht dabei. `docs/03-protokolle.md`
spezifiziert den Stream vollständig; der Python-Empfänger in
`host/scan3d/stream.py` ist rund 200 Zeilen und lässt sich direkt nach Kotlin
übertragen.

## Weiter

* `docs/01-hardware.md` — Stückliste, Mechanik, Verkabelung, Strombedarf
* `docs/02-geometrie.md` — Mathematik, Abdeckung, Kalibrierung
* `docs/03-protokolle.md` — S2-Protokoll und Streamformat
