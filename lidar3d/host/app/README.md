# Streamlit-Oberfläche

Live-Darstellung des RPLIDAR C1 am USB-Adapter, plus manuelle 3D-Aufnahme.

Das Projekt liegt auf dem Branch `claude/rplidar-s2-robot-q5c9aq` — auf `main`
gibt es den Ordner `lidar3d/` nicht.

Vom Wurzelverzeichnis des Repos aus:

```bash
git checkout claude/rplidar-s2-robot-q5c9aq
pip install -r lidar3d/host/requirements-app.txt
streamlit run lidar3d/host/app/streamlit_app.py
```

Oder aus `lidar3d/host/` heraus:

```bash
pip install -r requirements-app.txt
streamlit run app/streamlit_app.py
```

Beides geht: die App legt ihren eigenen Suchpfad an und ist damit unabhaengig
vom Arbeitsverzeichnis.

Startet mit **simuliertem Raum** — die App läuft also sofort, auch ohne
angeschlossene Hardware. In der Seitenleiste umschalten, sobald der C1 dranhängt.

## Anschließen

| System | Port |
|---|---|
| Linux | `/dev/ttyUSB0` (oder `/dev/ttyACM0`) |
| macOS | `/dev/tty.usbserial-XXXX` |
| Windows | `COM5` |

Baudrate **460800** (C1 ab Werk). Unter Linux muss der Benutzer in der Gruppe
`dialout` sein, sonst bleibt der Port zu:
`sudo usermod -aG dialout $USER` und neu anmelden.

Läuft der Motor nicht an, in der Seitenleiste *Motordrehzahl setzen* auf 600
stellen. Normalerweise startet der C1 von selbst.

## Die drei Ansichten

**Live 2D** — die aktuelle Umdrehung als Schnitt durch den Raum, eingefärbt nach
Entfernung, Ringe alle 2 m. Das ist der schnellste Test, ob Reichweite und
Signalgüte stimmen: die Wände sollten als saubere Linien erscheinen, nicht als
Wolke.

**3D aufnehmen** — Schritt für Schritt, genau der Ablauf, den die Firmware
später automatisch fährt:

1. Achse auf einen Gierwinkel stellen (von Hand oder mit dem Servo)
2. Winkel im Feld eintragen
3. **Ebene aufnehmen** — die aktuelle Umdrehung wandert unter diesem Winkel in
   die Wolke
4. Winkel zählt automatisch weiter, weiter bei 1.

**180° genügen für die volle Kugel**, weil der LiDAR in seiner Ebene bereits
360° misst. Bei 5° Schritten sind das 36 Ebenen à ~500 Punkte.

Ab 200 Punkten erscheint unter der Ansicht ein **Befund**: Er prüft, ob die
Kontur je nach Himmelsrichtung unterschiedlich weit reicht. Tut sie das nicht,
hat die Drehung nichts beigetragen — meist, weil die Scanebene die Drehachse
nicht enthält. Der Umschalter **von oben** zeigt dasselbe mit einem Blick: eine
Raumkontur muss Ecken haben.

Export als **PLY** (öffnet in CloudCompare, MeshLab, Blender) oder als **CSV**
mit den Spalten `Quality, Angle (degrees), Distance (mm), Rotation` — dieselben
Spalten wie das Instructables-Skript, sodass `convertAdjust.py` die Datei direkt
lesen kann.

**Diagnose** — Gerätemodell, Firmware, Health-Status, Verteilung von Signalgüte
und Distanzen.

## Kalibrierung

Der wichtigste Wert ist **Versatz radial**: der Abstand des optischen Zentrums
von der Drehachse. Ist er falsch, krümmen sich ebene Wände in der 3D-Ansicht
sichtbar. Verfahren in [`../../docs/02-geometrie.md`](../../docs/02-geometrie.md);
der Wert lässt sich im Nachhinein ändern, ohne neu aufzunehmen — er geht erst
beim Umrechnen ein.

## Aufbau

Die App wird im Test einmal komplett durchlaufen — gegen eine Attrappe
(`tests/fake_streamlit.py`), die Streamlits Zustandsregeln nachbildet,
insbesondere: *ein Widget-Key darf nicht mehr verändert werden, nachdem das
Widget im selben Durchlauf erzeugt wurde*. Genau daran ist die
Winkel-Weiterschaltung einmal gescheitert; `tests/test_streamlit_app.py` hält
das jetzt fest.

Die Oberfläche ist reine Darstellung. Das Lesen läuft in einem eigenen Thread
(`scan3d/reader.py`), damit Streamlit bei jedem Neulauf des Skripts nicht auf
die serielle Schnittstelle warten muss; der Leser hängt an `st.cache_resource`
und überlebt die Neuläufe. Getestet ist er in `tests/test_reader.py` — gegen den
simulierten Raum und gegen einen toten Port, der als Fehler gemeldet statt
geworfen werden muss.
