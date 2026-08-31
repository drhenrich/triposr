# Geometrie, Abdeckung, Kalibrierung

## Die Rechnung

Steht die Scanebene senkrecht und enthält sie die Drehachse, dann ist eine
Messung schlicht eine Kugelkoordinate:

* `α` — Polarwinkel, kommt vom LiDAR (0° = entlang der Drehachse nach oben)
* `ψ` — Azimut, kommt vom Schrittzähler
* `r` — Distanz

```
x = r · sin α · cos ψ
y = r · sin α · sin ψ
z = r · cos α
```

Mehr ist es nicht. Kein SLAM, keine Registrierung, keine Pose-Schätzung — der
Aufbau steht still, und beide Winkel sind direkt gemessen.

Sitzt das optische Zentrum nicht exakt auf der Drehachse, gehen die beiden
Versätze **vor** der Drehung ein:

```
u = d_radial + r · sin α        (radial von der Achse weg)
w = d_axial  + r · cos α        (entlang der Achse)

x = u · cos ψ
y = u · sin ψ
z = w
```

Implementiert in `host/scan3d/geometry.py:to_cartesian`.

## Warum 180° Gieren genügen

`u` darf negativ werden. Eine Messung mit α = 270° zeigt in dieselbe Richtung
wie eine Messung mit α = 90° bei einem um 180° gedrehten Kopf. Formal:

```
(r, α = 270°, ψ = 0°)  ≡  (r, α = 90°, ψ = 180°)
```

Der Test `test_180_degree_yaw_covers_the_back_half` prüft genau diese
Identität. Anders gesagt: **die hintere Hälfte der Scanebene erledigt bereits
die zweite Hälfte des Gierbereichs.** Ein Sweep von 0° bis 180° tastet die
volle Kugel ab; 360° zu fahren würde jeden Punkt nur ein zweites Mal messen.

Praktische Folge: eine Kabelschlaufe statt eines Schleifrings.

## Warum die Drehachse senkrecht steht (Z, nicht X)

Rechnerisch ist die Frage entschieden, bevor sie gestellt wird: 180° um **jede**
Achse decken die volle Kugel ab, solange die Scanebene die Achse enthält. Die
Formeln sind identisch, nur anders beschriftet. Die Wahl fällt also allein an
der Mechanik — und dort ist sie eindeutig.

**Die Schwerkraft entscheidet.** Steht die Achse senkrecht, wirkt das Gewicht
des LiDAR (110 g) *entlang* der Welle. Der Servo trägt es axial, das Halten
einer Position kostet ihn kein Drehmoment. Läge die Achse waagerecht, hinge der
Kopf seitlich heraus: das Lastmoment wechselt mit jedem Schritt, ist bei
waagerecht ausgestrecktem Kopf maximal, und der Servo müsste dauerhaft
dagegenhalten — Strom, Wärme, und eine Positionsabweichung, die mit dem Winkel
schwankt. Bei einer Aufnahme mit 180 Ebenen à 150 ms sind das 27 s unter Last
statt 27 s im Leerlauf.

**Die Verdeckung fällt dorthin, wo nichts ist.** Bei senkrechter Achse liegen
Stativ und Servo *unter* dem LiDAR und verdecken den Kegel nach unten — den
Boden direkt unter dem Gerät, also die uninteressanteste Stelle im Raum. Bei
waagerechter Achse säße der Servo *neben* dem Kopf und schnitte ein Stück Wand
heraus.

**Die Auswuchtung ist einfacher.** Auf einer senkrechten Welle lässt sich das
optische Zentrum auf die Achse legen, `offset_radial` wird null, und die
Parallaxenkorrektur entfällt. Um eine waagerechte Welle bräuchte es eine Wiege,
sonst ist der Versatz konstruktiv da.

**Die Punktdichte gibt nichts her.** Sie ist an den Polen hoch und am Äquator
niedrig (siehe unten) — bei senkrechter Achse also dicht an Decke und Boden,
dünn an den Wänden. Das klingt nach einem Argument für die waagerechte Achse,
ist aber keins: dort lägen die Pole auf zwei gegenüberliegenden Wänden, und der
dünnste Bereich enthielte dann Decke, Boden *und* die beiden anderen Wände. Die
schlechteste Auflösung bleibt in beiden Fällen gleich, sie wandert nur.

**Und die Anzeige rechnet mit Z.** Höhenfärbung, „Von oben", die Konturprüfung
in `host/scan3d/quality.py` und die Schätzung der Nulllage in `alignment.py`
setzen alle voraus, dass die Drehachse die Hochachse ist. Eine waagerechte
Achse hieße nicht nur andere Mechanik, sondern auch: alle diese Werkzeuge
messen etwas anderes als das, wonach sie benannt sind.

## Punktdichte

Die Dichte ist stark ungleichmäßig — dicht an den Polen der Drehachse, dünn am
Äquator. Der Winkelabstand zwischen benachbarten Scanebenen skaliert mit
`sin α`:

```
Δ_quer(α) = r · sin α · Δψ
```

Bei 1° Ebenenabstand und 5 m Entfernung sind das am Äquator 87 mm zwischen den
Ebenen, aber nur 0,1125° × 5 m = 10 mm *innerhalb* einer Ebene. Die Wolke ist
also in einer Richtung fast neunmal feiner als in der anderen.

Das ist die **Sanduhr- bzw. Fliegen-Form**, die in der Vorlage auf dem Display
zu sehen ist: an den Polen sammeln sich alle Ebenen, dort steht ein dichter
Kegel, dazwischen wird es dünn.

Zwei Gegenmittel:

* `--voxel 0.02` beim Export. Ein Punkt je 2-cm-Voxel vereinheitlicht die
  Dichte und schrumpft die Datei erheblich.
* Feinerer Ebenenabstand, wenn der Äquator wichtig ist — kostet linear Zeit
  (`python3 -m scan3d plan --step 0.5`).

## Wie die Messung zu ihrem Gierwinkel kommt

Gar nicht über die Zeit — die Achse **steht**, während gemessen wird.

Der Ablauf je Scanebene (`firmware/src/yaw_axis.h`):

1. Servo auf die nächste Ebene fahren und ankommen lassen.
2. 40 ms einrasten lassen, damit Getriebe und Lageregelung zur Ruhe kommen.
3. **Istposition vom Absolutencoder lesen.** Dieser gemessene Wert ist der
   Gierwinkel der Ebene — nicht der befohlene. Getriebespiel und bleibende
   Regelabweichung stehen damit in den Daten statt im Fehler.
4. Genau eine LiDAR-Umdrehung erfassen. Erkannt wird sie an den Umlaufmarken
   des Dekoders: zwischen der ersten und der zweiten Marke liegt exakt eine
   Umdrehung, unabhängig davon, wo der Kopf beim Anhalten gerade stand. So
   bekommt jede Ebene genau 3200 Punkte.
5. Weiter zur nächsten Ebene.

Alle 3200 Messungen einer Ebene tragen denselben Gierwinkel; im Frameprotokoll
sind `yaw_start` und `yaw_end` deshalb gleich.

**Das ist der Grund, warum es hier keine Zeitsynchronisation gibt.** Bei
Rotationsscannern mit Dauerfahrt ist die Zuordnung von Messzeitpunkt zu
Drehwinkel die Hauptfehlerquelle: Latenz, Jitter, verlorene Schritte,
Geschwindigkeitsschwankungen. Bei Schritt und Halt existiert die Frage nicht.

Der Preis: ein Sweep dauert rund 27 statt 18 Sekunden
(`python3 -m scan3d plan`).

Was stattdessen zählt:

| Fehlerquelle | Größenordnung |
|---|---|
| Encoderauflösung | 0,088° (4096 Zählwerte auf 360°) |
| Getriebespiel | zu messen, siehe `01-hardware.md` |
| Restschwingung nach dem Anfahren | von der Einrastzeit abgedeckt |

Die ersten beiden liegen deutlich unter dem Ebenenabstand von 1°.

## Kalibrierung

Fünf Parameter. Alle werden beim Export übergeben oder in `config.h`
eingetragen.

### 1. `alpha_zero` und `alpha_sign` — wo ist oben?

**Das ist der Wert, der am häufigsten schiefgeht — und der auffälligste Fehler
überhaupt.** Der LiDAR zählt seine Winkel ab einer Marke am Gehäuse. Liegt das
Gerät flach, zeigt sie waagerecht nach vorn. Stellt man es für den 3D-Scan
hochkant, zeigt sie zur Seite, nicht nach oben — beim C1 sind deshalb **90°
(oder 270°) richtig, nicht 0°**.

Steht hier fälschlich 0, wird der Abstand zur Decke als Radius verrechnet. Um
die Achse gedreht wird daraus ein Zylinder mit genau diesem Radius: die Wolke
sieht aus wie ein **runder Raum**, obwohl der Aufbau völlig richtig steht. Die
Raumtiefe landet dabei in der Höhe, die Wolke wird also **höher als breit** —
das ist das schnellste Erkennungsmerkmal.

Nicht verwechseln mit dem anderen Rundraum: bei falscher *Einbaulage* (Gerät
liegt flach, Scanebene enthält die Drehachse nicht) misst jede Ebene denselben
waagerechten Ring. Da hilft nur Umbauen, kein Parameter.

Schätzen statt messen: `scan3d.alignment.estimate_alpha_zero` sucht den Winkel,
bei dem möglichst viele Punkte auf genau dieselbe Höhe fallen — also den, bei
dem Decke und Boden waagerecht werden. In der Streamlit-App bietet der Befund
den gefundenen Wert direkt zum Übernehmen an und rechnet die Wolke aus den
Rohzeilen neu; für eine fertige Datei:

```
python tools/make_viewer.py scan.ply -o scan.html --auto-alpha0
```

Bleibt eine Mehrdeutigkeit, die in den Messwerten nicht steckt: `alpha_zero` und
`alpha_zero + 180°` erzeugen dieselbe Wolke, nur auf dem Kopf. Welche der beiden
Flächen Decke und welche Boden ist, muss der Mensch sagen — oder `alpha_sign -1`
setzen, wenn die Wolke gespiegelt ist.

Von Hand geht es auch: eine einzelne Scanebene aufnehmen (`--yaw-rate 0`) und den
LiDAR-Winkel des tiefsten Punktes ablesen. Dieser Winkel minus 180° ist
`alpha_zero`.

### 2. `offset_radial` — der wichtigste Wert

Ein falscher radialer Versatz **krümmt ebene Wände**. Das ist die auffälligste
Verzerrung überhaupt und lohnt die Mühe.

Verfahren:

1. Scanner mit ~3 m Abstand vor eine glatte, ebene Wand stellen.
2. Sweep aufnehmen und mit `--offset-radial 0` exportieren.
3. In CloudCompare die Wand von oben betrachten. Wölbt sie sich zum Scanner
   hin, ist der Versatz positiv; wölbt sie sich weg, negativ.
4. Den Wert in 5-mm-Schritten variieren und neu exportieren, bis die Wand
   gerade ist. Die Rohdaten müssen dafür nicht neu aufgenommen werden — der
   Versatz geht erst beim Export ein.

Der Startwert lässt sich messen: Abstand von der Drehachse zur Mitte der
optischen Baugruppe des S2, meist einige Zentimeter.

Der Test `test_uncalibrated_radial_offset_bends_a_flat_wall` zeigt die
Größenordnung: 40 mm Versatz verschieben eine 3 m entfernte Wand um bis zu
40 mm, und zwar richtungsabhängig — daher die Krümmung.

### 3. `offset_axial` — die Höhe

Reine Verschiebung entlang Z, verzerrt nichts. Nur nötig, wenn der Ursprung
der Punktwolke einer physischen Referenz entsprechen soll (Stativkopf,
Bodenplatte). Mit dem Messschieber abnehmen und eintragen.

### 4. `yaw_zero` — die Nordrichtung

Verzerrt ebenfalls nichts, dreht nur die ganze Wolke. Relevant, wenn mehrere
Sweeps zusammenpassen sollen. Beim STS3215 definiert der Absolutencoder
diesen Nullpunkt von selbst und reproduzierbar ueber Neustarts hinweg - eine
mechanische Referenz braucht es nicht.

### 5. Encoderskalierung

Zu prüfen, wenn die Wolke *entlang der Drehung* gestaucht oder gedehnt wirkt:
ein rechtwinkliger Raum erscheint dann keilförmig, oder die letzte Scanebene
trifft nicht auf die erste. Ursache wäre ein falscher
`SERVO_COUNTS_PER_REV` in `config.h` — beim STS3215 sind es 4096.

Gegenprobe ohne Rechnerei: einen Sweep über volle 180° in einem Raum mit zwei
gegenüberliegenden parallelen Wänden aufnehmen. Stehen die beiden Wände in der
Punktwolke parallel, stimmt die Skalierung.

Anders als beim Schrittmotor kann die Achse hier nicht „danebenliegen", ohne
dass es auffällt: der Winkel wird gemessen, nicht hochgerechnet. Ein
systematischer Fehler kann nur noch aus einer falschen Zählwert-Konstante
kommen.

## Reihenfolge

`alpha_sign` → `alpha_zero` → `offset_radial` → Encoderskalierung →
`offset_axial` → `yaw_zero`. Die ersten drei bestimmen die Form, die letzten
drei nur die Lage.
