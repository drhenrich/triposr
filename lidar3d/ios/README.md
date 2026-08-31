# iOS-App: Echtzeit-3D am iPhone

Zeigt die Punktwolke live, waehrend der Sweep laeuft. Verbindung wahlweise
ueber das USB-C-Kabel oder ueber WLAN — die App probiert beides der Reihe nach
durch, am Protokoll aendert sich nichts.

Warum das Kabel nicht seriell laeuft, sondern als USB-Ethernet: siehe
[`../docs/04-ios-usb.md`](../docs/04-ios-usb.md). Kurz: iOS reicht generische
USB-Geraete nicht an Apps durch, DriverKit gibt es auf dem iPhone nicht, und
MFi ist fuer einen Prototyp keine Option. USB-Ethernet (CDC-NCM) dagegen
unterstuetzt iOS seit 17 mit Bordmitteln.

## Aufbau

```
LidarKit/          Swift Package, ohne UI- und Metal-Bezug
  Sources/LidarKit/
    Frame.swift            Frameprotokoll (Parser, Capsule, Hello, Status)
    Geometry.swift         Kugelkoordinaten, Einbaulage, Sweep-Planung
    PointCloudBuffer.swift threadsichere Sammelstelle Netz -> Renderer
    ScannerClient.swift    TCP ueber Network.framework, USB zuerst, dann WLAN
  Tests/LidarKitTests/     laufen auf dem Mac, ohne Geraet
ScannerApp/        Quellen der App
  ScannerApp.swift         Einstiegspunkt
  ContentView.swift        Oberflaeche, HUD, Kamerasteuerung
  ScannerViewModel.swift   Status und Verbindung fuer SwiftUI
  PointCloudRenderer.swift Metal, haengt Punkte an einen GPU-Puffer an
  PointCloud.metal         Shader, gruener Hoehenverlauf
```

Der Decoder liegt bewusst in einem eigenen Paket: so laesst er sich auf dem
Mac testen, ohne die App zu starten und ohne Hardware.

## Xcode-Projekt erzeugen

Im Repo liegt kein `.xcodeproj`, sondern der Bauplan dazu: `project.yml`. Eine
`project.pbxproj` ist mehrere tausend Zeilen mit generierten Objekt-IDs —
von Hand geschrieben ist sie nicht pruefbar und bricht bei jeder Xcode-Version
anders. Aus dem Bauplan faellt sie reproduzierbar an:

```bash
brew install xcodegen
cd lidar3d/ios && xcodegen
open ScannerApp.xcodeproj
```

Damit sind auch die beiden Einstellungen gesetzt, die man sonst regelmaessig
vergisst: `NSLocalNetworkUsageDescription` (**ohne den Eintrag scheitert jede
Verbindung stillschweigend** — iOS blockiert lokales Netz ohne Fehlermeldung)
und das Deployment Target iOS 17.

In Xcode bleibt einer:

* **Signing & Capabilities → Team** auf die eigene Apple-ID stellen. Meldet
  Xcode den Bundle Identifier als vergeben, in `project.yml` unter
  `PRODUCT_BUNDLE_IDENTIFIER` einen anderen eintragen und `xcodegen` erneut
  laufen lassen.
* Mit einer **kostenlosen** Apple-ID laeuft die App sieben Tage, danach neu
  installieren. Beim ersten Start am iPhone einmal
  *Einstellungen → Allgemein → VPN & Geraeteverwaltung* → dem Entwickler
  vertrauen.

Metal laeuft **nicht im Simulator** — es braucht das Geraet.

<details>
<summary>Ohne XcodeGen, von Hand</summary>

1. Xcode → **File → New → Project → iOS → App**.
   Interface **SwiftUI**, Sprache **Swift**, Name `ScannerApp`.
2. Die von Xcode erzeugte `ContentView.swift` und `…App.swift` loeschen und
   stattdessen die vier Swift-Dateien plus `PointCloud.metal` aus
   `ScannerApp/` per Drag & Drop hinzufuegen (*Copy items if needed* an).
3. **File → Add Package Dependencies → Add Local…** und den Ordner
   `LidarKit` auswaehlen. Danach unter *General → Frameworks, Libraries* das
   Produkt `LidarKit` zum App-Target hinzufuegen.
4. Unter **Info** eintragen: `NSLocalNetworkUsageDescription` =
   `Verbindung zum LiDAR-Scanner`.
5. Deployment Target **iOS 17**.

</details>

## Was die App zeigt, solange kein ESP32 da ist

**Nichts ausser „getrennt".** Die App ist ein Netzwerk-Client: sie sucht den
Scanner unter `192.168.7.1` (USB-C) und `192.168.4.1` (WLAN). Beide Adressen
gehoeren dem ESP32-S3. Haengt der C1 per USB am Mac, gibt es nichts, womit sie
sich verbinden koennte — der Weg dorthin ist bis dahin die Streamlit-App unter
`host/`.

Ob der Decoder stimmt, laesst sich trotzdem jetzt schon beantworten, ohne
iPhone und ohne Scanner: mit `swift test`.

## Tests

```bash
cd ios/LidarKit && swift test
```

Prueft den Swift-Decoder gegen `lidar3d/tests/wire_fixture.txt`, also gegen
dieselben Bytes wie Firmware und Python-Host, dazu die Geometrie und die
Sammelstelle. Diese Tests brauchen weder Scanner noch iPhone und sind der
schnellste Weg, um zu sehen, ob die drei Protokollimplementierungen noch
zusammenpassen.

## Bedienung

Beim Start verbindet sich die App selbst und der Scanner beginnt einen Sweep
(`AUTO_START_ON_CONNECT` in der Firmware). Danach:

* **Ziehen** dreht die Kamera, **Zwei-Finger-Zoom** fuer den Abstand
* **SWEEP** startet einen neuen Durchlauf; die alte Wolke wird verworfen
* **STOP** bricht die Achsbewegung ab
* Im HUD stehen Transportweg, Zustand, Punktzahl, Capsules/s sowie
  Pruefsummenfehler und verworfene Frames

`verworfen > 0` heisst: der Transport kam nicht mit und die Wolke hat Luecken.
Ueber USB-C sollte das nie vorkommen, ueber WLAN je nach Umgebung schon.

## Stand

Die Swift-Quellen sind **nicht kompiliert** — in der Umgebung, in der sie
entstanden sind, gab es keine Swift-Toolchain und kein Xcode. Rechne mit
kleinen Korrekturen beim ersten Bauen. Die Protokoll- und Geometrielogik ist
dafuer 1:1 aus den Implementierungen uebernommen, die getestet sind (Python
und C++), und die mitgelieferten Tests decken genau diese Teile ab: einmal
`swift test` auf dem Mac beantwortet die Frage, ob der Decoder stimmt, bevor
ueberhaupt Hardware im Spiel ist.
