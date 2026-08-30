# iPhone am USB-C: was geht und was nicht

Kurzfassung: **Ein serielles USB-Gerät kann eine iPhone-App nicht ansprechen.**
Ein USB-*Netzwerk*gerät dagegen schon. Der ESP32-S3 meldet sich deshalb als
USB-Ethernet-Adapter (CDC-NCM) an, und darüber läuft exakt derselbe TCP-Stream
wie über WLAN.

## Warum nicht einfach seriell

Das wäre der naheliegende Weg und er ist versperrt:

* iOS hat **keine API für generische USB-Peripherie**. Serielle Geräte
  (CDC-ACM, FTDI, CP210x) werden vom System nicht an Apps durchgereicht.
* Das `ExternalAccessory`-Framework ist der offizielle Weg für eigene
  Hardware — verlangt aber **MFi-Zertifizierung** samt Apple-Authentifizierungschip
  im Gerät. Für einen Prototyp unrealistisch.
* **DriverKit/USBDriverKit** gibt es auf macOS und iPadOS (ab M1-iPad), aber
  **nicht auf dem iPhone**. Ein Apple-Ingenieur hat das im Developer-Forum
  ausdrücklich bestätigt; der Hinweis „iOS 16.0+" auf der Doku-Seite meint
  iPadOS.

Am USB-C-Anschluss des iPhone 17 Pro ändert das nichts — der Stecker ist neu,
die Softwarerichtlinie nicht.

## Warum Ethernet über USB funktioniert

Seit iOS 17 unterstützen iPhones mit USB-C **USB-Ethernet-Adapter** direkt.
Der Treiber steckt im System, arbeitet über die Geräteklasse **CDC-NCM** und
verlangt kein MFi — die verkauften Adapter sind zertifiziert, weil sich das gut
verkauft, nicht weil die Klasse es erzwingt.

Für uns heißt das: Der ESP32-S3 hat einen nativen USB-Anschluss und kann sich
mit TinyUSB als NCM-Gerät melden. Das iPhone sieht dann schlicht ein
Netzwerkinterface — mit einer IP-Adresse, über die sich der TCP-Server auf
Port 5005 ansprechen lässt.

**Das ist der eigentliche Gewinn: am Protokoll ändert sich nichts.** Der
Frame-Stream aus `03-protokolle.md`, der Swift-Decoder und die 3D-Ansicht sind
identisch, egal ob die Verbindung über WLAN oder über das Kabel kommt. Die App
verbindet sich in beiden Fällen auf eine IP und einen Port.

Nebeneffekt, der für das Kabel spricht: **das iPhone behält sein WLAN und sein
Mobilfunknetz.** Beim WLAN-Ansatz muss sich das Telefon in das Netz des
Scanners einbuchen und ist damit offline.

## Die Falle, die man kennen muss

Auf ESP32-S3 mit NCM gab es lange den Effekt: iOS erkennt das Interface, aber
es bekommt **keine IP-Adresse**. Ursache ist inzwischen bekannt und behoben
(espressif/esp-idf#18079, abgeschlossen Januar 2026):

1. **iOS fragt DHCP genau einmal**, direkt beim Link-Up, und wiederholt es
   nicht. Meldet das NCM-Interface „Link oben", bevor der DHCP-Server auf dem
   ESP32 bereit ist, wartet das iPhone für immer.
   → Der Link darf erst hochgezogen werden, wenn alles andere steht.
2. `esp_tinyusb` hat den Versand an `tud_ready()` gekoppelt, was DHCP-Pakete
   rund um Suspend/Resume verschluckt.
   → Gegen `tud_mounted()` tauschen.

Beides ist in aktuellem ESP-IDF drin. Die Firmware hier hält sich zusätzlich an
Punkt 1 und schaltet den Link erst frei, wenn DHCP-Server und TCP-Server laufen
(`firmware/src/usb_ncm.cpp`).

Bleibt es trotzdem bei „kein IP": in den iPhone-Einstellungen unter
*Ethernet* die IP von Hand auf `192.168.7.2 / 255.255.255.0` setzen. Damit
lässt sich sauber unterscheiden, ob das Netz oder nur DHCP klemmt.

## Stromversorgung

Der wichtigste Punkt am Kabel, und der einzige, an dem man Hardware zerstören
kann.

* Das iPhone liefert an USB-C nur wenige Watt. Der S2 will >2 W plus
  Anlaufstrom, der Schrittmotor 12 V. **Der Scanner muss seine eigene
  Stromversorgung haben**; das Kabel überträgt nur Daten.
* Damit hat man zwei 5-V-Quellen im Spiel — das Netzteil des Scanners und
  VBUS vom iPhone. Die dürfen sich nicht gegenseitig speisen. Wie das auf dem
  konkreten Board zu lösen ist (Entkopplungsdiode, VBUS nur als Erkennungs­signal,
  Jumper auf dem Devkit), steht im Schaltplan des jeweiligen Boards — das bitte
  nachsehen und nicht raten.
* Der ESP32-S3 muss VBUS erkennen können, um überhaupt in den Device-Modus zu
  gehen. VBUS komplett aufzutrennen ist deshalb keine Lösung.

## In der App nicht vergessen

`NSLocalNetworkUsageDescription` in die `Info.plist`. Ohne diesen Eintrag
verweigert iOS 14+ jede Verbindung ins lokale Netz — **stillschweigend**, die
`NWConnection` bleibt einfach hängen. Das kostet erfahrungsgemäß einen halben
Abend.

```xml
<key>NSLocalNetworkUsageDescription</key>
<string>Verbindung zum LiDAR-Scanner</string>
```

Beim ersten Start fragt iOS einmal nach; wird abgelehnt, hilft nur noch
*Einstellungen → Datenschutz → Lokales Netzwerk*.

## Die Adressen

| Weg | Scanner | iPhone |
|---|---|---|
| USB-C (NCM) | `192.168.7.1:5005` | `192.168.7.2` per DHCP |
| WLAN (AP) | `192.168.4.1:5005` | `192.168.4.2` per DHCP |

Die App probiert beide der Reihe nach, sodass man nichts umstellen muss —
Kabel dran heißt Kabel, sonst WLAN.

## Wenn das Kabel nicht will

Fallback-Reihenfolge, aufsteigend nach Aufwand:

1. **WLAN.** Funktioniert sofort, kostet das iPhone nur seine Internetverbindung.
2. **Statische IP** auf dem iPhone setzen (siehe oben) — trennt Netzwerk- von
   DHCP-Problemen.
3. **USB-Ethernet-Dongle plus echtes Ethernet** am ESP32 (W5500 o. ä.). Mehr
   Hardware, aber die Klasse ist dann garantiert die, für die iOS gebaut wurde.
4. **MFi.** Nur, wenn daraus ein Produkt werden soll.
