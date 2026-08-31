// Einstiegspunkt fuer die Arduino-IDE. Absichtlich leer.
//
// Dieses Projekt ist ein PlatformIO-Projekt; der Code liegt in src/, und
// setup() und loop() stehen in src/main.cpp. Die Arduino-IDE braucht aber
// eine .ino-Datei, die genauso heisst wie ihr Ordner - deshalb diese hier.
// Sie uebersetzt zusaetzlich alles unter src/ mit, rekursiv, und findet
// include/config.h ueber die relativen Includes in den Quelldateien.
//
// Damit laesst sich das Projekt in der Arduino-IDE oeffnen:
//
//   1. Ordner `firmware` oeffnen (Datei -> Oeffnen -> firmware.ino).
//   2. Board: "ESP32S3 Dev Module".
//   3. Hochladen.
//
// Zwei Bibliotheken muessen ueber den Bibliotheksverwalter dazu:
//
//   WebSockets    von Markus Sattler (links2004)  - Punkte an den Browser
//   EspUsbHost    von tanakamasayuki              - LiDAR am USB-Host-Port
//
// Was dabei NICHT dabei ist: USB-Ethernet (CDC-NCM) zum iPhone. Das braucht
// `esp_tinyusb` aus ESP-IDF, das die Arduino-IDE nicht mitbringt. config.h
// laesst ENABLE_USB_NCM deshalb standardmaessig aus, usb_ncm.cpp uebersetzt
// dann zu Rumpffunktionen, und es bleibt bei WLAN - was fuer die Webseite
// voellig genuegt. Wer den Kabelweg fuer die Swift-App will, baut mit
// PlatformIO: `pio run -e usb`.
