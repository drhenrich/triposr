# Übersetzungstest ohne ESP-Toolchain

```bash
make -C firmware/test/compile
```

Baut `main.cpp`, `rplidar.cpp`, `feetech_servo.cpp` und `yaw_axis.cpp` gegen
Attrappen der Arduino- und IDF-APIs (`stubs/`) und **bindet** sie.

## Wozu

Diese vier Dateien standen lange unter „nicht kompiliert" — in der Umgebung,
in der sie entstanden sind, gibt es keine PlatformIO-Toolchain, und die
Netzpolitik lässt `api.registry.platformio.org` nicht durch. Jeder Tippfehler
darin fiel damit erst beim Flashen auf, also beim Menschen mit der Hardware
auf dem Tisch.

Die Attrappen kosten rund 150 Zeilen und schließen diese Lücke zum größten
Teil. Nachgewiesen, indem drei Fehler absichtlich eingebaut wurden:

| eingebauter Fehler | wird gemeldet |
|---|---|
| Tippfehler in einem Konstantennamen | ja |
| falsche Argumentzahl an `writeFaultFrame` | ja |
| fehlende Variablendeklaration | ja |

## Was er *nicht* leistet

Kein Laufzeitverhalten, keine Register, kein Timing, keine
Speicherbeschränkungen des Geräts, keine Eigenheiten der Xtensa-Toolchain.
Er sagt: „der Code ist in sich schlüssig" — nicht: „er tut das Richtige".
Was das Richtige ist, prüft `test/native` an der Logik.

`usb_ncm.cpp` fehlt absichtlich. Es hängt an `esp_tinyusb`, dessen API sich
zwischen IDF-Fassungen mehrfach geändert hat; eine Attrappe davon würde eine
Vertrautheit vortäuschen, die nicht da ist. Die **Aufrufe** aus `main.cpp`
werden trotzdem geprüft, gegen die Deklarationen in `usb_ncm.h`.

## Aufbau

```
stubs/Arduino.h        Serial, delay, millis
stubs/WiFi.h           WiFi, WiFiServer, WiFiClient
stubs/freertos/        Queue, Task, Ticks
stubs/driver/uart.h    UART-Treiber des IDF
stubs/esp_timer.h      esp_timer_get_time
stubs.cpp              Rümpfe dazu, plus main()
```

Die Attrappen bilden **Signaturen** nach, kein Verhalten. Stimmt eine
Signatur nicht mit der echten API überein, fällt das hier nicht auf — dafür
ist der echte Build da. Sie sind eine Vorprüfung, kein Ersatz.
