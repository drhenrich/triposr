// Zentrale Konfiguration. Pins auf das eigene Board anpassen.
#pragma once

#include <stdint.h>

// --- Firmware -------------------------------------------------------------
#define FW_VERSION 1

// --- RPLIDAR C1 -----------------------------------------------------------
// Anschlussweg: der C1 haengt mit seinem USB-C-Adapter am USB-Host-Port des
// ESP32-S3 (LIDAR_LINK_USB 1, der Normalfall). Der Adapter ist ein
// CDC-Seriell-Wandler; der Bytestrom ist derselbe wie an der UART.
//
// Auf 0 stellen, wenn der C1 stattdessen direkt an den TTL-Pins haengt - dann
// gelten die LIDAR_*_PIN-Werte unten. Der C1 spricht 3.3-V-TTL mit 460800
// Baud, der ESP32-S3 auch: kein Pegelwandler noetig. Versorgung in beiden
// Faellen 5 V, nicht aus dem 3.3-V-Regler.
#ifndef LIDAR_LINK_USB
#define LIDAR_LINK_USB 1
#endif
// So lange wartet der Hochlauf darauf, dass sich der LiDAR am USB anmeldet.
#define LIDAR_USB_WAIT_MS 3000
//
// Betriebsart: die Firmware versucht zuerst den einfachen Scanmodus. Beim C1
// reichen dafuer 5000 Messungen/s a 5 Byte, also 25 kB/s von 46 kB/s. Nur
// wenn er sich nicht starten laesst, weicht sie auf die Dense-Capsules aus,
// die der S2 bei 32000 Messungen/s zwingend braucht. Siehe rplidar.h.
#define LIDAR_UART_NUM 1
#define LIDAR_RX_PIN 18  // ESP32 empfaengt, geht an TX des LiDAR
#define LIDAR_TX_PIN 17  // ESP32 sendet, geht an RX des LiDAR
#define LIDAR_BAUDRATE 460800
#define LIDAR_RX_BUFFER 8192
// 8N1: 10 Bit je Byte. Bei 460800 Baud sind das 21701 ns.
#define LIDAR_BYTE_TIME_NS 21701
// 10 Hz Scanrate = 600 rpm. Erlaubt sind laut Datenblatt 8..15 Hz.
#define LIDAR_RPM 600

// --- Gierachse: Feetech STS3215 Busservo ---------------------------------
// Gemeint ist die Ausfuehrung "12 V, 30 kg.cm, magnetische Kodierung"
// (C018, Untersetzung 1:345, Leerlauf 0.222 s/60 Grad = 270 Grad/s).
//
// Es gibt Geschwister mit 7.4 V und anderer Untersetzung (C001 1:345,
// C044 1:191, C046 1:147). Bei denen bleibt SERVO_COUNTS_PER_REV gleich -
// der Encoder sitzt auf der Abtriebswelle und zaehlt immer 4096 je Umdrehung -
// aber SERVO_MOVE_SPEED und SERVO_RETURN_SPEED muessen angepasst werden,
// weil die erreichbare Drehzahl sich aendert.
//
// Werkseinstellung ab Werk: ID 1, 1 MBaud. Beides unten so eingetragen.
//
// Halbduplex-TTL-Bus. Liegt SERVO_DIR_PIN >= 0, schaltet der ESP32 die
// Richtung selbst (RS485-Halbduplexmodus, blendet das eigene Echo aus);
// auf -1 setzen, wenn ein Adapterboard wie das FE-URT-1 das uebernimmt.
#define SERVO_UART_NUM 2
#define SERVO_RX_PIN 16
#define SERVO_TX_PIN 15
#define SERVO_DIR_PIN 7
#define SERVO_BAUDRATE 1000000
#define SERVO_ID 1

// 12-Bit-Absolutencoder: 4096 Zaehlwerte auf 360 Grad = 0.0879 Grad.
#define SERVO_COUNTS_PER_REV 4096
// Fahrgeschwindigkeit in Zaehlwerten/s. 1000 entspricht rund 88 Grad/s;
// die Leerlaufdrehzahl liegt bei etwa 3070 (270 Grad/s bei 12 V).
#define SERVO_MOVE_SPEED 1000
#define SERVO_RETURN_SPEED 2000
// Beschleunigung in 100 Zaehlwerten/s^2.
#define SERVO_ACCELERATION 50
// Ankunft gilt ab dieser Abweichung als erreicht (3 Zaehlwerte = 0.26 Grad).
#define SERVO_ARRIVE_TOLERANCE_COUNTS 3
#define SERVO_POLL_INTERVAL_MS 5
#define SERVO_MOVE_TIMEOUT_MS 4000

// --- Sweep: Schritt und Halt ---------------------------------------------
// 180 Grad genuegen fuer die volle Kugel, weil der LiDAR in seiner Ebene
// bereits 360 Grad misst. Der Bereich ist halboffen - bei 0 und bei 180 Grad
// waere es dieselbe Ebene. Damit entfaellt auch der Schleifring.
#define YAW_MIN_DEG 0.0
#define YAW_MAX_DEG 180.0
// Abstand zwischen zwei Scanebenen. 1 Grad ergibt beim C1 180 Ebenen
// a rund 500 Punkte, zusammen etwa 90000 Punkte.
#define YAW_PLANE_STEP_DEG 1.0

// Wartezeit nach der Ankunft, bevor gemessen wird: Getriebe und Regelung
// muessen zur Ruhe kommen.
#define PLANE_SETTLE_MS 40
// Notbremse, falls die Umlaufmarken des LiDAR ausbleiben. Eine Umdrehung
// dauert bei 10 Hz 100 ms; im ungueltigen Fall wird bis zu einer Umdrehung
// gewartet, bevor es weitergeht.
#define PLANE_CAPTURE_TIMEOUT_MS 400

// --- Einbaulage -----------------------------------------------------------
// Abstand des optischen Zentrums von der Gierachse (radial) und seine Hoehe
// (axial), in Mikrometern. Messen und eintragen - siehe docs/02-geometrie.md.
#define MOUNT_OFFSET_RADIAL_UM 0
#define MOUNT_OFFSET_AXIAL_UM 0

// Der LiDAR-Winkel, der nach oben zeigt. DER WICHTIGSTE WERT HIER.
//
// Der C1 zaehlt seine Winkel ab einer Marke am Gehaeuse. Liegt er flach,
// zeigt die waagerecht nach vorn; hochkant montiert zeigt sie zur Seite -
// also rund 90 Grad versetzt. Stuende hier 0, wuerde der Abstand zur Decke
// als Radius verrechnet und beim Drehen zu einem Zylinder verschmiert: der
// Raum saehe rund aus, und die Wolke waere hoeher als breit.
//
// 89 statt 90: an einer echten Aufnahme gemessen, ueber die Hoehe, bei der
// Decke und Boden waagerecht werden (host/scan3d/alignment.py). Der eigene
// Aufbau kann ein paar Grad daneben liegen - nachmessen lohnt.
#define MOUNT_ALPHA_ZERO_DEG 89.0f
// -1, wenn die Wolke auf dem Kopf steht (Decke unten).
#define MOUNT_ALPHA_SIGN 1.0f

// --- Netzwerk -------------------------------------------------------------
// Beide Transporte koennen gleichzeitig laufen; der TCP-Server lauscht auf
// allen Interfaces, und die App probiert Kabel zuerst, dann WLAN.
//
// USB-C: der ESP32-S3 meldet sich am iPhone als USB-Ethernet (CDC-NCM), weil
// iOS generische USB-Serial-Geraete nicht an Apps durchreicht. Siehe
// docs/04-ios-usb.md. Adresse des Scanners: 192.168.7.1
//
// Standardmaessig AUS, und das mit Absicht: NCM braucht `esp_tinyusb` aus
// ESP-IDF. Wer mit `framework = arduino` allein baut - also `pio run -e wifi`
// oder die Arduino-IDE -, hat das nicht, und der Build scheitert an
// tinyusb.h. Eingeschaltet wird es von `env:usb` ueber -DENABLE_USB_NCM=1.
//
// Die Schalter stehen bewusst in #ifndef: ein blankes #define hier wuerde
// jede Vorgabe von der Kommandozeile stillschweigend ueberschreiben. Genau
// das ist einmal passiert - `env:wifi` setzte 0, config.h machte wieder 1
// daraus, und der Build zog usb_ncm.cpp ohne IDF herein.
#ifndef ENABLE_USB_NCM
#define ENABLE_USB_NCM 0
#endif
// WLAN als Rueckfallweg. Kostet das iPhone seine Internetverbindung.
#ifndef ENABLE_WIFI
#define ENABLE_WIFI 1
#endif

// WIFI_AP_MODE 1: der Scanner spannt ein eigenes Netz auf (Handy verbindet
// sich damit). 0: er haengt sich in ein vorhandenes WLAN.
#define WIFI_AP_MODE 1
#define WIFI_SSID "lidar3d"
#define WIFI_PASSWORD "scanmemaybe"
#define TCP_PORT 5005
// 1: sobald sich ein Client verbindet, faehrt die Achse auf den Endschalter
// und startet einen Sweep. 0: der Client muss 'S' senden.
#define AUTO_START_ON_CONNECT 1

// Ringpuffer zwischen LiDAR-Task und Netz-Task. 256 Frames a 150 Byte sind
// rund 38 kB und puffern etwa 320 ms - genug fuer WLAN-Aussetzer.
#define FRAME_QUEUE_LENGTH 256

// Ringpuffer fuer die Webseite: 16 Buendel a 60 Punkte sind rund 12 kB und
// puffern etwa 190 ms. Laeuft er ueber, gehen Punkte verloren - das ist
// gewollt, der LiDAR soll deswegen nicht warten.
#define WEB_BATCH_QUEUE_LENGTH 16
