// Zentrale Konfiguration. Pins auf das eigene Board anpassen.
#pragma once

#include <stdint.h>

// --- Firmware -------------------------------------------------------------
#define FW_VERSION 1

// --- RPLIDAR S2 -----------------------------------------------------------
// Der S2 spricht 3.3-V-TTL-UART mit 1 Mbaud, der ESP32-S3 auch: kein
// Pegelwandler noetig. Versorgung 5 V, >2 W - nicht aus dem 3.3-V-Regler.
#define LIDAR_UART_NUM 1
#define LIDAR_RX_PIN 18  // ESP32 empfaengt, geht an TX des LiDAR
#define LIDAR_TX_PIN 17  // ESP32 sendet, geht an RX des LiDAR
#define LIDAR_BAUDRATE 1000000
#define LIDAR_RX_BUFFER 8192
// 8N1 bei 1 Mbaud: 10 Bit je Byte -> 10 us. Fuer die Zeitstempel.
#define LIDAR_BYTE_TIME_NS 10000
// 10 Hz Scanrate = 600 rpm. Erlaubt sind laut Datenblatt 8..15 Hz.
#define LIDAR_RPM 600

// --- Gierachse: Feetech STS3215 Busservo ---------------------------------
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
// Abstand zwischen zwei Scanebenen. 1 Grad ergibt 180 Ebenen a 3200 Punkte.
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
// Die Firmware rechnet damit nicht, sie meldet die Werte nur im Hello-Frame.
#define MOUNT_OFFSET_RADIAL_UM 0
#define MOUNT_OFFSET_AXIAL_UM 0

// --- Netzwerk -------------------------------------------------------------
// Beide Transporte koennen gleichzeitig laufen; der TCP-Server lauscht auf
// allen Interfaces, und die App probiert Kabel zuerst, dann WLAN.
//
// USB-C: der ESP32-S3 meldet sich am iPhone als USB-Ethernet (CDC-NCM), weil
// iOS generische USB-Serial-Geraete nicht an Apps durchreicht. Siehe
// docs/04-ios-usb.md. Adresse des Scanners: 192.168.7.1
#define ENABLE_USB_NCM 1
// WLAN als Rueckfallweg. Kostet das iPhone seine Internetverbindung.
#define ENABLE_WIFI 1

// WIFI_AP_MODE 1: der Scanner spannt ein eigenes Netz auf (Handy verbindet
// sich damit). 0: er haengt sich in ein vorhandenes WLAN.
#define WIFI_AP_MODE 1
#define WIFI_SSID "lidar3d"
#define WIFI_PASSWORD "scanmemaybe"
#define TCP_PORT 5005
// 1: sobald sich ein Client verbindet, faehrt die Achse auf den Endschalter
// und startet einen Sweep. 0: der Client muss 'S' senden.
#define AUTO_START_ON_CONNECT 1

// Ringpuffer zwischen LiDAR-Task und Netz-Task. 256 Frames a 104 Byte sind
// rund 27 kB und puffern etwa 320 ms - genug fuer WLAN-Aussetzer.
#define FRAME_QUEUE_LENGTH 256
