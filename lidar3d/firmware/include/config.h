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

// --- Gierachse: Schrittmotor ---------------------------------------------
#define STEP_PIN 5
#define DIR_PIN 6
#define ENABLE_PIN 7   // TMC2209 EN ist low-aktiv
#define ENDSTOP_PIN 4  // gegen GND schaltend, interner Pullup
#define ENDSTOP_ACTIVE_LOW 1

// TMC2209 Konfiguration ueber UART
#define TMC_UART_NUM 2
#define TMC_RX_PIN 16
#define TMC_TX_PIN 15
#define TMC_BAUDRATE 115200
#define TMC_ADDRESS 0b00       // MS1/MS2 beide auf GND
#define TMC_RSENSE 0.11f       // typisch fuer BigTreeTech/Fysetc TMC2209
#define TMC_RMS_CURRENT_MA 600 // NEMA17 mit ~1.0 A Nennstrom, leise und kuehl
#define TMC_MICROSTEPS 16

// Mechanik: NEMA17 mit 200 Vollschritten, GT2 20T -> 60T ergibt 3:1.
#define MOTOR_FULL_STEPS 200
#define GEAR_RATIO 3.0

// --- Sweep ----------------------------------------------------------------
// 180 Grad genuegen fuer die volle Kugel, weil der LiDAR in seiner Ebene
// bereits 360 Grad misst. Damit entfaellt der Schleifring.
#define YAW_MIN_DEG 0.0
#define YAW_MAX_DEG 180.0
// 10 deg/s ergibt bei 10 Hz Scanrate 1 Grad zwischen zwei Scanebenen
// und einen Sweep von 18 s.
#define YAW_SWEEP_RATE_DEG_S 10.0
#define YAW_RETURN_RATE_DEG_S 60.0
#define YAW_HOMING_RATE_DEG_S 30.0
// Sicherheitsgrenze: laenger darf keine Fahrt dauern.
#define YAW_MOVE_TIMEOUT_MS 30000

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
