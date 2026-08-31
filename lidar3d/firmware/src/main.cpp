// Standalone-3D-Scanner: RPLIDAR S2 auf einer Gierachse, ESP32-S3.
//
// Ablauf: Client verbindet sich per TCP -> Achse faehrt auf den Endschalter
// -> Sweep ueber 180 Grad mit konstanter Geschwindigkeit -> jede Dense-Capsule
// wird mit dem passenden Gierwinkel versehen und gestreamt -> Achse faehrt
// zurueck. 180 Grad genuegen fuer die volle Kugel, weil der LiDAR in seiner
// Ebene bereits 360 Grad misst.
//
// Zwei Tasks: der LiDAR-Task liest die UART und packt Frames, der Netz-Task
// schiebt sie ins WLAN. Dazwischen eine Queue, damit ein WLAN-Aussetzer den
// LiDAR-Strom nicht anhaelt.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "../include/config.h"
#include "dense_capsule.h"
#include "rplidar.h"
#include "stream_proto.h"
#include "usb_ncm.h"
#include "yaw_axis.h"

using namespace nwl;

struct FrameMsg {
  uint16_t len;
  uint8_t bytes[kMaxFrameSize];
};

static RPLidar g_lidar;
static YawAxis g_axis;
static CapsuleDecoder g_decoder;
static QueueHandle_t g_queue = nullptr;
static WiFiServer g_server(TCP_PORT);

static volatile uint32_t g_capsules = 0;
static volatile uint32_t g_dropped = 0;
static volatile bool g_scanRunning = false;
static uint16_t g_seq = 0;

// Ebenenerfassung: die Achse oeffnet das Fenster, aber gezaehlt wird erst ab
// der ersten Umlaufmarke des LiDAR. Zwischen erster und zweiter Marke liegt
// genau eine Umdrehung - so bekommt jede Ebene exakt 3200 Punkte, unabhaengig
// davon, wo der Kopf beim Anhalten gerade stand.
static bool g_planeRecording = false;
static bool g_lastCaptureActive = false;

// --- LiDAR-Seite ----------------------------------------------------------

static void onCapsule(void *, const DenseCapsule &capsule) {
  CapsuleSpan span;
  if (!g_decoder.push(capsule, span)) return;  // erste Capsule wird gehalten

  bool capturing = g_axis.captureActive();
  if (capturing != g_lastCaptureActive) {
    g_lastCaptureActive = capturing;
    g_planeRecording = false;  // neue Ebene, auf die erste Marke warten
  }

  uint8_t flags = 0;
  if (span.revolutionIndex >= 0) flags |= kFlagNewRevolution;

  if (capturing && span.revolutionIndex >= 0) {
    if (!g_planeRecording) {
      g_planeRecording = true;
    } else {
      // Zweite Marke: die Umdrehung ist voll, weiter zur naechsten Ebene.
      g_planeRecording = false;
      g_axis.planeCaptured();
    }
  }
  if (capturing && g_planeRecording) flags |= kFlagSweepActive;

  // Die Achse steht still, waehrend gemessen wird - Anfangs- und Endwinkel
  // der Capsule sind derselbe gemessene Wert. Deshalb braucht es hier keine
  // Interpolation ueber Zeitstempel mehr.
  uint32_t yaw = static_cast<uint32_t>(g_axis.planeYawQ16());

  FrameMsg msg;
  msg.len = static_cast<uint16_t>(
      writeCapsuleFrame(msg.bytes, g_seq++, flags, span, yaw, yaw));

  ++g_capsules;
  if (xQueueSend(g_queue, &msg, 0) != pdTRUE) {
    // Queue voll: lieber die neueste Capsule verwerfen als den LiDAR bremsen.
    ++g_dropped;
  }
}

// --- LiDAR-Seite, einfacher Scanmodus (C1) --------------------------------
//
// Dieselbe Ebenenlogik wie oben, nur kommt die Umlaufmarke hier je Messung
// statt je Capsule. Gesammelt wird in Gruppen: ein Frame endet spaetestens
// nach kScanMaxSamples und immer an einer Umlaufmarke - so gehoert ein Frame
// nie zu zwei Umdrehungen.
static ScanSample g_scanBuffer[kScanMaxSamples];
static int g_scanFill = 0;
static uint8_t g_scanFlags = 0;

static void flushScanFrame() {
  if (g_scanFill == 0) return;
  uint32_t yaw = static_cast<uint32_t>(g_axis.planeYawQ16());

  FrameMsg msg;
  msg.len = static_cast<uint16_t>(writeScanFrame(
      msg.bytes, g_seq++, g_scanFlags, g_scanBuffer, g_scanFill, yaw, yaw));
  g_scanFill = 0;
  g_scanFlags = 0;

  ++g_capsules;
  if (xQueueSend(g_queue, &msg, 0) != pdTRUE) {
    // Queue voll: lieber die neuesten Messungen verwerfen als den LiDAR bremsen.
    ++g_dropped;
  }
}

static void onScanSample(void *, const ScanSample &sample) {
  bool capturing = g_axis.captureActive();
  if (capturing != g_lastCaptureActive) {
    g_lastCaptureActive = capturing;
    g_planeRecording = false;  // neue Ebene, auf die erste Marke warten
  }

  if (sample.newRevolution) {
    // Die Marke gehoert zur naechsten Umdrehung, also erst abschliessen.
    flushScanFrame();
    if (capturing) {
      if (!g_planeRecording) {
        g_planeRecording = true;
      } else {
        // Zweite Marke: die Umdrehung ist voll, weiter zur naechsten Ebene.
        g_planeRecording = false;
        g_axis.planeCaptured();
      }
    }
    g_scanFlags |= kFlagNewRevolution;
  }
  if (capturing && g_planeRecording) g_scanFlags |= kFlagSweepActive;

  g_scanBuffer[g_scanFill++] = sample;
  if (g_scanFill == kScanMaxSamples) flushScanFrame();
}

static void lidarTask(void *) {
  for (;;) {
    if (g_scanRunning) {
      if (g_lidar.usesStandardScan()) {
        g_lidar.pollScan(onScanSample, nullptr, 20);
      } else {
        g_lidar.poll(onCapsule, nullptr, 20);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    g_axis.update();
  }
}

// --- Netz-Seite -----------------------------------------------------------

static void sendHello(WiFiClient &client) {
  uint8_t buf[kMaxFrameSize];
  size_t n = writeHelloFrame(buf, g_seq++, FW_VERSION, LIDAR_RPM,
                             MOUNT_OFFSET_RADIAL_UM, MOUNT_OFFSET_AXIAL_UM,
                             static_cast<uint32_t>(degToQ16(YAW_MIN_DEG)),
                             static_cast<uint32_t>(degToQ16(YAW_MAX_DEG)));
  client.write(buf, n);
}

static void sendStatus(WiFiClient &client) {
  uint8_t buf[kMaxFrameSize];
  size_t n = writeStatusFrame(buf, g_seq++, g_axis.sweepIndex(), g_axis.state(),
                              static_cast<uint32_t>(g_axis.yawQ16Now()), g_capsules,
                              g_lidar.checksumErrors(), g_dropped);
  client.write(buf, n);
}

static void netTask(void *) {
  uint8_t batch[16 * kMaxFrameSize];

  for (;;) {
    WiFiClient client = g_server.available();
    if (!client) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    client.setNoDelay(true);
    Serial.printf("Client verbunden: %s\n", client.remoteIP().toString().c_str());

    xQueueReset(g_queue);
    g_capsules = 0;
    g_dropped = 0;
    sendHello(client);
    sendStatus(client);

#if AUTO_START_ON_CONNECT
    g_axis.startSweep();
#endif
    SweepState lastState = g_axis.state();
    uint32_t lastStatusMs = millis();

    while (client.connected()) {
      // Kommandos: 'S' startet einen Sweep, 'X' bricht ab.
      while (client.available()) {
        int c = client.read();
        if (c == 'S') {
          g_axis.startSweep();
        } else if (c == 'X') {
          g_axis.stop();
        }
      }

      // Sweep fertig (inklusive Rueckfahrt) -> Idle sofort melden, damit der
      // Host die Wolke abschliessen kann.
      if (lastState != kStateIdle && g_axis.state() == kStateIdle) {
        sendStatus(client);
      }
      lastState = g_axis.state();

      // Frames buendeln: 800 Einzelschreibvorgaenge/s waeren Verschwendung.
      size_t used = 0;
      FrameMsg msg;
      while (used + kMaxFrameSize <= sizeof(batch) &&
             xQueueReceive(g_queue, &msg, 0) == pdTRUE) {
        memcpy(batch + used, msg.bytes, msg.len);
        used += msg.len;
      }
      if (used > 0) {
        if (client.write(batch, used) != static_cast<int>(used)) break;
      } else {
        vTaskDelay(pdMS_TO_TICKS(2));
      }

      if (millis() - lastStatusMs >= 200) {
        lastStatusMs = millis();
        sendStatus(client);
      }
    }

    Serial.println("Client getrennt");
    g_axis.stop();
    client.stop();
  }
}

// --- Aufbau ---------------------------------------------------------------

static void startWifi() {
#if !ENABLE_WIFI
  Serial.println("WLAN deaktiviert");
#elif WIFI_AP_MODE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("AP %s, IP %s\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("verbinde mit WLAN");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
  }
  Serial.printf("\nIP %s\n", WiFi.localIP().toString().c_str());
#endif
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nlidar3d - RPLIDAR S2 auf Gierachse");

  if (!g_axis.begin()) {
    // Ohne Servo waeren alle Gierwinkel gelogen - lieber gar nicht scannen.
    Serial.println("FEHLER: STS3215 antwortet nicht. Bus-ID, Baudrate (1 Mbaud), "
                   "Halbduplex-Verdrahtung und 12-V-Versorgung pruefen.");
    while (true) delay(1000);
  }
  Serial.printf("Gierachse: Servo-Modell %u, %u Ebenen a %.2f deg, "
                "Encoderaufloesung %.4f deg\n",
                g_axis.servoModel(), g_axis.planeCount(), YAW_PLANE_STEP_DEG,
                360.0 / SERVO_COUNTS_PER_REV);

  if (!g_lidar.begin(static_cast<uart_port_t>(LIDAR_UART_NUM), LIDAR_RX_PIN,
                     LIDAR_TX_PIN, LIDAR_BAUDRATE, LIDAR_RX_BUFFER)) {
    Serial.println("FEHLER: LiDAR-UART liess sich nicht oeffnen");
    while (true) delay(1000);
  }
  g_lidar.setMotorRpm(LIDAR_RPM);
  delay(1500);  // Anlauf des LiDAR-Motors abwarten

  // Erst den einfachen Scanmodus versuchen - der C1 kann nur den, und beim S2
  // scheitert er erst an der Bandbreite. Nur wenn er sich nicht starten
  // laesst, auf die Dense-Capsules ausweichen.
  if (g_lidar.startStandardScan()) {
    Serial.println("LiDAR laeuft im einfachen Scanmodus (C1)");
  } else {
    int mode = g_lidar.startDenseScan();
    if (mode < 0) {
      Serial.println("FEHLER: Kein Scanmodus liess sich starten. Stromversorgung "
                     "(5 V, >2 W) und Baudrate pruefen (C1: 460800, S2: 1 Mbaud).");
      while (true) delay(1000);
    }
    Serial.printf("LiDAR laeuft mit Dense-Capsules, Scanmodus %d\n", mode);
  }
  g_scanRunning = true;

  g_queue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(FrameMsg));
  if (g_queue == nullptr) {
    Serial.println("FEHLER: Queue liess sich nicht anlegen");
    while (true) delay(1000);
  }

  startWifi();

#if ENABLE_USB_NCM
  // Interface anlegen, aber Link noch unten lassen: iOS fragt DHCP genau
  // einmal beim Link-Up und wiederholt es nie. Erst wenn der TCP-Server
  // lauscht, wird freigegeben.
  bool ncmReady = usbNcmStart();
  if (!ncmReady) Serial.println("WARNUNG: USB-NCM liess sich nicht starten");
#endif

  g_server.begin();

#if ENABLE_USB_NCM
  if (ncmReady) {
    usbNcmSetLinkUp(true);
    Serial.println("USB-C bereit: Scanner unter 192.168.7.1:5005");
  }
#endif

  // LiDAR auf Core 1, Netz auf Core 0 (dort laeuft auch der WLAN-Stack).
  xTaskCreatePinnedToCore(lidarTask, "lidar", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(netTask, "net", 6144, nullptr, 4, nullptr, 0);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
