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
#include "rplidar_s2.h"
#include "stream_proto.h"
#include "yaw_axis.h"

using namespace nwl;

// Eine Capsule deckt 40 Messungen ab; bei 32000 Messungen/s sind das 1250 us.
static const int64_t kCapsuleWindowUs = 1250;
// Uebertragung der 84 Byte bei 1 Mbaud. Zusammen mit der internen Latenz des
// LiDAR ein konstanter Versatz - er landet in yaw_zero der Kalibrierung.
static const int64_t kTransmitUs = 840;

struct FrameMsg {
  uint16_t len;
  uint8_t bytes[kMaxFrameSize];
};

static RPLidarS2 g_lidar;
static YawAxis g_axis;
static CapsuleDecoder g_decoder;
static QueueHandle_t g_queue = nullptr;
static WiFiServer g_server(TCP_PORT);

static volatile uint32_t g_capsules = 0;
static volatile uint32_t g_dropped = 0;
static volatile bool g_scanRunning = false;
static uint16_t g_seq = 0;

// --- LiDAR-Seite ----------------------------------------------------------

static void onCapsule(void *, const DenseCapsule &capsule) {
  CapsuleSpan span;
  if (!g_decoder.push(capsule, span)) return;  // erste Capsule wird gehalten

  // Messzeitpunkt der ersten Messung dieser Capsule zurueckrechnen.
  int64_t tStart = span.timestampUs - kTransmitUs - kCapsuleWindowUs;

  uint8_t flags = 0;
  if (g_axis.sweepActive()) flags |= kFlagSweepActive;
  if (span.revolutionIndex >= 0) flags |= kFlagNewRevolution;

  FrameMsg msg;
  msg.len = static_cast<uint16_t>(writeCapsuleFrame(
      msg.bytes, g_seq++, flags, span,
      static_cast<uint32_t>(g_axis.yawQ16At(tStart)),
      static_cast<uint32_t>(g_axis.yawQ16At(tStart + kCapsuleWindowUs))));

  ++g_capsules;
  if (xQueueSend(g_queue, &msg, 0) != pdTRUE) {
    // Queue voll: lieber die neueste Capsule verwerfen als den LiDAR bremsen.
    ++g_dropped;
  }
}

static void lidarTask(void *) {
  for (;;) {
    if (g_scanRunning) {
      g_lidar.poll(onCapsule, nullptr, 20);
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
    bool sweepQueued = true;
    g_axis.startHoming();
#else
    bool sweepQueued = false;
#endif
    SweepState lastState = g_axis.state();
    uint32_t lastStatusMs = millis();

    while (client.connected()) {
      // Kommandos: 'S' startet einen Sweep, 'X' bricht ab.
      while (client.available()) {
        int c = client.read();
        if (c == 'S') {
          sweepQueued = true;
          g_axis.startHoming();
        } else if (c == 'X') {
          sweepQueued = false;
          g_axis.stop();
        }
      }

      // Homing fertig -> Sweep starten.
      if (sweepQueued && g_axis.state() == kStateIdle) {
        sweepQueued = false;
        g_axis.startSweep();
      }
      // Sweep fertig -> zuruecksetzen und Idle melden.
      if (lastState == kStateSweeping && g_axis.state() == kStateIdle) {
        sendStatus(client);
        g_axis.startReturn();
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
#if WIFI_AP_MODE
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
    Serial.println("WARNUNG: TMC2209 antwortet nicht ueber UART "
                   "(Verkabelung/Adresse pruefen). Achse laeuft mit Defaults.");
  }
  Serial.printf("Gierachse: %.4f deg je Microstep\n", g_axis.degreesPerStep());

  if (!g_lidar.begin(static_cast<uart_port_t>(LIDAR_UART_NUM), LIDAR_RX_PIN,
                     LIDAR_TX_PIN, LIDAR_BAUDRATE, LIDAR_RX_BUFFER)) {
    Serial.println("FEHLER: LiDAR-UART liess sich nicht oeffnen");
    while (true) delay(1000);
  }
  g_lidar.setMotorRpm(LIDAR_RPM);
  delay(1500);  // Anlauf des LiDAR-Motors abwarten

  int mode = g_lidar.startDenseScan();
  if (mode < 0) {
    Serial.println("FEHLER: Dense-Scan liess sich nicht starten. Stromversorgung "
                   "(5 V, >2 W) und Baudrate (1 Mbaud) pruefen.");
    while (true) delay(1000);
  }
  Serial.printf("LiDAR laeuft, Scanmodus %d\n", mode);
  g_scanRunning = true;

  g_queue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(FrameMsg));
  if (g_queue == nullptr) {
    Serial.println("FEHLER: Queue liess sich nicht anlegen");
    while (true) delay(1000);
  }

  startWifi();
  g_server.begin();

  // LiDAR auf Core 1, Netz auf Core 0 (dort laeuft auch der WLAN-Stack).
  xTaskCreatePinnedToCore(lidarTask, "lidar", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(netTask, "net", 6144, nullptr, 4, nullptr, 0);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
