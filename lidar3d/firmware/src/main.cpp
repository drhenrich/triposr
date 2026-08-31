// Standalone-3D-Scanner: RPLIDAR (C1 oder S2) auf einer Gierachse, ESP32-S3.
//
// Ablauf: Client verbindet sich per TCP -> Achse faehrt auf den Endschalter
// -> Sweep ueber 180 Grad, Schritt fuer Schritt -> jede Messgruppe wird mit
// dem passenden Gierwinkel versehen und gestreamt -> Achse faehrt zurueck.
// 180 Grad genuegen fuer die volle Kugel, weil der LiDAR in seiner Ebene
// bereits 360 Grad misst.
//
// Das Netz kommt vor jeder Hardwarepruefung hoch: faellt der Hochlauf aus,
// bleibt der Scanner erreichbar und meldet den Grund als FAULT-Frame, statt
// stumm dazustehen.
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
#include "geometry.h"
#include "rplidar.h"
#include "web_stream.h"
#if LIDAR_LINK_USB
#include "lidar_link_usb.h"
#else
#include "lidar_link_uart.h"
#endif
#include "stream_proto.h"
#include "usb_ncm.h"
#include "yaw_axis.h"

using namespace nwl;

struct FrameMsg {
  uint16_t len;
  uint8_t bytes[kMaxFrameSize];
};

static RPLidar g_lidar;
#if LIDAR_LINK_USB
static UsbLidarLink g_link;
#else
static UartLidarLink g_link;
#endif
static YawAxis g_axis;
static CapsuleDecoder g_decoder;
static QueueHandle_t g_queue = nullptr;
static WiFiServer g_server(TCP_PORT);

static volatile uint32_t g_capsules = 0;
static volatile uint32_t g_dropped = 0;
static volatile bool g_scanRunning = false;
static uint16_t g_seq = 0;

// Was beim Hochlauf fehlgeschlagen ist. Der Scanner laeuft trotzdem weiter,
// bleibt im Netz erreichbar und schickt den Grund an jeden Client.
static uint8_t g_faultCode = kFaultNone;
static char g_faultText[kFaultMaxTextLen + 1] = "";

// Ebenenerfassung: die Achse oeffnet das Fenster, aber gezaehlt wird erst ab
// der ersten Umlaufmarke des LiDAR. Zwischen erster und zweiter Marke liegt
// genau eine Umdrehung - so bekommt jede Ebene exakt 3200 Punkte, unabhaengig
// davon, wo der Kopf beim Anhalten gerade stand.
static bool g_planeRecording = false;
static bool g_lastCaptureActive = false;

// --- Punkte fuer die Webseite ---------------------------------------------
//
// Die Seite bekommt fertige Koordinaten, keine Rohmessungen: sie soll die
// Einbaulage nicht kennen muessen. Gerechnet wird hier, gebuendelt an den
// Netz-Task uebergeben - siehe web_stream.h.
static const MountGeometry kMount = {
    static_cast<float>(MOUNT_OFFSET_RADIAL_UM) / 1000.0f,
    static_cast<float>(MOUNT_OFFSET_AXIAL_UM) / 1000.0f,
    MOUNT_ALPHA_ZERO_DEG, MOUNT_ALPHA_SIGN, 0.0f, 1.0f};
static const RangeFilter kRange;

static Point3 g_webPoints[kWebBatchPoints];
static int g_webFill = 0;

static void flushWebPoints() {
  if (g_webFill == 0) return;
  g_web.pushPoints(g_webPoints, g_webFill);
  g_webFill = 0;
}

static void addWebPoint(float distanceMm, float alphaDeg, float yawDeg) {
  if (!kRange.accepts(distanceMm)) return;
  g_webPoints[g_webFill++] = toCartesian(distanceMm, alphaDeg, yawDeg, kMount);
  if (g_webFill == kWebBatchPoints) flushWebPoints();
}

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
  if (capturing && g_planeRecording) {
    flags |= kFlagSweepActive;
    const float yawDeg = g_axis.planeYawQ16() / 65536.0f;
    float alphaQ16 = static_cast<float>(span.alphaStartQ6) * 1024.0f;
    for (int i = 0; i < kDenseCabinCount; ++i) {
      addWebPoint(static_cast<float>(span.distanceMm[i]),
                  (alphaQ16 / 1024.0f) / 64.0f, yawDeg);
      alphaQ16 += static_cast<float>(span.alphaIncQ16);
    }
  }

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
    // Angefangenes Buendel mitschicken: sonst haengt der letzte Rest einer
    // Ebene, bis die naechste ihn auffuellt, und die Anzeige ruckelt.
    flushWebPoints();
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
  if (capturing && g_planeRecording) {
    g_scanFlags |= kFlagSweepActive;
    addWebPoint(static_cast<float>(sample.distanceMm),
                sample.angleQ6 / 64.0f,
                g_axis.planeYawQ16() / 65536.0f);
  }

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

static void sendFault(WiFiClient &client) {
  uint8_t buf[kMaxFrameSize];
  size_t n = writeFaultFrame(buf, g_seq++, g_faultCode, g_faultText);
  client.write(buf, n);
}

static void sendStatus(WiFiClient &client) {
  uint8_t buf[kMaxFrameSize];
  size_t n = writeStatusFrame(buf, g_seq++, g_axis.sweepIndex(), g_axis.state(),
                              static_cast<uint32_t>(g_axis.yawQ16Now()), g_capsules,
                              g_lidar.checksumErrors(), g_dropped);
  client.write(buf, n);
}

// Kommandos von der Webseite, dieselben wie ueber TCP.
static void onWebCommand(char command) {
  if (command == 'S') {
    // Ohne funktionierende Achse waeren alle Gierwinkel gelogen.
    if (g_faultCode == kFaultNone) g_axis.startSweep();
  } else if (command == 'X') {
    g_axis.stop();
  }
}

static const char *stateLabel() {
  if (g_faultCode != kFaultNone) return "gestoert";
  switch (g_axis.state()) {
    case kStateHoming: return "Anfahrt";
    case kStateSweeping: return "Sweep";
    case kStateReturning: return "Ruecklauf";
    default: return "bereit";
  }
}

// Ein einziger Task fuer beide Wege: Webseite (HTTP + WebSocket) und der
// binaere TCP-Strom fuer die Swift-App.
//
// Der TCP-Teil war frueher eine blockierende Schleife, solange ein Client
// verbunden war. Das ginge jetzt nicht mehr: die Webseite muss zwischendurch
// drankommen, sonst friert sie ein, sobald jemand die App benutzt. Deshalb
// haelt der Task den Client als Zustand und arbeitet ihn Durchlauf fuer
// Durchlauf ab.
static void netTask(void *) {
  static uint8_t batch[16 * kMaxFrameSize];
  WiFiClient client;
  bool greeted = false;
  SweepState lastState = kStateIdle;
  uint32_t lastStatusMs = 0;

  for (;;) {
    g_web.loop();

    if (!client || !client.connected()) {
      if (greeted) {
        Serial.println("Client getrennt");
        client.stop();
        greeted = false;
        // Nur anhalten, wenn kein Browser mehr zusieht - sonst wuerde das
        // Schliessen der App einen laufenden Sweep abwuergen.
        if (!g_web.hasViewer()) g_axis.stop();
      }
      client = g_server.available();
      if (client) {
        client.setNoDelay(true);
        Serial.printf("Client verbunden: %s\n",
                      client.remoteIP().toString().c_str());
        if (g_queue != nullptr) xQueueReset(g_queue);
        sendHello(client);
        if (g_faultCode != kFaultNone) sendFault(client);
        sendStatus(client);
        greeted = true;
#if AUTO_START_ON_CONNECT
        if (g_faultCode == kFaultNone) g_axis.startSweep();
#endif
      }
    }

    if (greeted) {
      while (client.available()) {
        int c = client.read();
        if (c == 'S') {
          // Bei gestoerter Achse den Grund erneut melden statt loszufahren.
          if (g_faultCode != kFaultNone) sendFault(client);
          else g_axis.startSweep();
        } else if (c == 'X') {
          g_axis.stop();
        }
      }

      // Frames buendeln: 800 Einzelschreibvorgaenge/s waeren Verschwendung.
      size_t used = 0;
      FrameMsg msg;
      while (g_queue != nullptr && used + kMaxFrameSize <= sizeof(batch) &&
             xQueueReceive(g_queue, &msg, 0) == pdTRUE) {
        memcpy(batch + used, msg.bytes, msg.len);
        used += msg.len;
      }
      if (used > 0 && client.write(batch, used) != static_cast<int>(used)) {
        client.stop();
      }
    } else if (g_queue != nullptr) {
      // Niemand hoert auf dem TCP-Weg zu: die Queue trotzdem leeren, sonst
      // laeuft sie voll und der LiDAR-Task zaehlt lauter verworfene Frames.
      FrameMsg msg;
      while (xQueueReceive(g_queue, &msg, 0) == pdTRUE) {
      }
    }

    // Sweep fertig (inklusive Rueckfahrt) -> sofort melden, damit der Host
    // die Wolke abschliessen kann.
    const SweepState now = g_axis.state();
    const uint32_t ms = millis();
    if ((lastState != kStateIdle && now == kStateIdle) || ms - lastStatusMs >= 200) {
      lastStatusMs = ms;
      if (greeted) sendStatus(client);
      g_web.sendStatus(stateLabel(), g_axis.sweepIndex(),
                       g_axis.yawQ16Now() / 65536.0f,
                       g_faultCode != kFaultNone ? g_faultText : nullptr);
    }
    lastState = now;

    vTaskDelay(pdMS_TO_TICKS(2));
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

// Einen Hochlauffehler festhalten statt stehenzubleiben.
static void setFault(uint8_t code, const char *text) {
  if (g_faultCode != kFaultNone) return;  // der erste Fehler ist der Grund
  g_faultCode = code;
  snprintf(g_faultText, sizeof(g_faultText), "%s", text);
  Serial.printf("FEHLER: %s\n", text);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nlidar3d - RPLIDAR auf Gierachse");

  // Das Netz kommt zuerst, und zwar vor jeder Hardwarepruefung.
  //
  // Frueher stand es am Ende von setup(), hinter vier Stellen, die im
  // Fehlerfall in einer Endlosschleife haengen blieben. Fehlte also nur der
  // Servo, spannte der ESP32 nie ein WLAN auf und meldete sich nie am USB -
  // von aussen sah das aus, als sei die Firmware gar nicht drauf. Wer den
  // Grund wissen wollte, brauchte das serielle Kabel.
  //
  // Jetzt ist der Scanner immer erreichbar und sagt selbst, was fehlt.
  startWifi();

#if ENABLE_USB_NCM
  // Interface anlegen, aber Link noch unten lassen: iOS fragt DHCP genau
  // einmal beim Link-Up und wiederholt es nie. Erst wenn der TCP-Server
  // lauscht, wird freigegeben.
  bool ncmReady = usbNcmStart();
  if (!ncmReady) Serial.println("WARNUNG: USB-NCM liess sich nicht starten");
#endif

  g_server.begin();
  g_web.begin(onWebCommand);

#if ENABLE_USB_NCM
  if (ncmReady) {
    usbNcmSetLinkUp(true);
    Serial.println("USB-C bereit: Scanner unter 192.168.7.1:5005");
  }
#endif

  g_queue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(FrameMsg));
  if (g_queue == nullptr) {
    setFault(kFaultQueue, "Speicher fuer die Frame-Queue reicht nicht");
  }

  if (!g_axis.begin()) {
    // Ohne Servo waeren alle Gierwinkel gelogen - dann lieber nicht scannen.
    setFault(kFaultServo,
             "STS3215 antwortet nicht. Bus-ID, Baudrate (1 Mbaud), "
             "Halbduplex und 12 V pruefen.");
  } else {
    Serial.printf("Gierachse: Servo-Modell %u, %u Ebenen a %.2f deg, "
                  "Encoderaufloesung %.4f deg\n",
                  g_axis.servoModel(), g_axis.planeCount(), YAW_PLANE_STEP_DEG,
                  360.0 / SERVO_COUNTS_PER_REV);
  }

  if (g_faultCode == kFaultNone) {
#if LIDAR_LINK_USB
    const bool linkUp = g_link.begin(LIDAR_BAUDRATE, LIDAR_USB_WAIT_MS);
    const char *linkError =
        "Kein LiDAR am USB-Host-Port. Kabel, Adapter und 5-V-Versorgung "
        "pruefen (der C1 zieht mehr, als der USB-Port allein liefert).";
#else
    const bool linkUp = g_link.begin(static_cast<uart_port_t>(LIDAR_UART_NUM),
                                     LIDAR_RX_PIN, LIDAR_TX_PIN,
                                     LIDAR_BAUDRATE, LIDAR_RX_BUFFER);
    const char *linkError = "LiDAR-UART liess sich nicht oeffnen";
#endif
    if (!linkUp) {
      setFault(kFaultLidarPort, linkError);
    } else {
      g_lidar.begin(&g_link);
      g_lidar.setMotorRpm(LIDAR_RPM);
      delay(1500);  // Anlauf des LiDAR-Motors abwarten

      // Erst den einfachen Scanmodus versuchen - der C1 kann nur den, und beim
      // S2 scheitert er erst an der Bandbreite. Nur wenn er sich nicht starten
      // laesst, auf die Dense-Capsules ausweichen.
      if (g_lidar.startStandardScan()) {
        Serial.println("LiDAR laeuft im einfachen Scanmodus (C1)");
        g_scanRunning = true;
      } else {
        int mode = g_lidar.startDenseScan();
        if (mode < 0) {
          setFault(kFaultLidarScan,
                   "Kein Scanmodus startbar. Versorgung (5 V, >2 W) und "
                   "Baudrate pruefen (C1 460800, S2 1 Mbaud).");
        } else {
          Serial.printf("LiDAR laeuft mit Dense-Capsules, Scanmodus %d\n", mode);
          g_scanRunning = true;
        }
      }
    }
  }

  if (g_faultCode != kFaultNone) {
    Serial.println("Der Scanner bleibt im Netz erreichbar und meldet den Grund. "
                   "Nach dem Beheben neu starten.");
  }

  // LiDAR auf Core 1, Netz auf Core 0 (dort laeuft auch der WLAN-Stack).
  // Der Netz-Task laeuft auch im Fehlerfall - er ist der einzige Weg, den
  // Grund ohne serielles Kabel zu erfahren.
  if (g_queue != nullptr) {
    xTaskCreatePinnedToCore(lidarTask, "lidar", 4096, nullptr, 5, nullptr, 1);
  }
  xTaskCreatePinnedToCore(netTask, "net", 6144, nullptr, 4, nullptr, 0);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
