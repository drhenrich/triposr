// Siehe web_stream.h.

#include "web_stream.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdio.h>
#include <string.h>

#include "../include/config.h"
#include "web_ui.h"

namespace nwl {

WebStream g_web;

namespace {

WebServer g_http(80);
WebSocketsServer g_ws(81);
QueueHandle_t g_batches = nullptr;
WebCommandHandler g_onCommand = nullptr;
int *g_viewerCount = nullptr;

// Der Header eines Punktbuendels: Typ, reserviert, Anzahl. Danach folgen
// count * 3 float32 (x, y, z in Metern, little endian) - so, wie sie im
// Speicher stehen, ohne Umkopieren.
const uint8_t kMsgPoints = 1;
const size_t kHeaderSize = 4;

uint8_t g_frame[kHeaderSize + kWebBatchPoints * sizeof(Point3)];

void onWsEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      if (g_viewerCount != nullptr) ++(*g_viewerCount);
      Serial.printf("Browser verbunden (%u)\n", client);
      break;
    case WStype_DISCONNECTED:
      if (g_viewerCount != nullptr && *g_viewerCount > 0) --(*g_viewerCount);
      Serial.printf("Browser getrennt (%u)\n", client);
      break;
    case WStype_TEXT:
      if (length > 0 && g_onCommand != nullptr) {
        g_onCommand(static_cast<char>(payload[0]));
      }
      break;
    default:
      break;
  }
}

void handleIndex() {
  // Die Seite liegt im Flash, nicht im RAM - sie ist einige Kilobyte gross.
  g_http.send_P(200, "text/html; charset=utf-8", kIndexHtml);
}

}  // namespace

void WebStream::begin(WebCommandHandler onCommand) {
  g_onCommand = onCommand;
  g_viewerCount = &viewers_;

  if (g_batches == nullptr) {
    g_batches = xQueueCreate(WEB_BATCH_QUEUE_LENGTH, sizeof(PointBatch));
  }

  g_http.on("/", handleIndex);
  // Ohne das antwortet der Scanner auf jeden anderen Pfad mit 404, und iOS
  // haelt das Netz fuer kaputt ("Kein Internet"). Alles auf die Seite lenken.
  g_http.onNotFound(handleIndex);
  g_http.begin();

  g_ws.begin();
  g_ws.onEvent(onWsEvent);
  Serial.println("Webseite: http://192.168.4.1/  (WebSocket auf Port 81)");
}

bool WebStream::pushPoints(const Point3 *points, int count) {
  if (g_batches == nullptr || count <= 0) return false;
  if (count > kWebBatchPoints) count = kWebBatchPoints;

  PointBatch batch;
  batch.count = static_cast<uint16_t>(count);
  for (int i = 0; i < count; ++i) batch.points[i] = points[i];

  if (xQueueSend(g_batches, &batch, 0) != pdTRUE) {
    ++dropped_;
    return false;
  }
  return true;
}

void WebStream::sendStatus(const char *state, uint16_t planes, float yawDeg,
                           const char *fault) {
  if (viewers_ == 0) return;
  char json[240];
  // Der Fehlertext kommt aus der Firmware, nicht von aussen - er enthaelt
  // keine Anfuehrungszeichen. Trotzdem gekappt, damit der Puffer reicht.
  snprintf(json, sizeof(json),
           "{\"state\":\"%s\",\"planes\":%u,\"yaw\":%.1f,\"fault\":\"%.120s\"}",
           state, static_cast<unsigned>(planes), static_cast<double>(yawDeg),
           fault != nullptr ? fault : "");
  g_ws.broadcastTXT(json);
}

void WebStream::loop() {
  g_http.handleClient();
  g_ws.loop();

  if (g_batches == nullptr) return;

  // Nur so viele Buendel je Durchlauf, dass HTTP und WebSocket zwischendurch
  // wieder drankommen. Ohne diese Grenze verhungert die Bedienung, sobald der
  // LiDAR schneller liefert, als das WLAN abnimmt.
  PointBatch batch;
  for (int sent = 0; sent < 8; ++sent) {
    if (xQueueReceive(g_batches, &batch, 0) != pdTRUE) break;
    if (viewers_ == 0) continue;  // niemand schaut zu: verwerfen, nicht stauen

    g_frame[0] = kMsgPoints;
    g_frame[1] = 0;
    g_frame[2] = static_cast<uint8_t>(batch.count);
    g_frame[3] = static_cast<uint8_t>(batch.count >> 8);
    const size_t bytes = batch.count * sizeof(Point3);
    memcpy(g_frame + kHeaderSize, batch.points, bytes);
    g_ws.broadcastBIN(g_frame, kHeaderSize + bytes);
  }
}

}  // namespace nwl
