// Die Punktwolke live an einen Browser.
//
// Der Scanner liefert die Seite selbst aus (HTTP, Port 80) und schiebt die
// Punkte ueber einen WebSocket nach (Port 81). Damit braucht das iPhone nur
// Safari: keine App, kein Xcode, keine Signierung, und es laeuft auf jedem
// Geraet im Accesspoint des Scanners.
//
// Zwei Dinge sind hier wichtig:
//
// 1. GEBUENDELT. Ein WebSocket-Frame je Punkt waeren bei 5000 Messungen/s
//    5000 Frames pro Sekunde - der ESP32 kaeme nicht mit und das WLAN auch
//    nicht. Gesendet wird deshalb in Buendeln von kWebBatchPoints Punkten,
//    binaer statt als Text.
//
// 2. UEBER EINE QUEUE. Der LiDAR-Task darf den WebSocket nicht selbst
//    anfassen - die Bibliothek ist nicht threadsicher. Er legt Buendel ab,
//    der Netz-Task holt sie.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "geometry.h"

namespace nwl {

// 60 Punkte sind 724 Byte je Buendel und bei 5000 Messungen/s rund 83
// Buendel/s - gross genug, dass der Rahmen nicht ins Gewicht faellt, klein
// genug fuer eine fluessige Anzeige.
static const int kWebBatchPoints = 60;

struct PointBatch {
  uint16_t count;
  Point3 points[kWebBatchPoints];
};

// Wird aufgerufen, wenn die Seite ein Kommando schickt ('S' oder 'X').
typedef void (*WebCommandHandler)(char command);

class WebStream {
 public:
  // HTTP- und WebSocket-Server starten. Erst aufrufen, wenn das Netz steht.
  void begin(WebCommandHandler onCommand);

  // Im Netz-Task aufrufen: bedient HTTP, WebSocket und leert die Queue.
  void loop();

  // Aus dem LiDAR-Task aufrufen. Threadsicher, blockiert nie: ist die Queue
  // voll, geht das Buendel verloren und wird gezaehlt - lieber Punkte
  // verwerfen als den LiDAR bremsen.
  bool pushPoints(const Point3 *points, int count);

  // Zustandszeile fuer die Seite. fault darf nullptr sein.
  void sendStatus(const char *state, uint16_t planes, float yawDeg,
                  const char *fault);

  bool hasViewer() const { return viewers_ > 0; }
  uint32_t dropped() const { return dropped_; }

 private:
  int viewers_ = 0;
  uint32_t dropped_ = 0;
};

extern WebStream g_web;

}  // namespace nwl
