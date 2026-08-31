// Dekoder fuer den einfachen Scanmodus (Antworttyp 0x81) des RPLIDAR C1.
//
// Der C1 liefert 5000 Messungen/s. Bei 5 Byte je Messung sind das 25 kB/s von
// den rund 46 kB/s, die 460800 Baud hergeben - der einfache Modus reicht also
// bequem, und der dense-capsuled Modus des S2 wird hier nicht gebraucht.
//
// Bewusst frei von Arduino- und IDF-Abhaengigkeiten, damit die Logik nativ
// getestet werden kann. Gegenstueck auf der Hostseite:
// host/scan3d/rplidar.py (StandardScanParser) - beide muessen dieselben
// Winkel liefern.
//
// Aufbau einer Messung (5 Byte):
//   Byte 0  Bit 0    S       Beginn einer Umdrehung
//           Bit 1    !S      muss das Gegenteil von S sein
//           Bit 2..7 quality
//   Byte 1  Bit 0    check   muss 1 sein
//           Bit 1..7 angle_q6, untere 7 Bit
//   Byte 2           angle_q6, obere 8 Bit
//   Byte 3..4        distance_q2, little endian
//
// Anders als beim S2 traegt jede Messung ihren eigenen Winkel. Der ist nicht
// exakt gleichmaessig verteilt, deshalb wird er weitergereicht statt
// interpoliert.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace nwl {

static const int kStandardNodeSize = 5;

struct ScanSample {
  uint16_t angleQ6;      // 0 .. 360*64-1
  uint16_t distanceMm;   // 0 = ungueltig (kein Echo)
  uint8_t quality;
  bool newRevolution;    // erste Messung einer Umdrehung
};

typedef void (*ScanSampleSink)(void *ctx, const ScanSample &sample);

// Byte-Strom -> Messungen. Faengt der Strom mitten in einer Messung an oder
// gehen Bytes verloren, wird byteweise resynchronisiert: die beiden
// Pruefbits sind das einzige Raster, das der einfache Modus hergibt.
class StandardScanParser {
 public:
  StandardScanParser() { reset(); }

  void reset() {
    fill_ = 0;
    resyncs_ = 0;
    memset(node_, 0, sizeof(node_));
  }

  void feed(const uint8_t *data, size_t len, ScanSampleSink sink, void *ctx) {
    for (size_t i = 0; i < len; ++i) {
      if (push(data[i])) sink(ctx, sample_);
    }
  }

  uint32_t resyncs() const { return resyncs_; }
  // Der einfache Modus hat keine Pruefsumme; einheitliche Schnittstelle zum
  // Dense-Capsule-Parser, damit die Statusframes gleich bleiben.
  uint32_t checksumErrors() const { return 0; }

  // Sichtbar fuer die Tests: taugt dieses Bytepaar als Messungsanfang?
  static bool nodeLooksValid(uint8_t b0, uint8_t b1) {
    const uint8_t start = b0 & 0x01;
    const uint8_t startInverted = (b0 >> 1) & 0x01;
    const uint8_t check = b1 & 0x01;
    return start != startInverted && check == 1;
  }

 private:
  bool push(uint8_t byte) {
    if (fill_ == 0) {
      // Erstes Byte: nur die beiden S-Bits pruefbar, das genuegt vorerst.
      if (((byte & 0x01) ^ ((byte >> 1) & 0x01)) == 0) {
        ++resyncs_;
        return false;
      }
      node_[fill_++] = byte;
      return false;
    }
    if (fill_ == 1 && !nodeLooksValid(node_[0], byte)) {
      // Das erste Byte war ein Fehlstart. Es koennte aber selbst ein
      // gueltiger Anfang sein - deshalb neu bewerten statt wegwerfen.
      ++resyncs_;
      fill_ = 0;
      return push(byte);
    }

    node_[fill_++] = byte;
    if (fill_ < kStandardNodeSize) return false;

    fill_ = 0;
    sample_.newRevolution = (node_[0] & 0x01) != 0;
    sample_.quality = static_cast<uint8_t>(node_[0] >> 2);
    sample_.angleQ6 = static_cast<uint16_t>((node_[1] >> 1) |
                                            (static_cast<uint16_t>(node_[2]) << 7));
    const uint16_t distanceQ2 =
        static_cast<uint16_t>(node_[3] | (static_cast<uint16_t>(node_[4]) << 8));
    // Q2 sind Viertelmillimeter. Auf ganze mm runden: bei 30 mm Messfehler
    // laut Datenblatt ist der Viertelmillimeter ohnehin Zierrat, und mm
    // passen in dieselben 16 Bit wie beim S2.
    sample_.distanceMm = static_cast<uint16_t>((distanceQ2 + 2) / 4);
    return true;
  }

  uint8_t node_[kStandardNodeSize];
  uint8_t fill_;
  uint32_t resyncs_;
  ScanSample sample_;
};

}  // namespace nwl
