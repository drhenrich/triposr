// Nativer Test der hardwareunabhaengigen Firmware-Logik.
//
//   cd firmware/test/native && make
//
// Prueft Capsule-Parser, Winkeldekoder, Gier-Festkommamathematik und das
// Byte-Layout der Frames. Das Frame-Layout wird gegen dieselbe Fixture
// geprueft wie der Python-Host (tests/wire_fixture.txt) - damit koennen die
// beiden Seiten des Protokolls nicht unbemerkt auseinanderlaufen.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "../../src/angle_util.h"
#include "../../src/dense_capsule.h"
#include "../../src/feetech_bus.h"
#include "../../src/geometry.h"
#include "../../src/standard_scan.h"
#include "../../src/stream_proto.h"
#include "../../src/sweep_plan.h"

using namespace nwl;

// --- winziges Testgeruest ------------------------------------------------

static int g_failures = 0;
static int g_checks = 0;
static const char *g_case = "";

#define CHECK(cond)                                                          \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_failures;                                                          \
      std::printf("  FAIL %s:%d [%s] %s\n", __FILE__, __LINE__, g_case,      \
                  #cond);                                                    \
    }                                                                        \
  } while (0)

#define CHECK_EQ(a, b)                                                       \
  do {                                                                       \
    ++g_checks;                                                              \
    auto va_ = (a);                                                          \
    auto vb_ = (b);                                                          \
    if (!(va_ == vb_)) {                                                     \
      ++g_failures;                                                          \
      std::printf("  FAIL %s:%d [%s] %s == %s (%lld vs %lld)\n", __FILE__,   \
                  __LINE__, g_case, #a, #b, (long long)va_, (long long)vb_); \
    }                                                                        \
  } while (0)

#define CASE(name)                                                           \
  g_case = name;                                                             \
  std::printf("- %s\n", name);

// --- Hilfsmittel ---------------------------------------------------------

// Baut eine gueltige Dense-Capsule, identisch zu build_capsule() im Pythontest.
static std::vector<uint8_t> buildCapsule(double startAngleDeg,
                                         const uint16_t *distances,
                                         bool startFlag = false) {
  uint16_t raw = static_cast<uint16_t>(std::lround(startAngleDeg * 64.0)) & 0x7FFF;
  if (startFlag) raw |= 0x8000;

  std::vector<uint8_t> body;
  body.push_back(static_cast<uint8_t>(raw));
  body.push_back(static_cast<uint8_t>(raw >> 8));
  for (int i = 0; i < kDenseCabinCount; ++i) {
    body.push_back(static_cast<uint8_t>(distances[i]));
    body.push_back(static_cast<uint8_t>(distances[i] >> 8));
  }
  uint8_t checksum = 0;
  for (uint8_t b : body) checksum ^= b;

  std::vector<uint8_t> out;
  out.push_back(static_cast<uint8_t>((kExpSync1 << 4) | (checksum & 0x0F)));
  out.push_back(static_cast<uint8_t>((kExpSync2 << 4) | ((checksum >> 4) & 0x0F)));
  out.insert(out.end(), body.begin(), body.end());
  return out;
}

struct Collector {
  std::vector<DenseCapsule> capsules;
};

static void collect(void *ctx, const DenseCapsule &capsule) {
  static_cast<Collector *>(ctx)->capsules.push_back(capsule);
}

static std::vector<uint16_t> rampDistances(uint16_t base = 1000, uint16_t step = 1) {
  std::vector<uint16_t> d(kDenseCabinCount);
  for (int i = 0; i < kDenseCabinCount; ++i) {
    d[i] = static_cast<uint16_t>(base + step * i);
  }
  return d;
}

static std::string toHex(const uint8_t *data, size_t len) {
  static const char *digits = "0123456789abcdef";
  std::string out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out.push_back(digits[data[i] >> 4]);
    out.push_back(digits[data[i] & 0x0F]);
  }
  return out;
}

static std::map<std::string, std::string> loadFixture(const char *path) {
  std::map<std::string, std::string> out;
  std::ifstream in(path);
  if (!in) {
    std::printf("  FAIL Fixture nicht lesbar: %s\n", path);
    ++g_failures;
    return out;
  }
  std::string name, hex;
  while (in >> name) {
    if (!name.empty() && name[0] == '#') {
      std::getline(in, hex);
      continue;
    }
    if (!(in >> hex)) break;
    out[name] = hex;
  }
  return out;
}

// --- Tests ---------------------------------------------------------------

static void testParserHappyPath() {
  CASE("Parser: sauberer Strom");
  auto d = rampDistances();
  std::vector<uint8_t> stream;
  for (int i = 0; i < 20; ++i) {
    auto c = buildCapsule(i * 4.5, d.data());
    stream.insert(stream.end(), c.begin(), c.end());
  }

  Collector sink;
  CapsuleParser parser;
  parser.feed(stream.data(), stream.size(), 1000000, 10000, collect, &sink);

  CHECK_EQ(sink.capsules.size(), 20u);
  CHECK_EQ(parser.checksumErrors(), 0u);
  CHECK_EQ(parser.resyncs(), 0u);
  CHECK_EQ(sink.capsules[0].startAngleQ6, 0);
  CHECK_EQ(sink.capsules[1].startAngleQ6, static_cast<uint16_t>(4.5 * 64));
  CHECK_EQ(sink.capsules[5].distanceMm[7], 1007);
  CHECK(!sink.capsules[0].startFlag);
}

static void testParserStartFlag() {
  CASE("Parser: startFlag wird ausgewertet");
  auto d = rampDistances();
  auto c = buildCapsule(12.5, d.data(), /*startFlag=*/true);
  Collector sink;
  CapsuleParser parser;
  parser.feed(c.data(), c.size(), 0, 10000, collect, &sink);
  CHECK_EQ(sink.capsules.size(), 1u);
  CHECK(sink.capsules[0].startFlag);
  CHECK_EQ(sink.capsules[0].startAngleQ6, 800);  // 12.5 * 64
}

static void testParserSplitFeeds() {
  CASE("Parser: Capsule ueber mehrere Lesevorgaenge verteilt");
  auto d = rampDistances();
  auto c = buildCapsule(90.0, d.data());
  Collector sink;
  CapsuleParser parser;
  parser.feed(c.data(), 30, 0, 10000, collect, &sink);
  CHECK_EQ(sink.capsules.size(), 0u);
  parser.feed(c.data() + 30, c.size() - 30, 0, 10000, collect, &sink);
  CHECK_EQ(sink.capsules.size(), 1u);
  CHECK_EQ(sink.capsules[0].startAngleQ6, 5760);  // 90 * 64
}

static void testParserLeadingGarbage() {
  CASE("Parser: synchronisiert nach Muell");
  auto d = rampDistances();
  auto c = buildCapsule(45.0, d.data());
  std::vector<uint8_t> stream = {0x11, 0x22, 0x33, 0x44};
  stream.insert(stream.end(), c.begin(), c.end());

  Collector sink;
  CapsuleParser parser;
  parser.feed(stream.data(), stream.size(), 0, 10000, collect, &sink);
  CHECK_EQ(sink.capsules.size(), 1u);
  CHECK(parser.resyncs() >= 3);
  CHECK_EQ(parser.checksumErrors(), 0u);
}

static void testParserBadChecksumRecovers() {
  CASE("Parser: kippt eine Capsule, faengt die naechste");
  auto d = rampDistances();
  auto bad = buildCapsule(0.0, d.data());
  bad[10] ^= 0xFF;  // eine Distanz verfaelschen
  auto good = buildCapsule(180.0, d.data());

  std::vector<uint8_t> stream(bad.begin(), bad.end());
  stream.insert(stream.end(), good.begin(), good.end());

  Collector sink;
  CapsuleParser parser;
  parser.feed(stream.data(), stream.size(), 0, 10000, collect, &sink);
  CHECK_EQ(parser.checksumErrors(), 1u);
  CHECK_EQ(sink.capsules.size(), 1u);
  CHECK_EQ(sink.capsules[0].startAngleQ6, 11520);  // 180 * 64
}

static void testParserTimestamps() {
  CASE("Parser: Zeitstempel aus Byteposition");
  auto d = rampDistances();
  std::vector<uint8_t> stream;
  for (int i = 0; i < 3; ++i) {
    auto c = buildCapsule(i * 4.5, d.data());
    stream.insert(stream.end(), c.begin(), c.end());
  }

  Collector sink;
  CapsuleParser parser;
  const int64_t tLast = 1000000;
  const uint32_t byteTimeNs = 10000;  // 1 Mbaud, 8N1 -> 10 us je Byte
  parser.feed(stream.data(), stream.size(), tLast, byteTimeNs, collect, &sink);

  CHECK_EQ(sink.capsules.size(), 3u);
  // Die letzte Capsule endet exakt beim letzten Byte, die davor 84 Byte frueher.
  CHECK_EQ(sink.capsules[2].timestampUs, tLast);
  CHECK_EQ(sink.capsules[1].timestampUs, tLast - 84 * 10);
  CHECK_EQ(sink.capsules[0].timestampUs, tLast - 2 * 84 * 10);
}

static void testDecoderInterpolation() {
  CASE("Dekoder: Winkel zwischen zwei Capsules");
  auto d = rampDistances(2000, 0);
  Collector sink;
  CapsuleParser parser;
  auto a = buildCapsule(0.0, d.data());
  auto b = buildCapsule(4.0, d.data());
  parser.feed(a.data(), a.size(), 0, 10000, collect, &sink);
  parser.feed(b.data(), b.size(), 1000, 10000, collect, &sink);
  CHECK_EQ(sink.capsules.size(), 2u);

  CapsuleDecoder decoder;
  CapsuleSpan span;
  CHECK(!decoder.push(sink.capsules[0], span));  // erste Capsule wird gehalten
  CHECK(decoder.push(sink.capsules[1], span));

  CHECK_EQ(span.alphaStartQ6, 0);
  // 4 deg auf 40 Messungen: (4*64*4 << 8) / 40 = 6553 (Q16)
  CHECK_EQ(span.alphaIncQ16, 6553);
  CHECK_EQ(span.revolutionIndex, -1);
  CHECK_EQ(span.timestampUs, 0);  // Zeitstempel der zurueckgehaltenen Capsule
  CHECK_EQ(span.distanceMm[3], 2000);
}

// Startwinkel in Q6, laufender Dekoder ueber mehrere Umdrehungen.
static int countRevolutions(double stepDeg, int capsuleCount, int *firstIndex) {
  auto d = rampDistances(2000, 0);
  CapsuleDecoder decoder;
  int revolutions = 0;
  int sampleIndex = 0;
  *firstIndex = -1;

  for (int i = 0; i < capsuleCount; ++i) {
    double angle = std::fmod(i * stepDeg, 360.0);
    auto raw = buildCapsule(angle, d.data());
    Collector sink;
    CapsuleParser parser;
    parser.feed(raw.data(), raw.size(), 0, 10000, collect, &sink);

    CapsuleSpan span;
    if (!decoder.push(sink.capsules[0], span)) continue;
    if (span.revolutionIndex >= 0) {
      if (*firstIndex < 0) *firstIndex = sampleIndex + span.revolutionIndex;
      ++revolutions;
    }
    sampleIndex += kDenseCabinCount;
  }
  return revolutions;
}

static void testDecoderRevolutionAligned() {
  CASE("Dekoder: Umlauf bei Capsules exakt auf der 360-Grad-Grenze");
  int first = -1;
  int revs = countRevolutions(360.0 / 80, 161, &first);
  CHECK_EQ(revs, 1);
  // 10 Hz bei 32000 Messungen/s -> 3200 Messungen je Umdrehung
  CHECK_EQ(first, 3200);
}

static void testDecoderRevolutionUnaligned() {
  CASE("Dekoder: Umlauf mitten in einer Capsule");
  int first = -1;
  int revs = countRevolutions(7.0, 120, &first);
  CHECK_EQ(revs, 2);
  CHECK(first > 0);
}

// --- Gierachse: Encoderumrechnung und Sweep-Plan --------------------------

static void testAngleUtil() {
  CASE("Gier: Encoderzaehlwerte <-> Grad");
  const int32_t kCounts = 4096;  // STS3215, 12 Bit absolut

  CHECK_EQ(countsToQ16(kCounts, kCounts), 360 << 16);
  CHECK_EQ(countsToQ16(kCounts / 4, kCounts), 90 << 16);
  CHECK_EQ(countsToQ16(0, kCounts), 0);

  // Ein Zaehlwert entspricht 360/4096 = 0.087890625 Grad.
  CHECK(std::fabs(q16ToDeg(countsToQ16(1, kCounts)) - 0.087890625) < 1e-6);

  CHECK_EQ(q16ToCounts(degToQ16(180.0), kCounts), 2048);
  CHECK_EQ(q16ToCounts(degToQ16(0.0), kCounts), 0);
  // 1 Grad sind 11.38 Zaehlwerte, wird kaufmaennisch gerundet.
  CHECK_EQ(q16ToCounts(degToQ16(1.0), kCounts), 11);
  CHECK_EQ(q16ToCounts(degToQ16(-1.0), kCounts), -11);

  // Hin und zurueck bleibt innerhalb eines Zaehlwerts.
  for (int32_t counts = 0; counts < kCounts; counts += 137) {
    CHECK_EQ(q16ToCounts(countsToQ16(counts, kCounts), kCounts), counts);
  }
}

static void testSweepPlan() {
  CASE("Sweep: Ebenen bei halboffenem Bereich");
  SweepPlan plan;  // 0..180 Grad in 1-Grad-Schritten
  CHECK_EQ(plan.planeCount(), 180);
  CHECK_EQ(plan.yawForPlane(0), 0);
  CHECK_EQ(plan.yawForPlane(1), degToQ16(1.0));
  // Kernpunkt: die letzte Ebene liegt bei 179, nicht bei 180 Grad. Bei 180
  // waere es dieselbe Ebene wie bei 0, weil der LiDAR 360 Grad misst.
  CHECK_EQ(plan.yawForPlane(179), degToQ16(179.0));
  CHECK(plan.yawForPlane(plan.planeCount()) == plan.endQ16);

  SweepPlan fine;
  fine.stepQ16 = degToQ16(0.5);
  CHECK_EQ(fine.planeCount(), 360);

  SweepPlan quarter;
  quarter.endQ16 = degToQ16(90.0);
  CHECK_EQ(quarter.planeCount(), 90);
}

static void testSweepPlanDegenerate() {
  CASE("Sweep: unsinnige Bereiche liefern null Ebenen");
  SweepPlan zeroStep;
  zeroStep.stepQ16 = 0;
  CHECK_EQ(zeroStep.planeCount(), 0);

  SweepPlan reversed;
  reversed.startQ16 = degToQ16(180.0);
  reversed.endQ16 = degToQ16(0.0);
  CHECK_EQ(reversed.planeCount(), 0);

  SweepPlan empty;
  empty.endQ16 = empty.startQ16;
  CHECK_EQ(empty.planeCount(), 0);
}

// --- Feetech-Busprotokoll -------------------------------------------------

static void testFeetechPacketLayout() {
  CASE("Feetech: Paketaufbau und Pruefsumme");
  uint8_t buf[feetech::kMaxPacketSize];

  // Drehmoment einschalten: FF FF 01 04 03 28 01 CE
  size_t n = feetech::buildWrite8(buf, 1, feetech::kRegTorqueEnable, 1);
  CHECK_EQ(n, 8u);
  CHECK(toHex(buf, n) == "ffff0104032801ce");

  // Istposition lesen: FF FF 01 04 02 38 02 BE
  n = feetech::buildRead(buf, 1, feetech::kRegPresentPositionL, 2);
  CHECK_EQ(n, 8u);
  CHECK(toHex(buf, n) == "ffff0104023802be");

  // Modellnummer lesen: FF FF 01 04 02 03 02 F3
  n = feetech::buildRead(buf, 1, feetech::kRegModelL, 2);
  CHECK_EQ(n, 8u);
  CHECK(toHex(buf, n) == "ffff0104020302f3");

  // Ping: FF FF 01 02 01 FB
  n = feetech::buildPing(buf, 1);
  CHECK_EQ(n, 6u);
  CHECK(toHex(buf, n) == "ffff010201fb");

  // Eine andere Bus-ID aendert Adressbyte und Pruefsumme.
  n = feetech::buildPing(buf, 5);
  CHECK_EQ(buf[2], 5);
  CHECK_EQ(buf[5], feetech::checksum(buf, 5));
}

static void testFeetechMovePacket() {
  CASE("Feetech: Fahrbefehl schreibt ACC..GOAL_SPEED am Stueck");
  uint8_t buf[feetech::kMaxPacketSize];
  size_t n = feetech::buildMove(buf, 1, 1024, 1000, 50);

  CHECK_EQ(n, 14u);
  CHECK_EQ(buf[2], 1);                            // ID
  CHECK_EQ(buf[3], 10);                           // LEN = 7 Daten + reg + 2
  CHECK_EQ(buf[4], feetech::kWrite);
  CHECK_EQ(buf[5], feetech::kRegAcc);             // 41, danach 42..47
  CHECK_EQ(buf[6], 50);                           // ACC
  CHECK_EQ(feetech::get16(buf + 7), 1024);        // Zielposition, little endian
  CHECK_EQ(feetech::get16(buf + 9), 0);           // GOAL_TIME = 0
  CHECK_EQ(feetech::get16(buf + 11), 1000);       // Zieldrehzahl
  CHECK_EQ(buf[13], feetech::checksum(buf, 13));
  CHECK(toHex(buf, n) == "ffff010a03293200040000e803a7");
}

static void testFeetechSignedValues() {
  CASE("Feetech: Vorzeichen steckt in Bit 15, nicht im Zweierkomplement");
  CHECK_EQ(feetech::decodeSigned(0), 0);
  CHECK_EQ(feetech::decodeSigned(1000), 1000);
  CHECK_EQ(feetech::decodeSigned(0x8000 | 1000), -1000);
  CHECK_EQ(feetech::decodeSigned(0x7FFF), 32767);
  CHECK_EQ(feetech::decodeSigned(0x8001), -1);
}

// Baut ein gueltiges Statuspaket: FF FF ID LEN ERR PARAM... CHK
static std::vector<uint8_t> buildStatus(uint8_t id, uint8_t error,
                                        const std::vector<uint8_t> &params) {
  std::vector<uint8_t> packet = {0xFF, 0xFF, id,
                                 static_cast<uint8_t>(params.size() + 2), error};
  packet.insert(packet.end(), params.begin(), params.end());
  packet.push_back(feetech::checksum(packet.data(), packet.size()));
  return packet;
}

static void testFeetechStatusParser() {
  CASE("Feetech: Statuspakete lesen");
  feetech::StatusParser parser;
  feetech::StatusPacket packet;

  auto raw = buildStatus(1, 0, {0x00, 0x04});  // Position 1024
  bool got = false;
  for (uint8_t b : raw) got = parser.push(b, packet) || got;
  CHECK(got);
  CHECK_EQ(packet.id, 1);
  CHECK_EQ(packet.error, 0);
  CHECK_EQ(packet.paramCount, 2);
  CHECK_EQ(feetech::get16(packet.params), 1024);
}

static void testFeetechStatusParserRobustness() {
  CASE("Feetech: Muell, Echo und kaputte Pruefsummen");
  feetech::StatusParser parser;
  feetech::StatusPacket packet;

  // Halbduplex: erst das eigene Echo, dann die Antwort. Beides muss
  // durchlaufen, ohne dass die Antwort verloren geht.
  uint8_t request[feetech::kMaxPacketSize];
  size_t reqLen = feetech::buildRead(request, 1, feetech::kRegPresentPositionL, 2);
  auto response = buildStatus(1, 0, {0x34, 0x12});

  std::vector<uint8_t> stream(request, request + reqLen);
  stream.insert(stream.end(), response.begin(), response.end());

  int packets = 0;
  for (uint8_t b : stream) {
    if (parser.push(b, packet)) ++packets;
  }
  // Das Echo ist selbst ein gueltig gerahmtes Paket, deshalb koennen zwei
  // durchkommen; entscheidend ist, dass das letzte die Antwort ist.
  CHECK(packets >= 1);
  CHECK_EQ(feetech::get16(packet.params), 0x1234);

  // Kaputte Pruefsumme wird verworfen, das folgende Paket wieder gefunden.
  feetech::StatusParser second;
  auto broken = buildStatus(1, 0, {0x00, 0x04});
  broken.back() ^= 0xFF;
  auto good = buildStatus(1, 0, {0x11, 0x22});
  std::vector<uint8_t> mixed(broken.begin(), broken.end());
  mixed.insert(mixed.end(), good.begin(), good.end());

  int accepted = 0;
  for (uint8_t b : mixed) {
    if (second.push(b, packet)) ++accepted;
  }
  CHECK_EQ(accepted, 1);
  CHECK_EQ(second.checksumErrors(), 1u);
  CHECK_EQ(feetech::get16(packet.params), 0x2211);

  // Mehrere Kopfbytes hintereinander duerfen nicht als ID durchgehen.
  feetech::StatusParser third;
  std::vector<uint8_t> padded = {0xFF, 0xFF, 0xFF};
  auto tail = buildStatus(2, 0, {0x07});
  padded.insert(padded.end(), tail.begin() + 2, tail.end());
  int found = 0;
  for (uint8_t b : padded) {
    if (third.push(b, packet)) ++found;
  }
  CHECK_EQ(found, 1);
  CHECK_EQ(packet.id, 2);
  CHECK_EQ(packet.params[0], 7);
}

static void testWireFormat(const char *fixturePath) {
  CASE("Protokoll: Bytes stimmen mit der Fixture ueberein");
  auto fixture = loadFixture(fixturePath);
  if (fixture.empty()) return;

  uint8_t buf[kMaxFrameSize];

  uint16_t distances[kDenseCabinCount];
  for (int i = 0; i < kDenseCabinCount; ++i) {
    distances[i] = static_cast<uint16_t>(1000 + 7 * i);
  }
  CapsuleSpan span;
  span.alphaStartQ6 = 7888;   // 123.25 deg
  span.alphaIncQ16 = 7373;    // 0.1125 deg
  span.distanceMm = distances;
  span.timestampUs = 0;
  span.revolutionIndex = 0;

  size_t n = writeCapsuleFrame(buf, 0x1234,
                               kFlagSweepActive | kFlagNewRevolution, span,
                               2785280u, 2786099u);
  CHECK_EQ(n, kCapsuleFrameSize);
  CHECK_EQ(n, 104u);
  CHECK(toHex(buf, n) == fixture["capsule"]);

  n = writeHelloFrame(buf, 1, 1, 600, -40500, 12000, 0, 11796480u);
  CHECK_EQ(n, kHeaderSize + kHelloPayloadSize);
  CHECK(toHex(buf, n) == fixture["hello"]);

  n = writeStatusFrame(buf, 2, 3, kFrameStatus, 5931008u, 123456u, 7u, 2u);
  CHECK_EQ(n, kHeaderSize + kStatusPayloadSize);
  CHECK(toHex(buf, n) == fixture["status"]);

  // Scanframe (C1). Die Winkel sind absichtlich ungleichmaessig - genau so
  // liefert der C1 sie, und genau deshalb traegt jede Messung ihren eigenen.
  static const double kAlpha[8] = {0.0, 0.72, 1.51, 2.19, 2.95, 3.68, 4.39, 5.14};
  ScanSample samples[8];
  for (int i = 0; i < 8; ++i) {
    samples[i].angleQ6 = static_cast<uint16_t>(std::lround(kAlpha[i] * 64.0));
    samples[i].distanceMm = static_cast<uint16_t>(1000 + 5 * i);
    samples[i].quality = 47;
    samples[i].newRevolution = (i == 0);
  }
  n = writeScanFrame(buf, 4, kFlagNewRevolution | kFlagSweepActive, samples, 8,
                     2752512u, 2785280u);  // 42.0 und 42.5 Grad in Q16
  CHECK_EQ(n, kHeaderSize + kScanHeadSize + kScanSampleSize * 8);
  CHECK_EQ(n, 52u);
  CHECK(toHex(buf, n) == fixture["scan"]);

  // Fehlerframe: der Scanner bleibt erreichbar und sagt, was fehlt.
  const char *faultText =
      "STS3215 antwortet nicht. Bus-ID, Baudrate (1 Mbaud), Halbduplex und "
      "12 V pruefen.";
  n = writeFaultFrame(buf, 9, kFaultServo, faultText);
  CHECK_EQ(n, kHeaderSize + 2 + std::strlen(faultText));
  CHECK(toHex(buf, n) == fixture["fault"]);
}

// --- Geometrie auf dem Geraet --------------------------------------------

// --- Halbduplex: das eigene Echo ------------------------------------------
//
// Ohne Transceiver kommt das Gesendete als Erstes zurueck. Ein Kommando ist
// aber genauso gerahmt wie eine Antwort - der StatusParser haelt es fuer ein
// gueltiges Paket mit passender ID und passender Parameterzahl. Genau daran
// meldete ping() Erfolg, obwohl kein Servo antwortete.

// Alle Bytes durch Filter und Parser schicken; zurueck kommt die Anzahl
// erkannter Pakete und das letzte davon.
static int runBus(const uint8_t *request, size_t reqLen,
                  const std::vector<uint8_t> &stream,
                  feetech::StatusPacket &last) {
  feetech::EchoFilter echo;
  echo.reset(request, reqLen);
  feetech::StatusParser parser;
  uint8_t forward[feetech::kMaxPacketSize + 1];
  int packets = 0;
  for (uint8_t b : stream) {
    size_t n = echo.push(b, forward);
    for (size_t i = 0; i < n; ++i) {
      if (parser.push(forward[i], last)) ++packets;
    }
  }
  return packets;
}

static void testEchoAloneIsNotAnAnswer() {
  CASE("Feetech: das eigene Echo gilt nicht als Antwort");
  uint8_t request[feetech::kMaxPacketSize];
  size_t reqLen = feetech::buildPing(request, 1);

  // Nur das Echo, kein Servo dahinter. Vorher kam hier ein Paket durch, und
  // ping() meldete Erfolg an einem Bus ohne Servo.
  feetech::StatusPacket last;
  std::vector<uint8_t> onlyEcho(request, request + reqLen);
  CHECK_EQ(runBus(request, reqLen, onlyEcho, last), 0);
}

static void testEchoIsSkippedAndTheAnswerArrives() {
  CASE("Feetech: Echo wird verworfen, die Antwort kommt an");
  uint8_t request[feetech::kMaxPacketSize];
  size_t reqLen = feetech::buildRead(request, 1, feetech::kRegModelL, 2);
  auto answer = buildStatus(1, 0, {0x09, 0x03});

  std::vector<uint8_t> stream(request, request + reqLen);
  stream.insert(stream.end(), answer.begin(), answer.end());

  feetech::StatusPacket last;
  CHECK_EQ(runBus(request, reqLen, stream, last), 1);
  CHECK_EQ(feetech::get16(last.params), 0x0309);
}

static void testWithoutEchoNothingIsSwallowed() {
  CASE("Feetech: mit Transceiver geht der Antwortanfang nicht verloren");
  // Antwort und Kommando beginnen beide mit FF FF. Der Filter darf die
  // gemeinsamen Bytes nicht behalten, wenn gar kein Echo kommt.
  uint8_t request[feetech::kMaxPacketSize];
  size_t reqLen = feetech::buildRead(request, 1, feetech::kRegModelL, 2);
  auto answer = buildStatus(1, 0, {0x09, 0x03});

  feetech::StatusPacket last;
  CHECK_EQ(runBus(request, reqLen, std::vector<uint8_t>(answer.begin(),
                                                       answer.end()), last), 1);
  CHECK_EQ(feetech::get16(last.params), 0x0309);
}

static void testEchoFilterSurvivesAPartialMatch() {
  CASE("Feetech: teilweise gleicher Anfang wird nachgereicht");
  uint8_t request[feetech::kMaxPacketSize];
  size_t reqLen = feetech::buildPing(request, 1);

  // Etwas, das die ersten drei Bytes des Kommandos teilt und dann abweicht -
  // die Antwort eines anderen Servos etwa. Nichts davon darf verschwinden.
  auto answer = buildStatus(1, 0, {0x42});
  feetech::StatusPacket last;
  std::vector<uint8_t> stream(answer.begin(), answer.end());
  CHECK_EQ(runBus(request, reqLen, stream, last), 1);
  CHECK_EQ(last.paramCount, 1);
  CHECK_EQ(last.params[0], 0x42);
}

static void testGeometryMatchesTheHost() {
  CASE("Geometrie: dieselben Zahlen wie host/scan3d/geometry.py");
  // Erzeugt mit to_cartesian() aus dem Pythonpaket. Laufen die beiden
  // Implementierungen auseinander, zeigt die Webseite eine andere Wolke als
  // der Host aus derselben Aufnahme - und niemand merkt es.
  struct Case {
    float distanceMm, alphaDeg, yawDeg;
    MountGeometry mount;
    float x, y, z;
  };
  static const Case cases[] = {
    {2500.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f},
     0.000000f, 0.000000f, 2.500000f},
    {2500.0f, 90.0f, 0.0f, {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f},
     2.500000f, 0.000000f, 0.000000f},
    {2500.0f, 90.0f, 90.0f, {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f},
     0.000000f, 2.500000f, 0.000000f},
    {2500.0f, 200.0f, 37.0f, {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f},
     -0.682874f, -0.514582f, -2.349232f},
    {1500.0f, 45.0f, 120.0f, {-40.5f, 12.0f, 0.0f, 1.0f, 0.0f, 1.0f},
     -0.510080f, 0.883485f, 1.072660f},
    // Die Nulllage, die dieser Aufbau wirklich braucht.
    {1500.0f, 45.0f, 120.0f, {0.0f, 0.0f, 89.0f, 1.0f, 0.0f, 1.0f},
     0.520994f, -0.902388f, 1.079010f},
    {1000.0f, 30.0f, 10.0f, {0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -1.0f},
     -0.492404f, 0.086824f, 0.866025f},
  };
  for (const Case &c : cases) {
    Point3 p = toCartesian(c.distanceMm, c.alphaDeg, c.yawDeg, c.mount);
    CHECK(std::fabs(p.x - c.x) < 1e-4);
    CHECK(std::fabs(p.y - c.y) < 1e-4);
    CHECK(std::fabs(p.z - c.z) < 1e-4);
  }
}

static void testGeometryHalfTurnCoversTheSphere() {
  CASE("Geometrie: 180 Grad Gieren decken die ganze Kugel ab");
  // Der Grund, warum kein Schleifring noetig ist: der radiale Anteil wird
  // negativ, sobald alpha ueber 180 Grad geht - derselbe Gierwinkel liefert
  // also beide Himmelsrichtungen.
  MountGeometry mount;
  Point3 front = toCartesian(1000.0f, 90.0f, 0.0f, mount);
  Point3 back = toCartesian(1000.0f, 270.0f, 0.0f, mount);
  CHECK(std::fabs(front.x - 1.0f) < 1e-4);
  CHECK(std::fabs(back.x + 1.0f) < 1e-4);
  CHECK(std::fabs(front.z) < 1e-4);
  CHECK(std::fabs(back.z) < 1e-4);
}

static void testRangeFilterMatchesTheC1() {
  CASE("Geometrie: Gueltigkeitsfenster des C1");
  RangeFilter range;
  CHECK(!range.accepts(0.0f));      // kein Echo
  CHECK(!range.accepts(100.0f));    // Blindzone
  CHECK(range.accepts(1500.0f));
  CHECK(range.accepts(12000.0f));
  CHECK(!range.accepts(12001.0f));  // hinter der Reichweite
}

static void testMaxFrameSizeCoversEveryFrame() {
  CASE("Protokoll: kMaxFrameSize deckt wirklich jeden Frametyp ab");
  // Jeder Sender legt sich einen Puffer dieser Groesse auf den Stapel. Fehlt
  // hier ein Frametyp, schreibt writeXxxFrame darueber hinaus - und das faellt
  // beim Testen nie auf, sondern erst als seltsamer Absturz auf dem Geraet.
  CHECK(kCapsuleFrameSize <= kMaxFrameSize);
  CHECK(kScanMaxFrameSize <= kMaxFrameSize);
  CHECK(kFaultMaxFrameSize <= kMaxFrameSize);
  CHECK(kHeaderSize + kHelloPayloadSize <= kMaxFrameSize);
  CHECK(kHeaderSize + kStatusPayloadSize <= kMaxFrameSize);

  // Und die Probe aufs Exempel: der laengstmoegliche Fehlertext passt hinein.
  uint8_t buf[kMaxFrameSize];
  std::string longest(kFaultMaxTextLen, 'x');
  size_t n = writeFaultFrame(buf, 0, kFaultServo, longest.c_str());
  CHECK_EQ(n, kFaultMaxFrameSize);
  CHECK(n <= sizeof(buf));
}

static void testFaultTextIsLongEnoughForTheRealMessages() {
  CASE("Protokoll: die echten Meldungen werden nicht abgeschnitten");
  // Bei 96 Zeichen endete die Servo-Meldung mitten im Wort ("... fehlt de"),
  // still und leise. Die Laengen hier stammen aus main.cpp.
  static const char *messages[] = {
    "Kein Servo (STS3215). Der LiDAR laeuft trotzdem, aber im Freilauf: alles "
    "landet in der Ebene bei 0 Grad. Fuer 3D fehlt die Gierachse.",
    "Kein LiDAR am USB-Host-Port. Kabel, Adapter und 5-V-Versorgung pruefen "
    "(der C1 zieht mehr, als der USB-Port allein liefert).",
    "Kein Scanmodus startbar. Versorgung (5 V, >2 W) und Baudrate pruefen "
    "(C1 460800, S2 1 Mbaud).",
    "LiDAR-UART liess sich nicht oeffnen",
    "Speicher fuer die Frame-Queue reicht nicht",
  };
  for (const char *m : messages) CHECK(std::strlen(m) <= kFaultMaxTextLen);
}

static void testFaultFrameCapsTheText() {
  CASE("Protokoll: ueberlanger Fehlertext wird gekappt");
  uint8_t buf[kMaxFrameSize];
  std::string tooLong(300, 'x');
  size_t n = writeFaultFrame(buf, 0, kFaultQueue, tooLong.c_str());
  CHECK_EQ(n, kHeaderSize + 2 + kFaultMaxTextLen);
  // Laengenbyte und Rahmenlaenge muessen zusammenpassen, sonst verwirft der
  // Empfaenger den Frame.
  CHECK_EQ(static_cast<size_t>(buf[kHeaderSize + 1]), kFaultMaxTextLen);
  CHECK_EQ(static_cast<size_t>(buf[6] | (buf[7] << 8)), 2 + kFaultMaxTextLen);
  CHECK(n <= kMaxFrameSize);
}

static void testFaultFrameHandlesEmptyText() {
  CASE("Protokoll: Fehlerframe ohne Text bleibt gueltig");
  uint8_t buf[kMaxFrameSize];
  size_t n = writeFaultFrame(buf, 1, kFaultLidarPort, "");
  CHECK_EQ(n, kHeaderSize + 2u);
  CHECK_EQ(buf[kHeaderSize + 0], kFaultLidarPort);
  CHECK_EQ(buf[kHeaderSize + 1], 0);
}

// --- Einfacher Scanmodus (C1) --------------------------------------------

struct ScanCollector {
  std::vector<ScanSample> samples;
};

static void collectScan(void *ctx, const ScanSample &sample) {
  static_cast<ScanCollector *>(ctx)->samples.push_back(sample);
}

// Eine Messung im Rohformat des C1 bauen: 5 Byte, mit den beiden Pruefbits.
static void makeNode(uint8_t *out, double angleDeg, uint16_t distanceMm,
                     uint8_t quality, bool startFlag) {
  const uint16_t angleQ6 = static_cast<uint16_t>(std::lround(angleDeg * 64.0));
  const uint16_t distanceQ2 = static_cast<uint16_t>(distanceMm * 4);
  out[0] = static_cast<uint8_t>((quality << 2) | (startFlag ? 0x01 : 0x02));
  out[1] = static_cast<uint8_t>(((angleQ6 & 0x7F) << 1) | 0x01);
  out[2] = static_cast<uint8_t>(angleQ6 >> 7);
  out[3] = static_cast<uint8_t>(distanceQ2);
  out[4] = static_cast<uint8_t>(distanceQ2 >> 8);
}

static void testStandardScanHappyPath() {
  CASE("Einfacher Scan: Winkel, Distanz, Umlaufmarke");
  uint8_t stream[15];
  makeNode(stream + 0, 0.0, 1000, 47, true);
  makeNode(stream + 5, 123.25, 2500, 30, false);
  makeNode(stream + 10, 359.75, 12000, 15, false);

  ScanCollector out;
  StandardScanParser parser;
  parser.feed(stream, sizeof(stream), collectScan, &out);

  CHECK_EQ(out.samples.size(), 3u);
  if (out.samples.size() < 3) return;
  CHECK(out.samples[0].newRevolution);
  CHECK(!out.samples[1].newRevolution);
  CHECK_EQ(out.samples[0].distanceMm, 1000);
  CHECK_EQ(out.samples[0].quality, 47);
  CHECK(std::fabs(out.samples[1].angleQ6 / 64.0 - (123.25)) < (1.0 / 64));
  CHECK_EQ(out.samples[1].distanceMm, 2500);
  CHECK(std::fabs(out.samples[2].angleQ6 / 64.0 - (359.75)) < (1.0 / 64));
  CHECK_EQ(out.samples[2].distanceMm, 12000);
  CHECK_EQ(parser.resyncs(), 0u);
}

static void testStandardScanResync() {
  CASE("Einfacher Scan: findet nach Muell wieder ins Raster");
  uint8_t stream[23];
  // Vier Fuellbytes, die als Anfang nicht taugen (S == !S beziehungsweise
  // check == 0), dann drei saubere Messungen.
  stream[0] = 0x00;
  stream[1] = 0xFF;
  stream[2] = 0x00;
  stream[3] = 0xFF;
  makeNode(stream + 4, 10.0, 1100, 20, true);
  makeNode(stream + 9, 20.0, 1200, 20, false);
  makeNode(stream + 14, 30.0, 1300, 20, false);
  stream[19] = 0x00;
  stream[20] = 0x00;
  stream[21] = 0x00;
  stream[22] = 0x00;

  ScanCollector out;
  StandardScanParser parser;
  parser.feed(stream, sizeof(stream), collectScan, &out);

  CHECK_EQ(out.samples.size(), 3u);
  if (out.samples.size() < 3) return;
  CHECK(std::fabs(out.samples[0].angleQ6 / 64.0 - (10.0)) < (1.0 / 64));
  CHECK(std::fabs(out.samples[2].angleQ6 / 64.0 - (30.0)) < (1.0 / 64));
  CHECK(parser.resyncs() > 0u);
}

static void testStandardScanSplitFeeds() {
  CASE("Einfacher Scan: Messung ueber zwei Lesevorgaenge hinweg");
  uint8_t stream[10];
  makeNode(stream + 0, 45.0, 1500, 40, true);
  makeNode(stream + 5, 90.0, 1600, 40, false);

  ScanCollector out;
  StandardScanParser parser;
  // Mitten in der ersten Messung trennen - so kommt es an der UART wirklich an.
  parser.feed(stream, 3, collectScan, &out);
  CHECK_EQ(out.samples.size(), 0u);
  parser.feed(stream + 3, sizeof(stream) - 3, collectScan, &out);
  CHECK_EQ(out.samples.size(), 2u);
  if (out.samples.size() < 2) return;
  CHECK(std::fabs(out.samples[0].angleQ6 / 64.0 - (45.0)) < (1.0 / 64));
  CHECK(std::fabs(out.samples[1].angleQ6 / 64.0 - (90.0)) < (1.0 / 64));
}

static void testStandardScanRoundsQuarterMillimetres() {
  CASE("Einfacher Scan: Viertelmillimeter werden gerundet, nicht abgeschnitten");
  uint8_t node[5];
  makeNode(node, 0.0, 1000, 10, true);
  // distance_q2 von 4000 auf 4002 anheben: 1000.5 mm, muss auf 1001 runden.
  node[3] = static_cast<uint8_t>(4002 & 0xFF);
  node[4] = static_cast<uint8_t>(4002 >> 8);

  ScanCollector out;
  StandardScanParser parser;
  parser.feed(node, sizeof(node), collectScan, &out);
  CHECK_EQ(out.samples.size(), 1u);
  if (out.samples.empty()) return;
  CHECK_EQ(out.samples[0].distanceMm, 1001);
}

int main(int argc, char **argv) {
  const char *fixture = (argc > 1) ? argv[1] : "../../../tests/wire_fixture.txt";

  testParserHappyPath();
  testParserStartFlag();
  testParserSplitFeeds();
  testParserLeadingGarbage();
  testParserBadChecksumRecovers();
  testParserTimestamps();
  testDecoderInterpolation();
  testDecoderRevolutionAligned();
  testDecoderRevolutionUnaligned();
  testAngleUtil();
  testSweepPlan();
  testSweepPlanDegenerate();
  testFeetechPacketLayout();
  testFeetechMovePacket();
  testFeetechSignedValues();
  testFeetechStatusParser();
  testFeetechStatusParserRobustness();
  testStandardScanHappyPath();
  testStandardScanResync();
  testStandardScanSplitFeeds();
  testStandardScanRoundsQuarterMillimetres();
  testEchoAloneIsNotAnAnswer();
  testEchoIsSkippedAndTheAnswerArrives();
  testWithoutEchoNothingIsSwallowed();
  testEchoFilterSurvivesAPartialMatch();
  testGeometryMatchesTheHost();
  testGeometryHalfTurnCoversTheSphere();
  testRangeFilterMatchesTheC1();
  testMaxFrameSizeCoversEveryFrame();
  testFaultTextIsLongEnoughForTheRealMessages();
  testFaultFrameCapsTheText();
  testFaultFrameHandlesEmptyText();
  testWireFormat(fixture);

  std::printf("\n%d Pruefungen, %d Fehler\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
