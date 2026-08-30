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

#include "../../src/dense_capsule.h"
#include "../../src/stream_proto.h"
#include "../../src/yaw_model.h"

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

static void testYawModel() {
  CASE("Gier: Festkommamathematik");
  // NEMA17, 1/16 Microstepping, 3:1 Riemen -> 9600 Schritte je Umdrehung
  double dps = degreesPerStep(200, 16, 3.0);
  CHECK(std::fabs(dps - 0.0375) < 1e-9);

  double hz = stepHzForRate(10.0, dps);
  CHECK(std::fabs(hz - 266.666666) < 1e-4);

  YawModel model;
  model.start(1000000, degToQ16(0.0), degPerUsQ32(dps, hz), +1);

  // Nach 18 s muessen 180 deg erreicht sein (auf 0.01 deg genau).
  double after18s = q16ToDeg(model.atQ16(1000000 + 18000000));
  CHECK(std::fabs(after18s - 180.0) < 0.01);

  double after1ms = q16ToDeg(model.atQ16(1000000 + 1000));
  CHECK(std::fabs(after1ms - 0.01) < 1e-4);

  int64_t tEnd = model.timeForQ16(degToQ16(180.0));
  CHECK(std::llabs(tEnd - (1000000 + 18000000)) < 2000);
}

static void testYawModelReverse() {
  CASE("Gier: Rueckwaertsfahrt");
  double dps = degreesPerStep(200, 16, 3.0);
  YawModel model;
  model.start(0, degToQ16(180.0), degPerUsQ32(dps, stepHzForRate(60.0, dps)), -1);
  double after1s = q16ToDeg(model.atQ16(1000000));
  CHECK(std::fabs(after1s - 120.0) < 0.01);
  CHECK(std::llabs(model.timeForQ16(degToQ16(0.0)) - 3000000) < 3000);
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
  testYawModel();
  testYawModelReverse();
  testWireFormat(fixture);

  std::printf("\n%d Pruefungen, %d Fehler\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
