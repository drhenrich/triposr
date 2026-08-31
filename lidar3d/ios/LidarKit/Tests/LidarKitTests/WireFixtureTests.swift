// Prueft den Swift-Decoder gegen dieselbe Byte-Fixture wie Firmware und
// Python-Host: lidar3d/tests/wire_fixture.txt.
//
// Damit sind alle drei Implementierungen des Protokolls aneinander gebunden.
// Aendert jemand nur eine, schlaegt hier ein Test fehl.

import XCTest
@testable import LidarKit

final class WireFixtureTests: XCTestCase {

    /// Die Fixture liegt ausserhalb des Pakets, deshalb ueber #filePath statt
    /// ueber Bundle-Ressourcen - so bleibt sie die einzige Quelle der Wahrheit.
    static func loadFixture() throws -> [String: [UInt8]] {
        // .../lidar3d/ios/LidarKit/Tests/LidarKitTests/WireFixtureTests.swift
        let root = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()  // LidarKitTests
            .deletingLastPathComponent()  // Tests
            .deletingLastPathComponent()  // LidarKit
            .deletingLastPathComponent()  // ios
            .deletingLastPathComponent()  // lidar3d
        let url = root.appendingPathComponent("tests/wire_fixture.txt")
        let text = try String(contentsOf: url, encoding: .utf8)

        var out: [String: [UInt8]] = [:]
        for line in text.split(separator: "\n") {
            let trimmed = line.trimmingCharacters(in: .whitespaces)
            if trimmed.isEmpty || trimmed.hasPrefix("#") { continue }
            let parts = trimmed.split(separator: " ")
            guard parts.count == 2 else { continue }
            out[String(parts[0])] = hexToBytes(String(parts[1]))
        }
        return out
    }

    static func hexToBytes(_ hex: String) -> [UInt8] {
        var bytes: [UInt8] = []
        bytes.reserveCapacity(hex.count / 2)
        var index = hex.startIndex
        while index < hex.endIndex {
            let next = hex.index(index, offsetBy: 2)
            bytes.append(UInt8(hex[index ..< next], radix: 16) ?? 0)
            index = next
        }
        return bytes
    }

    func testFixtureLoads() throws {
        let fixture = try Self.loadFixture()
        XCTAssertEqual(Set(fixture.keys), ["scan", "capsule", "hello", "status"])
        XCTAssertEqual(fixture["capsule"]?.count, 104)
        // 8 Header + 12 Kopf + 8 Messungen a 4 Byte
        XCTAssertEqual(fixture["scan"]?.count, 52)
    }

    func testDecodesScan() throws {
        let fixture = try Self.loadFixture()
        var parser = FrameParser()
        let frames = parser.feed(fixture["scan"]!)
        XCTAssertEqual(frames.count, 1)

        let scan = try XCTUnwrap(ScanFrame(frames[0]))
        XCTAssertEqual(scan.seq, 4)
        XCTAssertTrue(scan.newRevolution)
        XCTAssertTrue(scan.sweepActive)
        XCTAssertEqual(scan.yawStartDeg, 42.0, accuracy: 1.0 / 65536)
        XCTAssertEqual(scan.yawEndDeg, 42.5, accuracy: 1.0 / 65536)
        XCTAssertEqual(scan.distancesMm.count, 8)
        XCTAssertEqual(scan.distancesMm.first, 1000)
        XCTAssertEqual(scan.distancesMm.last, 1035)
        // Winkel sind Q6, quantisieren also auf 1/64 Grad.
        XCTAssertEqual(scan.anglesDeg[0], 0.0, accuracy: 1.0 / 64)
        XCTAssertEqual(scan.anglesDeg[1], 0.72, accuracy: 1.0 / 64)
        XCTAssertEqual(scan.anglesDeg[7], 5.14, accuracy: 1.0 / 64)
    }

    func testScanInterpolatesOnlyTheYaw() throws {
        let fixture = try Self.loadFixture()
        var parser = FrameParser()
        let scan = try XCTUnwrap(ScanFrame(parser.feed(fixture["scan"]!)[0]))

        var yaws: [Float] = []
        var alphas: [Float] = []
        scan.forEachSample { _, alpha, yaw in
            alphas.append(alpha)
            yaws.append(yaw)
        }
        // Gierwinkel gleichmaessig ueber den Frame ...
        XCTAssertEqual(yaws.first!, 42.0, accuracy: 1e-3)
        XCTAssertEqual(yaws[4], 42.25, accuracy: 1e-3)
        // ... Scanwinkel dagegen unveraendert aus den Bytes.
        XCTAssertEqual(alphas, scan.anglesDeg)
    }

    func testRejectsScanWithMismatchedCount() throws {
        let fixture = try Self.loadFixture()
        var bytes = fixture["scan"]!
        // count auf 9 stellen, ohne Bytes anzuhaengen: muss abgelehnt werden.
        bytes[8 + 8] = 9
        var parser = FrameParser()
        XCTAssertNil(ScanFrame(parser.feed(bytes)[0]))
    }

    func testDecodesCapsule() throws {
        let fixture = try Self.loadFixture()
        var parser = FrameParser()
        let frames = parser.feed(fixture["capsule"]!)
        XCTAssertEqual(frames.count, 1)

        let capsule = try XCTUnwrap(CapsuleFrame(frames[0]))
        XCTAssertEqual(capsule.seq, 0x1234)
        XCTAssertTrue(capsule.sweepActive)
        XCTAssertTrue(capsule.flags.contains(.newRevolution))
        XCTAssertEqual(capsule.yawStartDeg, 42.5, accuracy: 1.0 / 65536)
        XCTAssertEqual(capsule.yawEndDeg, 42.5125, accuracy: 1.0 / 65536)
        XCTAssertEqual(capsule.alphaStartDeg, 123.25, accuracy: 1.0 / 64)
        XCTAssertEqual(capsule.alphaIncDeg, 0.1125, accuracy: 1.0 / 65536)
        XCTAssertEqual(capsule.distancesMm.count, 40)
        XCTAssertEqual(capsule.distancesMm.first, 1000)
        XCTAssertEqual(capsule.distancesMm.last, 1000 + 7 * 39)
    }

    func testDecodesHello() throws {
        let fixture = try Self.loadFixture()
        var parser = FrameParser()
        let hello = try XCTUnwrap(HelloFrame(parser.feed(fixture["hello"]!)[0]))
        XCTAssertEqual(hello.fwVersion, 1)
        XCTAssertEqual(hello.lidarRpm, 600)
        XCTAssertEqual(hello.offsetRadialMm, -40.5, accuracy: 1e-3)
        XCTAssertEqual(hello.offsetAxialMm, 12.0, accuracy: 1e-3)
        XCTAssertEqual(hello.yawMaxDeg, 180.0, accuracy: 1e-3)
    }

    func testDecodesStatus() throws {
        let fixture = try Self.loadFixture()
        var parser = FrameParser()
        let status = try XCTUnwrap(StatusFrame(parser.feed(fixture["status"]!)[0]))
        XCTAssertEqual(status.sweepIndex, 3)
        XCTAssertEqual(status.state, .sweeping)
        XCTAssertEqual(status.yawDeg, 90.5, accuracy: 1e-3)
        XCTAssertEqual(status.capsules, 123456)
        XCTAssertEqual(status.checksumErrors, 7)
        XCTAssertEqual(status.droppedFrames, 2)
    }

    /// Der Strom kommt in TCP-Haeppchen an, nicht in Frames.
    func testSplitAndConcatenatedDelivery() throws {
        let fixture = try Self.loadFixture()
        let blob = fixture["hello"]! + fixture["capsule"]! + fixture["status"]!

        var parser = FrameParser()
        var frames: [Frame] = []
        var index = 0
        // Absichtlich krumme Haeppchengroessen, quer ueber die Framegrenzen.
        for size in [5, 60, 3, 1, 100, 90, 200] {
            guard index < blob.count else { break }
            let end = min(index + size, blob.count)
            frames += parser.feed(Array(blob[index ..< end]))
            index = end
        }
        frames += parser.feed(Array(blob[index...]))

        XCTAssertEqual(frames.map(\.type), [0, 1, 2])
        XCTAssertEqual(parser.resyncs, 0)
    }

    func testResyncsAfterGarbage() throws {
        let fixture = try Self.loadFixture()
        var parser = FrameParser()
        let frames = parser.feed([0xDE, 0xAD, 0xBE, 0xEF] + fixture["capsule"]!)
        XCTAssertEqual(frames.count, 1)
        XCTAssertEqual(parser.resyncs, 4)
    }

    func testRejectsWrongFrameType() throws {
        let fixture = try Self.loadFixture()
        var parser = FrameParser()
        let statusFrame = parser.feed(fixture["status"]!)[0]
        XCTAssertNil(CapsuleFrame(statusFrame))
        XCTAssertNil(HelloFrame(statusFrame))
    }
}
