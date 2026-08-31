// Dieselben Zusicherungen wie host/tests/test_geometry.py. Die Geometrie ist
// die Stelle, an der eine Punktwolke lautlos falsch wird - deshalb doppelt.

import XCTest
import simd
@testable import LidarKit

final class GeometryTests: XCTestCase {

    private func assertClose(_ a: SIMD3<Float>, _ b: SIMD3<Float>,
                             accuracy: Float = 1e-5,
                             file: StaticString = #filePath, line: UInt = #line) {
        XCTAssertEqual(a.x, b.x, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(a.y, b.y, accuracy: accuracy, file: file, line: line)
        XCTAssertEqual(a.z, b.z, accuracy: accuracy, file: file, line: line)
    }

    func testStraightUpIsPlusZ() {
        assertClose(toCartesian(distanceMm: 1000, alphaDeg: 0, yawDeg: 0),
                    SIMD3<Float>(0, 0, 1))
    }

    func testStraightDownIsMinusZ() {
        let p = toCartesian(distanceMm: 1000, alphaDeg: 180, yawDeg: 123)
        XCTAssertEqual(p.z, -1, accuracy: 1e-5)
    }

    func testHorizontalFollowsYaw() {
        assertClose(toCartesian(distanceMm: 2000, alphaDeg: 90, yawDeg: 0),
                    SIMD3<Float>(2, 0, 0))
        assertClose(toCartesian(distanceMm: 2000, alphaDeg: 90, yawDeg: 90),
                    SIMD3<Float>(0, 2, 0))
    }

    /// Der Kernpunkt des Aufbaus: 180 Grad Gieren decken die volle Kugel ab.
    /// alpha=270 bei yaw=0 zeigt dorthin wie alpha=90 bei yaw=180.
    func test180DegreeYawCoversTheBackHalf() {
        assertClose(toCartesian(distanceMm: 1000, alphaDeg: 270, yawDeg: 0),
                    toCartesian(distanceMm: 1000, alphaDeg: 90, yawDeg: 180))
    }

    func testRadialOffsetShiftsBeforeRotation() {
        let mount = MountGeometry(offsetRadialMm: 50)
        assertClose(toCartesian(distanceMm: 1000, alphaDeg: 90, yawDeg: 0, mount: mount),
                    SIMD3<Float>(1.05, 0, 0))
    }

    func testAxialOffsetShiftsZ() {
        let mount = MountGeometry(offsetAxialMm: -25)
        let p = toCartesian(distanceMm: 1000, alphaDeg: 0, yawDeg: 0, mount: mount)
        XCTAssertEqual(p.z, 0.975, accuracy: 1e-5)
    }

    func testAlphaSignMirrorsTheScanPlane() {
        let mount = MountGeometry(alphaSign: -1)
        let p = toCartesian(distanceMm: 1000, alphaDeg: 90, yawDeg: 0, mount: mount)
        XCTAssertEqual(p.x, -1, accuracy: 1e-5)
    }

    func testFlatWallStaysFlat() {
        for alpha in [Float(60), 75, 90, 105, 120] {
            let r = 3000 / sin(alpha * .pi / 180)
            let p = toCartesian(distanceMm: r, alphaDeg: alpha, yawDeg: 0)
            XCTAssertEqual(p.x, 3, accuracy: 1e-4)
        }
    }

    /// Standardfall: RPLIDAR C1 mit 5000 Messungen/s bei 10 Hz.
    func testSweepPlan() {
        let plan = SweepPlan(yawSpanDeg: 180, yawStepDeg: 1)
        XCTAssertEqual(plan.planes, 180)
        // Je Ebene eine Umdrehung (100 ms) plus Fahrt und Einrasten (50 ms).
        XCTAssertEqual(plan.secondsPerPlane, 0.15, accuracy: 1e-5)
        XCTAssertEqual(plan.durationSeconds, 27, accuracy: 1e-3)
        XCTAssertEqual(plan.samplesPerPlane, 500, accuracy: 1e-4)
        XCTAssertEqual(plan.totalSamples, 90000, accuracy: 1)
        XCTAssertEqual(plan.inPlaneResolutionDeg, 0.72, accuracy: 1e-6)
    }

    /// Zum Vergleich der S2: gleiche Dauer, aber 6,4-fache Punktzahl. Die
    /// Sweep-Dauer haengt an der Scanrate, nicht an der Messrate - beide
    /// Geraete drehen mit 10 Hz.
    func testSweepPlanForTheS2() {
        let plan = SweepPlan(yawSpanDeg: 180, yawStepDeg: 1, samplesPerSecond: 32000)
        XCTAssertEqual(plan.durationSeconds, 27, accuracy: 1e-3)
        XCTAssertEqual(plan.samplesPerPlane, 3200, accuracy: 1e-4)
        XCTAssertEqual(plan.totalSamples, 576000, accuracy: 1)
        XCTAssertEqual(plan.inPlaneResolutionDeg, 0.1125, accuracy: 1e-6)
    }

    func testSweepPlanWithoutOverheadIsPureMeasurementTime() {
        let plan = SweepPlan(yawSpanDeg: 180, yawStepDeg: 1, planeOverheadSeconds: 0)
        XCTAssertEqual(plan.durationSeconds, 18, accuracy: 1e-3)
    }

    func testRangeFilterRejectsBlindZoneAndDropouts() {
        let filter = RangeFilter()
        XCTAssertFalse(filter.accepts(0))     // kein Echo
        XCTAssertFalse(filter.accepts(100))   // in der Blindzone
        XCTAssertFalse(filter.accepts(12001))  // hinter der Reichweite des C1
        XCTAssertTrue(filter.accepts(1500))
    }

    func testRangeFilterCanBeWidenedForTheS2() {
        let filter = RangeFilter(minMm: 150, maxMm: 30000)
        XCTAssertTrue(filter.accepts(25000))
        XCTAssertFalse(filter.accepts(30001))
    }
}

final class PointCloudBufferTests: XCTestCase {

    private func capsule(sweepActive: Bool, distance: UInt16 = 1000) -> CapsuleFrame {
        var payload = [UInt8](repeating: 0, count: CapsuleFrame.payloadSize)
        // alpha_inc_q16 = 0.1125 deg, Distanzen konstant
        let inc = Int32(0.1125 * 65536)
        payload[8] = UInt8(truncatingIfNeeded: inc)
        payload[9] = UInt8(truncatingIfNeeded: inc >> 8)
        for i in 0 ..< CapsuleFrame.cabinCount {
            payload[16 + 2 * i] = UInt8(truncatingIfNeeded: distance)
            payload[17 + 2 * i] = UInt8(truncatingIfNeeded: distance >> 8)
        }
        let flags: FrameFlags = sweepActive ? .sweepActive : []
        let frame = Frame(type: FrameType.capsule.rawValue, flags: flags,
                          seq: 0, payload: payload)
        return CapsuleFrame(frame)!
    }

    func testIgnoresCapsulesOutsideTheSweep() {
        let buffer = PointCloudBuffer()
        buffer.append(capsule(sweepActive: false))
        XCTAssertEqual(buffer.drain().points.count, 0)
    }

    func testCollectsSweepPoints() {
        let buffer = PointCloudBuffer()
        buffer.append(capsule(sweepActive: true))
        buffer.append(capsule(sweepActive: true))
        let drained = buffer.drain()
        XCTAssertEqual(drained.points.count, 2 * CapsuleFrame.cabinCount)
        XCTAssertEqual(buffer.drain().points.count, 0, "drain leert die Zwischenablage")
    }

    func testNewSweepStartsAFreshCloud() {
        let buffer = PointCloudBuffer()
        buffer.append(capsule(sweepActive: true))
        let first = buffer.drain().generation

        buffer.append(capsule(sweepActive: false))  // Sweep zu Ende
        buffer.append(capsule(sweepActive: true))   // neuer Sweep
        let second = buffer.drain()
        XCTAssertGreaterThan(second.generation, first)
        XCTAssertEqual(second.points.count, CapsuleFrame.cabinCount)
    }

    func testDropsMeasurementsInTheBlindZone() {
        let buffer = PointCloudBuffer()
        buffer.append(capsule(sweepActive: true, distance: 100))
        XCTAssertEqual(buffer.drain().points.count, 0)
        XCTAssertEqual(buffer.statistics.rejected, CapsuleFrame.cabinCount)
    }
}
