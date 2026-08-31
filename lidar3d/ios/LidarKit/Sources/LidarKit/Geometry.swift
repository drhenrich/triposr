// Kugelkoordinaten -> kartesische Punkte. Spiegelt host/scan3d/geometry.py.
//
// Der LiDAR liefert den Polarwinkel, der Schrittmotor den Azimut, also ist
// eine Messung direkt eine Kugelkoordinate. Herleitung und Kalibrierung
// stehen in docs/02-geometrie.md.

import Foundation
import simd

public struct MountGeometry: Sendable, Equatable {
    /// Abstand des optischen Zentrums von der Gierachse, senkrecht dazu, in mm.
    public var offsetRadialMm: Float
    /// Hoehe des optischen Zentrums entlang der Gierachse, in mm.
    public var offsetAxialMm: Float
    /// LiDAR-Winkel, der nach oben (+Z) zeigt.
    public var alphaZeroDeg: Float
    /// -1, wenn der LiDAR-Winkel entgegen der gewuenschten Richtung laeuft.
    public var alphaSign: Float
    /// Gierwinkel, der als Azimut 0 gilt.
    public var yawZeroDeg: Float
    public var yawSign: Float

    public init(offsetRadialMm: Float = 0,
                offsetAxialMm: Float = 0,
                alphaZeroDeg: Float = 0,
                alphaSign: Float = 1,
                yawZeroDeg: Float = 0,
                yawSign: Float = 1) {
        self.offsetRadialMm = offsetRadialMm
        self.offsetAxialMm = offsetAxialMm
        self.alphaZeroDeg = alphaZeroDeg
        self.alphaSign = alphaSign
        self.yawZeroDeg = yawZeroDeg
        self.yawSign = yawSign
    }

    public static let identity = MountGeometry()
}

public struct RangeFilter: Sendable, Equatable {
    public var minMm: Float
    public var maxMm: Float

    /// Untergrenze oberhalb der Blindzone (Datenblatt: 50 mm) plus Reserve.
    /// Obergrenze ist die Reichweite des C1 (12 m); der S2 kaeme auf 30 m.
    public init(minMm: Float = 150, maxMm: Float = 12000) {
        self.minMm = minMm
        self.maxMm = maxMm
    }

    @inline(__always)
    public func accepts(_ distanceMm: Float) -> Bool {
        distanceMm > 0 && distanceMm >= minMm && distanceMm <= maxMm
    }
}

private let degToRad = Float.pi / 180

/// Eine Messung in Meter-Weltkoordinaten. Z ist die Gierachse und zeigt nach oben.
@inline(__always)
public func toCartesian(distanceMm: Float,
                        alphaDeg: Float,
                        yawDeg: Float,
                        mount: MountGeometry = .identity) -> SIMD3<Float> {
    let a = mount.alphaSign * (alphaDeg - mount.alphaZeroDeg) * degToRad
    // Koordinaten in der Scanebene: u radial von der Achse weg, w entlang der Achse.
    // u darf negativ werden - genau deshalb genuegen 180 Grad Gieren.
    let u = mount.offsetRadialMm + distanceMm * sin(a)
    let w = mount.offsetAxialMm + distanceMm * cos(a)

    let psi = mount.yawSign * (yawDeg - mount.yawZeroDeg) * degToRad
    return SIMD3<Float>(u * cos(psi) / 1000, u * sin(psi) / 1000, w / 1000)
}

/// Eine Capsule in Punkte umrechnen und dabei ungueltige Messungen aussortieren.
public func project(capsule: CapsuleFrame,
                    mount: MountGeometry = .identity,
                    range: RangeFilter = RangeFilter(),
                    into points: inout [SIMD3<Float>]) {
    capsule.forEachSample { dist, alpha, yaw in
        guard range.accepts(dist) else { return }
        points.append(toCartesian(distanceMm: dist, alphaDeg: alpha, yawDeg: yaw, mount: mount))
    }
}

/// Dasselbe fuer den einfachen Scanmodus des C1.
public func project(scan: ScanFrame,
                    mount: MountGeometry = .identity,
                    range: RangeFilter = RangeFilter(),
                    into points: inout [SIMD3<Float>]) {
    scan.forEachSample { dist, alpha, yaw in
        guard range.accepts(dist) else { return }
        points.append(toCartesian(distanceMm: dist, alphaDeg: alpha, yawDeg: yaw, mount: mount))
    }
}

/// Dauer, Punktzahl und Aufloesung eines Sweeps vorausrechnen.
///
/// Der Scanner arbeitet Schritt fuer Schritt: je Ebene anfahren, einrasten,
/// genau eine LiDAR-Umdrehung erfassen. Die Dauer ist deshalb die Umdrehung
/// (1/scanHz) plus Zuschlag fuer Fahrt und Einrasten.
public struct SweepPlan: Sendable {
    public let planes: Float
    public let durationSeconds: Float
    public let secondsPerPlane: Float
    public let samplesPerPlane: Float
    public let totalSamples: Float
    public let inPlaneResolutionDeg: Float

    /// Vorbelegt mit den Werten des C1: 5000 Messungen/s bei 10 Hz, also rund
    /// 500 Punkte je Ebene und 0,72 Grad in der Ebene. Der S2 kaeme mit 32000
    /// auf 0,1125 Grad.
    public init(yawSpanDeg: Float = 180,
                yawStepDeg: Float = 1,
                scanHz: Float = 10,
                samplesPerSecond: Float = 5000,
                planeOverheadSeconds: Float = 0.05) {
        planes = yawSpanDeg / yawStepDeg
        secondsPerPlane = 1 / scanHz + planeOverheadSeconds
        durationSeconds = planes * secondsPerPlane
        samplesPerPlane = samplesPerSecond / scanHz
        totalSamples = planes * samplesPerPlane
        inPlaneResolutionDeg = 360 / samplesPerPlane
    }
}
