// Sammelstelle zwischen Netz und Darstellung.
//
// Der Netz-Thread schiebt Punkte hinein, der Renderer holt sie einmal je Bild
// heraus. Das haelt die Konvertierung aus dem Renderloop und braucht keine
// GPU-Puffer im Netzcode.
//
// 32000 Punkte/s sind fuer die CPU nichts; die Umrechnung darf ruhig hier
// passieren statt im Shader.

import Foundation
import simd

public final class PointCloudBuffer: @unchecked Sendable {
    private let lock = NSLock()
    private var staging: [SIMD3<Float>] = []
    private var accepted = 0
    private var rejected = 0
    private var sweepGeneration = 0
    private var lastSweepActive = false

    /// Obergrenze fuer eine Wolke. 180 Grad bei 1 Grad Ebenenabstand ergeben
    /// 576k Punkte, bei 0.5 Grad 1.15 Mio - 2 Mio decken beides ab.
    public let capacity: Int

    public var mount: MountGeometry {
        get { lock.lock(); defer { lock.unlock() }; return _mount }
        set { lock.lock(); _mount = newValue; lock.unlock() }
    }
    private var _mount = MountGeometry.identity

    public var range: RangeFilter {
        get { lock.lock(); defer { lock.unlock() }; return _range }
        set { lock.lock(); _range = newValue; lock.unlock() }
    }
    private var _range = RangeFilter()

    public init(capacity: Int = 2_000_000) {
        self.capacity = capacity
        staging.reserveCapacity(64 * CapsuleFrame.cabinCount)
    }

    /// Vom Netz-Thread aufgerufen. Threadsicher.
    public func append(_ capsule: CapsuleFrame) {
        lock.lock()
        defer { lock.unlock() }

        // Ein neuer Sweep verwirft die alte Wolke. Sauberer als ein Ringpuffer,
        // und der Renderer bekommt nie halb ueberschriebene Daten zu sehen.
        if capsule.sweepActive && !lastSweepActive {
            sweepGeneration += 1
            staging.removeAll(keepingCapacity: true)
            accepted = 0
            rejected = 0
        }
        lastSweepActive = capsule.sweepActive
        guard capsule.sweepActive, accepted < capacity else { return }

        let mount = _mount
        let range = _range
        capsule.forEachSample { dist, alpha, yaw in
            guard range.accepts(dist) else {
                rejected += 1
                return
            }
            staging.append(toCartesian(distanceMm: dist, alphaDeg: alpha,
                                       yawDeg: yaw, mount: mount))
            accepted += 1
        }
    }

    /// Vom Renderer je Bild aufgerufen. Gibt die seither eingetroffenen Punkte
    /// zurueck und leert die Zwischenablage.
    public func drain() -> (points: [SIMD3<Float>], generation: Int) {
        lock.lock()
        defer { lock.unlock() }
        let out = staging
        staging.removeAll(keepingCapacity: true)
        return (out, sweepGeneration)
    }

    public func clear() {
        lock.lock()
        defer { lock.unlock() }
        staging.removeAll(keepingCapacity: true)
        accepted = 0
        rejected = 0
        sweepGeneration += 1
    }

    public var statistics: (accepted: Int, rejected: Int, generation: Int) {
        lock.lock()
        defer { lock.unlock() }
        return (accepted, rejected, sweepGeneration)
    }
}
