// Bindeglied zwischen LidarKit und der Oberflaeche.
//
// Capsules laufen am ViewModel vorbei direkt in den PointCloudBuffer - bei
// 800 Capsules/s hat der Mainthread Besseres zu tun als SwiftUI-Updates.
// Hier landen nur Status und Verbindungszustand, also ein paar Aktualisierungen
// je Sekunde.

import Combine
import Foundation
import LidarKit

@MainActor
final class ScannerViewModel: ObservableObject {

    @Published private(set) var connection: ScannerConnectionState = .idle
    @Published private(set) var hello: HelloFrame?
    @Published private(set) var status: StatusFrame?
    @Published private(set) var capsulesPerSecond: Double = 0
    @Published var pointSize: Float = 3.0

    /// Wird vom Renderer je Bild gesetzt, damit die Anzeige die echte Punktzahl zeigt.
    @Published var visiblePoints: Int = 0

    let cloud = PointCloudBuffer()
    private let client = ScannerClient()
    private var lastCapsules: UInt32 = 0
    private var lastCapsuleSampleTime = Date()

    init() {
        client.onCapsule = { [cloud] capsule in
            // Hintergrundqueue - PointCloudBuffer ist threadsicher.
            cloud.append(capsule)
        }
        // onState/onHello/onStatus liefert ScannerClient bereits auf dem
        // Mainthread; assumeIsolated sagt das dem Compiler, statt es ihn raten
        // zu lassen.
        client.onState = { [weak self] state in
            MainActor.assumeIsolated { self?.connection = state }
        }
        client.onHello = { [weak self] hello in
            MainActor.assumeIsolated {
                guard let self else { return }
                self.hello = hello
                // Die Einbaulage kommt vom Scanner, damit sie nicht doppelt
                // gepflegt werden muss.
                var mount = self.cloud.mount
                mount.offsetRadialMm = hello.offsetRadialMm
                mount.offsetAxialMm = hello.offsetAxialMm
                self.cloud.mount = mount
            }
        }
        client.onStatus = { [weak self] status in
            MainActor.assumeIsolated { self?.apply(status) }
        }
    }

    private func apply(_ status: StatusFrame) {
        let now = Date()
        let elapsed = now.timeIntervalSince(lastCapsuleSampleTime)
        if elapsed > 0.4 {
            let delta = status.capsules >= lastCapsules ? status.capsules - lastCapsules : 0
            capsulesPerSecond = Double(delta) / elapsed
            lastCapsules = status.capsules
            lastCapsuleSampleTime = now
        }
        self.status = status
    }

    // MARK: - Steuerung

    func connect() {
        lastCapsules = 0
        lastCapsuleSampleTime = Date()
        client.connect()
    }

    func disconnect() {
        client.disconnect()
    }

    func startSweep() {
        cloud.clear()
        client.startSweep()
    }

    func abortSweep() {
        client.abortSweep()
    }

    // MARK: - Anzeige

    var isConnected: Bool {
        if case .connected = connection { return true }
        return false
    }

    var transportLabel: String {
        switch connection {
        case .connected(let label): return label
        case .connecting(let label): return "\(label) …"
        case .failed(let reason): return reason
        case .idle: return "getrennt"
        }
    }

    var stateLabel: String {
        switch status?.state {
        case .homing: return "referenzieren"
        case .sweeping: return "Sweep laeuft"
        case .returning: return "Ruecklauf"
        case .idle, nil: return "bereit"
        }
    }

    /// Warnzeichen: die Wolke hat Luecken, weil der Transport nicht mitkam.
    var hasDrops: Bool { (status?.droppedFrames ?? 0) > 0 }
}
