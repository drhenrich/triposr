// Bindeglied zwischen LidarKit und der Oberflaeche.
//
// Messdaten laufen am ViewModel vorbei direkt in den PointCloudBuffer - bei
// 160 Frames/s (C1) bis 800 (S2) hat der Mainthread Besseres zu tun als
// SwiftUI-Updates.
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
    /// Gesetzt, wenn der Scanner meldet, dass sein Hochlauf gescheitert ist.
    @Published private(set) var fault: FaultFrame?
    @Published private(set) var framesPerSecond: Double = 0
    @Published var pointSize: Float = 3.0

    /// Wird vom Renderer je Bild gesetzt, damit die Anzeige die echte Punktzahl zeigt.
    @Published var visiblePoints: Int = 0

    let cloud = PointCloudBuffer()
    private let client = ScannerClient()
    private var lastFrameCount: UInt32 = 0
    private var lastFrameSampleTime = Date()

    init() {
        // Beide Datenwege - Dense-Capsules vom S2, Scanframes vom C1. Welcher
        // kommt, entscheidet die Firmware; die Wolke sieht in beiden Faellen
        // dieselben Messungen.
        client.onCapsule = { [cloud] capsule in
            // Hintergrundqueue - PointCloudBuffer ist threadsicher.
            cloud.append(capsule)
        }
        client.onScan = { [cloud] scan in
            cloud.append(scan)
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
        client.onFault = { [weak self] fault in
            MainActor.assumeIsolated { self?.fault = fault }
        }
    }

    private func apply(_ status: StatusFrame) {
        let now = Date()
        let elapsed = now.timeIntervalSince(lastFrameSampleTime)
        if elapsed > 0.4 {
            let delta = status.capsules >= lastFrameCount ? status.capsules - lastFrameCount : 0
            framesPerSecond = Double(delta) / elapsed
            lastFrameCount = status.capsules
            lastFrameSampleTime = now
        }
        self.status = status
    }

    // MARK: - Steuerung

    func connect() {
        fault = nil
        lastFrameCount = 0
        lastFrameSampleTime = Date()
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

    /// Was der Scanner meldet, wenn er zwar antwortet, aber nicht scannen kann.
    var faultLabel: String? {
        guard let fault, fault.code != .ok else { return nil }
        return fault.text
    }

    var stateLabel: String {
        if fault != nil { return "gestoert" }
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
