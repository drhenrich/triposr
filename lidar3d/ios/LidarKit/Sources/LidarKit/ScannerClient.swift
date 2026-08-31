// TCP-Verbindung zum Scanner.
//
// Transportneutral: ueber das USB-C-Kabel (der ESP32-S3 meldet sich als
// USB-Ethernet, CDC-NCM) und ueber WLAN laeuft derselbe Stream. Die App
// probiert beide Adressen der Reihe nach durch - siehe docs/04-ios-usb.md.
//
// WICHTIG: ohne NSLocalNetworkUsageDescription in der Info.plist blockiert
// iOS die Verbindung stillschweigend.

import Foundation
import Network

public struct ScannerAddress: Sendable, Equatable {
    public let host: String
    public let port: UInt16
    public let label: String

    public init(host: String, port: UInt16 = 5005, label: String) {
        self.host = host
        self.port = port
        self.label = label
    }

    /// ESP32-S3 als USB-Ethernet am Kabel.
    public static let usb = ScannerAddress(host: "192.168.7.1", label: "USB-C")
    /// ESP32-S3 als WLAN-Accesspoint.
    public static let wifi = ScannerAddress(host: "192.168.4.1", label: "WLAN")
    /// Kabel zuerst - es ist schneller und das iPhone behaelt sein WLAN.
    public static let defaults: [ScannerAddress] = [.usb, .wifi]
}

public enum ScannerConnectionState: Equatable, Sendable {
    case idle
    case connecting(String)
    case connected(String)
    case failed(String)
}

/// Verbindet sich, dekodiert den Strom und reicht Messdaten weiter.
///
/// `onCapsule` und `onScan` werden auf einer internen Hintergrundqueue
/// aufgerufen, nicht auf dem Mainthread - bei 160 Frames/s (C1) bis 800 (S2)
/// hat der Mainthread Besseres zu tun.
/// Der Empfaenger muss threadsicher sein (`PointCloudBuffer` ist es).
/// `onState`, `onHello` und `onStatus` kommen dagegen auf dem Mainthread.
public final class ScannerClient: @unchecked Sendable {
    private let queue = DispatchQueue(label: "lidar3d.scanner", qos: .userInitiated)
    private var connection: NWConnection?
    private var parser = FrameParser()
    private var candidates: [ScannerAddress] = []
    private var candidateIndex = 0
    private var probeTimer: DispatchSourceTimer?
    private var generation = 0

    /// Wie lange auf eine Adresse gewartet wird, bevor die naechste drankommt.
    public var probeTimeout: TimeInterval = 2.5

    /// Dense-Capsules des S2.
    public var onCapsule: ((CapsuleFrame) -> Void)?
    /// Scanframes des C1. Welche der beiden kommen, entscheidet die Firmware
    /// nach angeschlossenem Geraet - die App verarbeitet beide.
    public var onScan: ((ScanFrame) -> Void)?
    public var onState: ((ScannerConnectionState) -> Void)?
    public var onHello: ((HelloFrame) -> Void)?
    public var onStatus: ((StatusFrame) -> Void)?

    public init() {}

    // MARK: - Verbindungsaufbau

    public func connect(to addresses: [ScannerAddress] = ScannerAddress.defaults) {
        queue.async {
            self.teardown()
            self.generation += 1
            self.candidates = addresses
            self.candidateIndex = 0
            self.tryNextCandidate()
        }
    }

    public func disconnect() {
        queue.async {
            self.teardown()
            self.publishState(.idle)
        }
    }

    private func tryNextCandidate() {
        guard candidateIndex < candidates.count else {
            publishState(.failed("kein Scanner erreichbar"))
            return
        }
        let address = candidates[candidateIndex]
        candidateIndex += 1
        publishState(.connecting(address.label))

        let endpoint = NWEndpoint.hostPort(
            host: NWEndpoint.Host(address.host),
            port: NWEndpoint.Port(rawValue: address.port) ?? 5005)
        let options = NWProtocolTCP.Options()
        options.noDelay = true
        options.connectionTimeout = Int(probeTimeout)
        let connection = NWConnection(to: endpoint, using: NWParameters(tls: nil, tcp: options))
        self.connection = connection
        parser.reset()

        let generation = self.generation
        connection.stateUpdateHandler = { [weak self] state in
            guard let self, generation == self.generation else { return }
            switch state {
            case .ready:
                self.cancelProbeTimer()
                self.publishState(.connected(address.label))
                self.receive(on: connection)
            case .failed, .cancelled:
                self.cancelProbeTimer()
                self.advance(after: connection)
            default:
                break
            }
        }
        startProbeTimer(for: connection)
        connection.start(queue: queue)
    }

    /// NWConnection bleibt bei nicht erreichbaren Adressen in `.waiting` haengen,
    /// statt zu scheitern. Deshalb ein eigener Zeitgeber je Kandidat.
    private func startProbeTimer(for connection: NWConnection) {
        cancelProbeTimer()
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now() + probeTimeout)
        timer.setEventHandler { [weak self] in
            guard let self else { return }
            self.cancelProbeTimer()
            self.advance(after: connection)
        }
        probeTimer = timer
        timer.resume()
    }

    private func cancelProbeTimer() {
        probeTimer?.cancel()
        probeTimer = nil
    }

    private func advance(after connection: NWConnection) {
        guard connection === self.connection else { return }
        connection.cancel()
        self.connection = nil
        tryNextCandidate()
    }

    private func teardown() {
        cancelProbeTimer()
        connection?.cancel()
        connection = nil
        parser.reset()
    }

    // MARK: - Empfang

    private func receive(on connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 32768) {
            [weak self] data, _, isComplete, error in
            guard let self, connection === self.connection else { return }
            if let data, !data.isEmpty {
                for frame in self.parser.feed(data) { self.dispatch(frame) }
            }
            if error != nil || isComplete {
                self.publishState(.failed("Verbindung getrennt"))
                self.teardown()
                return
            }
            self.receive(on: connection)
        }
    }

    private func dispatch(_ frame: Frame) {
        switch frame.type {
        case FrameType.capsule.rawValue:
            if let capsule = CapsuleFrame(frame) { onCapsule?(capsule) }
        case FrameType.scan.rawValue:
            if let scan = ScanFrame(frame) { onScan?(scan) }
        case FrameType.hello.rawValue:
            if let hello = HelloFrame(frame) {
                DispatchQueue.main.async { self.onHello?(hello) }
            }
        case FrameType.status.rawValue:
            if let status = StatusFrame(frame) {
                DispatchQueue.main.async { self.onStatus?(status) }
            }
        default:
            break
        }
    }

    private func publishState(_ state: ScannerConnectionState) {
        DispatchQueue.main.async { self.onState?(state) }
    }

    // MARK: - Steuerung

    /// 'S' startet Homing und Sweep, 'X' bricht ab (siehe docs/03-protokolle.md).
    public func startSweep() { send(byte: 0x53) }
    public func abortSweep() { send(byte: 0x58) }

    private func send(byte: UInt8) {
        queue.async {
            self.connection?.send(content: Data([byte]), completion: .idempotent)
        }
    }
}
