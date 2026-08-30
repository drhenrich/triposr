// Frameprotokoll des Scanners, Empfaengerseite.
//
// Spiegelt host/scan3d/stream.py und firmware/src/stream_proto.h. Alle drei
// Implementierungen pruefen ihr Byte-Layout gegen tests/wire_fixture.txt.
//
// Bewusst ohne Abhaengigkeit zu Network.framework oder Metal, damit sich der
// Decoder isoliert testen laesst.

import Foundation

public enum FrameType: UInt8, Sendable {
    case hello = 0
    case capsule = 1
    case status = 2
}

public struct FrameFlags: OptionSet, Sendable {
    public let rawValue: UInt8
    public init(rawValue: UInt8) { self.rawValue = rawValue }

    public static let newRevolution = FrameFlags(rawValue: 1 << 0)
    public static let sweepActive = FrameFlags(rawValue: 1 << 1)
    public static let sweepReversed = FrameFlags(rawValue: 1 << 2)
}

public enum SweepState: UInt8, Sendable {
    case idle = 0
    case homing = 1
    case sweeping = 2
    case returning = 3
}

public struct Frame: Sendable {
    public let type: UInt8
    public let flags: FrameFlags
    public let seq: UInt16
    public let payload: [UInt8]
}

// MARK: - Little-Endian-Helfer

@inline(__always)
func readU16(_ b: [UInt8], _ i: Int) -> UInt16 {
    UInt16(b[i]) | (UInt16(b[i + 1]) << 8)
}

@inline(__always)
func readU32(_ b: [UInt8], _ i: Int) -> UInt32 {
    UInt32(b[i]) | (UInt32(b[i + 1]) << 8) | (UInt32(b[i + 2]) << 16) | (UInt32(b[i + 3]) << 24)
}

@inline(__always)
func readI32(_ b: [UInt8], _ i: Int) -> Int32 {
    Int32(bitPattern: readU32(b, i))
}

// MARK: - Frame-Parser

/// Byte-Strom -> Frames. Synchronisiert byteweise auf das Magic, falls der
/// Strom mitten im Frame beginnt oder Bytes verloren gehen.
public struct FrameParser {
    public static let magic: UInt16 = 0x4E57  // 'NW'
    public static let headerSize = 8
    public static let maxPayload = 4096

    private var buffer: [UInt8] = []
    private var readIndex = 0
    public private(set) var resyncs = 0

    public init() {}

    public mutating func reset() {
        buffer.removeAll(keepingCapacity: true)
        readIndex = 0
    }

    public mutating func feed(_ data: [UInt8]) -> [Frame] {
        buffer.append(contentsOf: data)
        var frames: [Frame] = []
        while let frame = pop() { frames.append(frame) }
        compact()
        return frames
    }

    public mutating func feed(_ data: Data) -> [Frame] {
        feed([UInt8](data))
    }

    private mutating func pop() -> Frame? {
        while true {
            let available = buffer.count - readIndex
            if available < Self.headerSize { return nil }

            let magic = readU16(buffer, readIndex)
            let length = Int(readU16(buffer, readIndex + 6))
            if magic != Self.magic || length > Self.maxPayload {
                readIndex += 1
                resyncs += 1
                continue
            }
            if available < Self.headerSize + length { return nil }

            let type = buffer[readIndex + 2]
            let flags = FrameFlags(rawValue: buffer[readIndex + 3])
            let seq = readU16(buffer, readIndex + 4)
            let start = readIndex + Self.headerSize
            let payload = Array(buffer[start ..< start + length])
            readIndex = start + length
            return Frame(type: type, flags: flags, seq: seq, payload: payload)
        }
    }

    /// Verbrauchte Bytes gelegentlich wegwerfen. Ohne das waechst der Puffer
    /// bei 800 Frames/s unbegrenzt.
    private mutating func compact() {
        guard readIndex > 4096 else { return }
        buffer.removeFirst(readIndex)
        readIndex = 0
    }
}

// MARK: - Capsule

public struct CapsuleFrame: Sendable {
    public static let cabinCount = 40
    public static let payloadSize = 16 + 2 * cabinCount  // 96

    public let seq: UInt16
    public let flags: FrameFlags
    public let yawStartDeg: Float
    public let yawEndDeg: Float
    public let alphaStartDeg: Float
    public let alphaIncDeg: Float
    public let distancesMm: [UInt16]

    public var sweepActive: Bool { flags.contains(.sweepActive) }

    public init?(_ frame: Frame) {
        guard frame.type == FrameType.capsule.rawValue,
              frame.payload.count == Self.payloadSize else { return nil }
        let p = frame.payload
        seq = frame.seq
        flags = frame.flags
        yawStartDeg = Float(readU32(p, 0)) / 65536.0
        yawEndDeg = Float(readU32(p, 4)) / 65536.0
        alphaIncDeg = Float(readI32(p, 8)) / 65536.0
        alphaStartDeg = Float(readU16(p, 12)) / 64.0
        var d = [UInt16](repeating: 0, count: Self.cabinCount)
        for i in 0 ..< Self.cabinCount { d[i] = readU16(p, 16 + 2 * i) }
        distancesMm = d
    }

    /// Ruft `body` fuer jede der 40 Messungen mit (Distanz mm, Scanwinkel, Gierwinkel).
    /// Gier- und Scanwinkel werden linear ueber die Capsule interpoliert.
    public func forEachSample(_ body: (Float, Float, Float) -> Void) {
        var yawSpan = yawEndDeg - yawStartDeg
        // Nulldurchgang der Gierachse abfangen (im Sweep sollte er nicht auftreten).
        if yawSpan > 180 { yawSpan -= 360 } else if yawSpan < -180 { yawSpan += 360 }
        let n = Float(distancesMm.count)
        for (i, dist) in distancesMm.enumerated() {
            let f = Float(i)
            var alpha = (alphaStartDeg + f * alphaIncDeg).truncatingRemainder(dividingBy: 360)
            if alpha < 0 { alpha += 360 }
            body(Float(dist), alpha, yawStartDeg + yawSpan * f / n)
        }
    }
}

public struct HelloFrame: Sendable {
    public let fwVersion: UInt16
    public let lidarRpm: UInt16
    public let offsetRadialMm: Float
    public let offsetAxialMm: Float
    public let yawMinDeg: Float
    public let yawMaxDeg: Float

    public init?(_ frame: Frame) {
        guard frame.type == FrameType.hello.rawValue, frame.payload.count >= 20 else { return nil }
        let p = frame.payload
        fwVersion = readU16(p, 0)
        lidarRpm = readU16(p, 2)
        offsetRadialMm = Float(readI32(p, 4)) / 1000.0
        offsetAxialMm = Float(readI32(p, 8)) / 1000.0
        yawMinDeg = Float(readU32(p, 12)) / 65536.0
        yawMaxDeg = Float(readU32(p, 16)) / 65536.0
    }
}

public struct StatusFrame: Sendable {
    public let sweepIndex: UInt16
    public let state: SweepState
    public let yawDeg: Float
    public let capsules: UInt32
    public let checksumErrors: UInt32
    public let droppedFrames: UInt32

    public init?(_ frame: Frame) {
        guard frame.type == FrameType.status.rawValue, frame.payload.count >= 20 else { return nil }
        let p = frame.payload
        sweepIndex = readU16(p, 0)
        state = SweepState(rawValue: p[2]) ?? .idle
        yawDeg = Float(readU32(p, 4)) / 65536.0
        capsules = readU32(p, 8)
        checksumErrors = readU32(p, 12)
        droppedFrames = readU32(p, 16)
    }
}
