// Echtzeitdarstellung der Punktwolke.
//
// Der Netz-Thread schiebt Punkte in den PointCloudBuffer, der Renderer holt
// sie einmal je Bild ab und haengt sie an den GPU-Puffer an. Es wird nur
// angehaengt, nie ueberschrieben - deshalb braucht es keine Synchronisierung
// mit der GPU: der Zeichenaufruf kennt seine Punktzahl, alles danach ist
// unsichtbar und stoert nicht.
//
// Ein neuer Sweep setzt den Schreibzeiger auf 0 zurueck.

import Foundation
import LidarKit
import Metal
import MetalKit
import simd

struct PointCloudUniforms {
    var mvp: simd_float4x4
    var pointSize: Float
    var zMin: Float
    var zMax: Float
}

final class PointCloudRenderer: NSObject, MTKViewDelegate {

    // MARK: Kamera (von der SwiftUI-Ansicht gesetzt)
    var azimuth: Float = 0.6
    var elevation: Float = 0.35
    var distance: Float = 8.0
    var pointSize: Float = 3.0

    private(set) var pointCount = 0
    private(set) var droppedForCapacity = 0

    private let device: MTLDevice
    private let queue: MTLCommandQueue
    private let pipeline: MTLRenderPipelineState
    private let depthState: MTLDepthStencilState
    private let vertexBuffer: MTLBuffer
    private let capacity: Int
    private let source: PointCloudBuffer

    private var generation = -1
    private var zMin: Float = -1
    private var zMax: Float = 1

    init?(view: MTKView, source: PointCloudBuffer) {
        guard let device = MTLCreateSystemDefaultDevice(),
              let queue = device.makeCommandQueue() else { return nil }
        self.device = device
        self.queue = queue
        self.source = source
        self.capacity = source.capacity

        let stride = MemoryLayout<SIMD3<Float>>.stride  // 16, passt zu float3 in Metal
        guard let buffer = device.makeBuffer(length: capacity * stride,
                                             options: .storageModeShared) else { return nil }
        self.vertexBuffer = buffer

        guard let library = device.makeDefaultLibrary(),
              let vertexFunction = library.makeFunction(name: "pointVertex"),
              let fragmentFunction = library.makeFunction(name: "pointFragment")
        else { return nil }

        view.device = device
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float
        view.clearColor = MTLClearColor(red: 0.02, green: 0.03, blue: 0.04, alpha: 1)
        view.preferredFramesPerSecond = 60

        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.fragmentFunction = fragmentFunction
        descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        guard let pipeline = try? device.makeRenderPipelineState(descriptor: descriptor)
        else { return nil }
        self.pipeline = pipeline

        let depth = MTLDepthStencilDescriptor()
        depth.depthCompareFunction = .less
        depth.isDepthWriteEnabled = true
        guard let depthState = device.makeDepthStencilState(descriptor: depth) else { return nil }
        self.depthState = depthState

        super.init()
        view.delegate = self
    }

    // MARK: - Punkte einsammeln

    private func ingest() {
        let (points, generation) = source.drain()

        if generation != self.generation {
            self.generation = generation
            pointCount = 0
            droppedForCapacity = 0
            zMin = -1
            zMax = 1
        }
        guard !points.isEmpty else { return }

        let room = capacity - pointCount
        if room <= 0 {
            droppedForCapacity += points.count
            return
        }
        let take = min(room, points.count)
        if take < points.count { droppedForCapacity += points.count - take }

        let stride = MemoryLayout<SIMD3<Float>>.stride
        points.withUnsafeBytes { raw in
            guard let base = raw.baseAddress else { return }
            vertexBuffer.contents()
                .advanced(by: pointCount * stride)
                .copyMemory(from: base, byteCount: take * stride)
        }
        pointCount += take

        // Farbverlauf an die tatsaechliche Hoehe anpassen.
        for i in 0 ..< take {
            let z = points[i].z
            if z < zMin { zMin = z }
            if z > zMax { zMax = z }
        }
    }

    // MARK: - Zeichnen

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        ingest()

        guard let descriptor = view.currentRenderPassDescriptor,
              let drawable = view.currentDrawable,
              let commandBuffer = queue.makeCommandBuffer(),
              let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: descriptor)
        else { return }

        if pointCount > 0 {
            let size = view.drawableSize
            let aspect = size.height > 0 ? Float(size.width / size.height) : 1
            var uniforms = PointCloudUniforms(mvp: viewProjection(aspect: aspect),
                                              pointSize: pointSize,
                                              zMin: zMin,
                                              zMax: zMax)
            encoder.setRenderPipelineState(pipeline)
            encoder.setDepthStencilState(depthState)
            encoder.setVertexBuffer(vertexBuffer, offset: 0, index: 0)
            encoder.setVertexBytes(&uniforms,
                                   length: MemoryLayout<PointCloudUniforms>.stride,
                                   index: 1)
            encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: pointCount)
        }

        encoder.endEncoding()
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    /// Orbit-Kamera um den Ursprung; der Scanner steht dort.
    private func viewProjection(aspect: Float) -> simd_float4x4 {
        let eye = SIMD3<Float>(distance * cos(elevation) * sin(azimuth),
                               distance * cos(elevation) * cos(azimuth),
                               distance * sin(elevation))
        let view = lookAt(eye: eye, center: .zero, up: SIMD3<Float>(0, 0, 1))
        let projection = perspective(fovYRadians: .pi / 3, aspect: aspect,
                                     near: 0.05, far: 200)
        return projection * view
    }
}

// MARK: - Matrizen

func perspective(fovYRadians: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
    let y = 1 / tan(fovYRadians * 0.5)
    let x = y / aspect
    let z = far / (near - far)
    return simd_float4x4(columns: (
        SIMD4<Float>(x, 0, 0, 0),
        SIMD4<Float>(0, y, 0, 0),
        SIMD4<Float>(0, 0, z, -1),
        SIMD4<Float>(0, 0, z * near, 0)
    ))
}

func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>, up: SIMD3<Float>) -> simd_float4x4 {
    let f = normalize(center - eye)
    let s = normalize(cross(f, up))
    let u = cross(s, f)
    return simd_float4x4(columns: (
        SIMD4<Float>(s.x, u.x, -f.x, 0),
        SIMD4<Float>(s.y, u.y, -f.y, 0),
        SIMD4<Float>(s.z, u.z, -f.z, 0),
        SIMD4<Float>(-dot(s, eye), -dot(u, eye), dot(f, eye), 1)
    ))
}
