// Oberflaeche: Punktwolke bildschirmfuellend, HUD darueber.

import LidarKit
import MetalKit
import SwiftUI

struct ContentView: View {
    @StateObject private var model = ScannerViewModel()
    @State private var azimuth: Float = 0.6
    @State private var elevation: Float = 0.35
    @State private var distance: Float = 8.0
    @State private var dragStart: (Float, Float)?
    @State private var pinchStart: Float?

    var body: some View {
        ZStack(alignment: .topLeading) {
            PointCloudView(source: model.cloud,
                           azimuth: azimuth,
                           elevation: elevation,
                           distance: distance,
                           pointSize: model.pointSize,
                           visiblePoints: $model.visiblePoints)
                .ignoresSafeArea()
                .gesture(orbitGesture)
                .simultaneousGesture(zoomGesture)

            hud
                .padding(12)

            VStack {
                Spacer()
                controls
                    .padding(.bottom, 24)
            }
            .frame(maxWidth: .infinity)
        }
        .background(Color.black)
        .preferredColorScheme(.dark)
        .onAppear { model.connect() }
    }

    // MARK: - HUD

    private var hud: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text("STANDALONE SWEEP | \(model.transportLabel.uppercased())")
                .foregroundStyle(model.isConnected ? .green : .orange)
            Text("\(model.stateLabel) | yaw \(format(model.status?.yawDeg ?? 0, 1)) deg")
            Text("Punkte \(model.visiblePoints) | \(format(model.framesPerSecond, 0)) Frames/s")
            if let status = model.status {
                Text("Pruefsummenfehler \(status.checksumErrors) | verworfen \(status.droppedFrames)")
                    .foregroundStyle(model.hasDrops ? .red : .secondary)
            }
            if let hello = model.hello {
                Text("FW \(hello.fwVersion) | \(hello.lidarRpm) rpm | "
                     + "yaw \(format(hello.yawMinDeg, 0))..\(format(hello.yawMaxDeg, 0)) deg")
                    .foregroundStyle(.secondary)
            }
        }
        .font(.system(size: 11, weight: .medium, design: .monospaced))
        .foregroundStyle(.green)
        .shadow(radius: 2)
    }

    // MARK: - Bedienung

    private var controls: some View {
        HStack(spacing: 10) {
            button(model.isConnected ? "TRENNEN" : "VERBINDEN") {
                model.isConnected ? model.disconnect() : model.connect()
            }
            button("SWEEP") { model.startSweep() }
                .disabled(!model.isConnected)
            button("STOP") { model.abortSweep() }
                .disabled(!model.isConnected)
            button("ZURUECKSETZEN") { model.cloud.clear() }
        }
        .font(.system(size: 12, weight: .semibold, design: .monospaced))
    }

    private func button(_ title: String, action: @escaping () -> Void) -> some View {
        Button(title, action: action)
            .padding(.horizontal, 12)
            .padding(.vertical, 8)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
            .foregroundStyle(.green)
    }

    // MARK: - Kamera

    private var orbitGesture: some Gesture {
        DragGesture()
            .onChanged { value in
                if dragStart == nil { dragStart = (azimuth, elevation) }
                guard let start = dragStart else { return }
                azimuth = start.0 - Float(value.translation.width) * 0.005
                elevation = max(-1.5, min(1.5,
                    start.1 + Float(value.translation.height) * 0.005))
            }
            .onEnded { _ in dragStart = nil }
    }

    private var zoomGesture: some Gesture {
        MagnificationGesture()
            .onChanged { scale in
                if pinchStart == nil { pinchStart = distance }
                guard let start = pinchStart else { return }
                distance = max(0.4, min(80, start / Float(scale)))
            }
            .onEnded { _ in pinchStart = nil }
    }

    private func format(_ value: Double, _ digits: Int) -> String {
        String(format: "%.\(digits)f", value)
    }

    private func format(_ value: Float, _ digits: Int) -> String {
        String(format: "%.\(digits)f", value)
    }
}

// MARK: - Metal-Ansicht

struct PointCloudView: UIViewRepresentable {
    let source: PointCloudBuffer
    var azimuth: Float
    var elevation: Float
    var distance: Float
    var pointSize: Float
    @Binding var visiblePoints: Int

    func makeCoordinator() -> Coordinator { Coordinator(visiblePoints: $visiblePoints) }

    func makeUIView(context: Context) -> MTKView {
        let view = MTKView()
        context.coordinator.renderer = PointCloudRenderer(view: view, source: source)
        context.coordinator.start()
        return view
    }

    func updateUIView(_ view: MTKView, context: Context) {
        guard let renderer = context.coordinator.renderer else { return }
        renderer.azimuth = azimuth
        renderer.elevation = elevation
        renderer.distance = distance
        renderer.pointSize = pointSize
    }

    static func dismantleUIView(_ view: MTKView, coordinator: Coordinator) {
        coordinator.stop()
    }

    final class Coordinator {
        var renderer: PointCloudRenderer?
        private var timer: Timer?
        private let visiblePoints: Binding<Int>

        init(visiblePoints: Binding<Int>) {
            self.visiblePoints = visiblePoints
        }

        /// Die Punktzahl wird bewusst nur 5x/s in SwiftUI gespiegelt, nicht je
        /// Bild - sonst laeuft SwiftUI mit 60 Hz durch den Diffing-Algorithmus.
        func start() {
            timer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { [weak self] _ in
                // Timer feuert auf dem Mainthread (RunLoop des Mainthreads).
                MainActor.assumeIsolated {
                    guard let count = self?.renderer?.pointCount else { return }
                    self?.visiblePoints.wrappedValue = count
                }
            }
        }

        func stop() {
            timer?.invalidate()
            timer = nil
        }
    }
}
