import SwiftUI

@main
struct ScannerApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
                // Waehrend eines 18-s-Sweeps soll der Bildschirm anbleiben.
                .onAppear { UIApplication.shared.isIdleTimerDisabled = true }
                .onDisappear { UIApplication.shared.isIdleTimerDisabled = false }
        }
    }
}
