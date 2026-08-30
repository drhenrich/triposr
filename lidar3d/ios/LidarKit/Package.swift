// swift-tools-version: 5.9
import PackageDescription

// Der Decoder ist absichtlich ein eigenes Paket ohne UI- oder Metal-Bezug:
// so laesst er sich auf dem Mac testen, ohne die App zu starten.
//
//   cd ios/LidarKit && swift test
//
let package = Package(
    name: "LidarKit",
    platforms: [.iOS(.v17), .macOS(.v14)],
    products: [
        .library(name: "LidarKit", targets: ["LidarKit"])
    ],
    targets: [
        .target(name: "LidarKit"),
        .testTarget(name: "LidarKitTests", dependencies: ["LidarKit"])
    ]
)
