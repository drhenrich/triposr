"""3D-Scanner aus einem 2D-LiDAR (RPLIDAR S2) auf einer Gierachse.

Module:
  rplidar   - S2-Protokoll, Dekoder fuer Dense-Capsules
  stream    - Frameprotokoll ESP32-S3 -> Host (WLAN/TCP)
  geometry  - Kugelkoordinaten, Einbaulage, Sweep-Planung
  ply       - Export der Punktwolke
  cli       - Kommandozeile
"""

__version__ = "0.1.0"
