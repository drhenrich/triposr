"""Kommandozeile: python -m scan3d <befehl>

  plan      Sweep vorausrechnen (Dauer, Punktzahl, Aufloesung)
  capture   Sweep vom ESP32 ueber WLAN holen und als PLY speichern
  serial    S2 direkt am USB-Adapter aufzeichnen (ESP32 dreht nur die Achse)
  simulate  Synthetischen Raum scannen - testet die Kette ohne Hardware
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import List, Tuple

from . import geometry, ply, stream

Triple = Tuple[float, float, float]


def _mount_from_args(args: argparse.Namespace) -> geometry.MountGeometry:
    return geometry.MountGeometry(
        offset_radial_mm=args.offset_radial,
        offset_axial_mm=args.offset_axial,
        alpha_zero_deg=args.alpha_zero,
        alpha_sign=args.alpha_sign,
        yaw_zero_deg=args.yaw_zero,
    )


def _add_mount_args(p: argparse.ArgumentParser) -> None:
    g = p.add_argument_group("Einbaulage")
    g.add_argument("--offset-radial", type=float, default=0.0,
                   help="Abstand optisches Zentrum <-> Gierachse in mm")
    g.add_argument("--offset-axial", type=float, default=0.0,
                   help="Hoehe des optischen Zentrums in mm")
    g.add_argument("--alpha-zero", type=float, default=0.0,
                   help="LiDAR-Winkel, der nach oben zeigt")
    g.add_argument("--alpha-sign", type=int, choices=(1, -1), default=1)
    g.add_argument("--yaw-zero", type=float, default=0.0)
    g.add_argument("--min-mm", type=float, default=150.0)
    g.add_argument("--max-mm", type=float, default=30000.0)
    g.add_argument("--voxel", type=float, default=0.0,
                   help="Voxelgroesse in m zum Ausduennen (0 = aus)")


def _export(args: argparse.Namespace, samples: List[Triple]) -> None:
    mount = _mount_from_args(args)
    rng = geometry.RangeFilter(min_mm=args.min_mm, max_mm=args.max_mm)
    points = list(geometry.project(samples, mount, rng))
    dropped = len(samples) - len(points)
    if args.voxel > 0:
        before = len(points)
        points = geometry.voxel_downsample(points, args.voxel)
        print(f"Voxelfilter {args.voxel} m: {before} -> {len(points)} Punkte")
    colors = ply.height_colors(points) if args.color else None
    ply.write_ply(args.output, points, colors)
    print(f"{len(points)} Punkte geschrieben nach {args.output} "
          f"({dropped} Messungen verworfen)")


# ---------------------------------------------------------------------------


def cmd_plan(args: argparse.Namespace) -> int:
    info = geometry.sweep_plan(args.span, args.step, args.scan_hz, args.sample_rate)
    print(f"Gierbereich          {args.span:.1f} deg")
    print(f"Ebenenabstand        {args.step:.3f} deg")
    print(f"Scanebenen           {info['planes']:.0f}")
    print(f"Sweep-Dauer          {info['duration_s']:.1f} s")
    print(f"Gierrate             {info['yaw_rate_deg_s']:.3f} deg/s")
    print(f"Messungen je Ebene   {info['samples_per_plane']:.0f}")
    print(f"Aufloesung in Ebene  {info['in_plane_resolution_deg']:.4f} deg")
    print(f"Punkte gesamt        {info['total_samples']:.0f}")
    if args.span >= 180.0:
        print("\nHinweis: >=180 deg Gierbereich decken die volle Kugel ab, weil der")
        print("LiDAR in seiner Ebene bereits 360 deg misst. Kein Schleifring noetig.")
    return 0


def cmd_capture(args: argparse.Namespace) -> int:
    print(f"verbinde mit {args.host}:{args.port} ...")
    with stream.TcpSource(args.host, args.port, args.timeout) as src:
        samples, hello = stream.collect_sweep(src.frames())
    if hello is not None:
        print(f"Firmware v{hello.fw_version}, LiDAR {hello.lidar_rpm} rpm, "
              f"Gierbereich {hello.yaw_min_deg:.1f}..{hello.yaw_max_deg:.1f} deg")
        if args.offset_radial == 0.0:
            args.offset_radial = hello.offset_radial_mm
        if args.offset_axial == 0.0:
            args.offset_axial = hello.offset_axial_mm
    if not samples:
        print("keine Messungen empfangen", file=sys.stderr)
        return 1
    _export(args, samples)
    return 0


def cmd_serial(args: argparse.Namespace) -> int:
    from .serial_source import LinearYaw, SerialLidar

    yaw = LinearYaw(start_deg=args.yaw_start, rate_deg_s=args.yaw_rate)
    with SerialLidar(args.port, args.baudrate) as lidar:
        mode = lidar.start_dense_scan()
        print(f"Scanmodus {mode}, Dense-Capsules aktiv")
        yaw.start()
        deadline = time.monotonic() + args.duration
        samples: List[Triple] = []
        for triple in lidar.samples(yaw):
            samples.append(triple)
            if time.monotonic() >= deadline:
                break
        print(f"{len(samples)} Messungen, {lidar.checksum_errors} Pruefsummenfehler")
    if not samples:
        return 1
    _export(args, samples)
    return 0


def cmd_simulate(args: argparse.Namespace) -> int:
    """Quader-Raum mit dem echten Sweep-Muster abtasten."""
    hx, hy, hz = args.room[0] / 2, args.room[1] / 2, args.room[2] / 2
    plan = geometry.sweep_plan(args.span, args.step, args.scan_hz, args.sample_rate)
    per_plane = int(plan["samples_per_plane"])
    planes = int(plan["planes"])
    samples: List[Triple] = []
    for p in range(planes):
        yaw = args.span * p / planes
        psi = math.radians(yaw)
        for k in range(per_plane):
            alpha = 360.0 * k / per_plane
            a = math.radians(alpha)
            # Richtungsvektor der Messung in Weltkoordinaten
            dx = math.sin(a) * math.cos(psi)
            dy = math.sin(a) * math.sin(psi)
            dz = math.cos(a)
            # Strahl vs. achsenparalleler Quader (Sensor sitzt im Ursprung)
            t = math.inf
            for d, half in ((dx, hx), (dy, hy), (dz, hz)):
                if abs(d) > 1e-9:
                    t = min(t, (half if d > 0 else -half) / d)
            if not math.isfinite(t):
                continue
            samples.append((t * 1000.0, alpha, yaw))
    print(f"{len(samples)} simulierte Messungen ({planes} Ebenen x {per_plane})")
    _export(args, samples)
    return 0


# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="scan3d", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)

    plan = sub.add_parser("plan", help="Sweep vorausrechnen")
    plan.add_argument("--span", type=float, default=180.0, help="Gierbereich in deg")
    plan.add_argument("--step", type=float, default=1.0, help="Ebenenabstand in deg")
    plan.add_argument("--scan-hz", type=float, default=10.0)
    plan.add_argument("--sample-rate", type=float, default=32000.0)
    plan.set_defaults(func=cmd_plan)

    cap = sub.add_parser("capture", help="Sweep ueber WLAN holen")
    cap.add_argument("host")
    cap.add_argument("--port", type=int, default=5005)
    cap.add_argument("--timeout", type=float, default=10.0)
    cap.add_argument("-o", "--output", default="scan.ply")
    cap.add_argument("--color", action="store_true", help="Hoehenfarben schreiben")
    _add_mount_args(cap)
    cap.set_defaults(func=cmd_capture)

    ser = sub.add_parser("serial", help="S2 direkt am USB-Adapter")
    ser.add_argument("port", help="z.B. /dev/ttyUSB0 oder COM5")
    ser.add_argument("--baudrate", type=int, default=1_000_000)
    ser.add_argument("--duration", type=float, default=20.0, help="Aufnahmedauer in s")
    ser.add_argument("--yaw-rate", type=float, default=10.0, help="Gierrate in deg/s")
    ser.add_argument("--yaw-start", type=float, default=0.0)
    ser.add_argument("-o", "--output", default="scan.ply")
    ser.add_argument("--color", action="store_true")
    _add_mount_args(ser)
    ser.set_defaults(func=cmd_serial)

    sim = sub.add_parser("simulate", help="synthetischen Raum scannen")
    sim.add_argument("--room", type=float, nargs=3, default=[6.0, 4.0, 2.6],
                     metavar=("X", "Y", "Z"), help="Raummasse in m")
    sim.add_argument("--span", type=float, default=180.0)
    sim.add_argument("--step", type=float, default=1.0)
    sim.add_argument("--scan-hz", type=float, default=10.0)
    sim.add_argument("--sample-rate", type=float, default=32000.0)
    sim.add_argument("-o", "--output", default="simulated.ply")
    sim.add_argument("--color", action="store_true")
    _add_mount_args(sim)
    sim.set_defaults(func=cmd_simulate)

    return p


def main(argv: List[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
