"""Minimaler PLY-Writer (binary little endian) fuer Punktwolken.

Bewusst ohne numpy/open3d, damit der Export ueberall laeuft. Die Dateien
oeffnen sich direkt in CloudCompare, MeshLab, Blender oder open3d.
"""

from __future__ import annotations

import io
import struct
from typing import BinaryIO, Iterable, Optional, Sequence, Tuple

Point = Tuple[float, float, float]
Color = Tuple[int, int, int]

_HEADER = """ply
format binary_little_endian 1.0
comment created by lidar3d (RPLIDAR S2 spinning scanner)
element vertex {count}
property float x
property float y
property float z
"""

_COLOR_PROPS = """property uchar red
property uchar green
property uchar blue
"""


def write_ply(
    path: str,
    points: Sequence[Point],
    colors: Optional[Sequence[Color]] = None,
) -> None:
    """Punktwolke schreiben. ``colors`` muss, falls gesetzt, gleich lang sein."""
    if colors is not None and len(colors) != len(points):
        raise ValueError("colors und points muessen gleich lang sein")
    with open(path, "wb") as fh:
        _write_header(fh, len(points), colors is not None)
        if colors is None:
            packer = struct.Struct("<fff")
            for p in points:
                fh.write(packer.pack(*p))
        else:
            packer = struct.Struct("<fffBBB")
            for p, c in zip(points, colors):
                fh.write(packer.pack(p[0], p[1], p[2], c[0], c[1], c[2]))


def dumps_ply(
    points: Sequence[Point],
    colors: Optional[Sequence[Color]] = None,
) -> bytes:
    """Wie ``write_ply``, aber in den Speicher - fuer Downloads im Browser."""
    if colors is not None and len(colors) != len(points):
        raise ValueError("colors und points muessen gleich lang sein")
    buffer = io.BytesIO()
    _write_header(buffer, len(points), colors is not None)
    if colors is None:
        packer = struct.Struct("<fff")
        for p in points:
            buffer.write(packer.pack(*p))
    else:
        packer = struct.Struct("<fffBBB")
        for p, c in zip(points, colors):
            buffer.write(packer.pack(p[0], p[1], p[2], c[0], c[1], c[2]))
    return buffer.getvalue()


def _write_header(fh: BinaryIO, count: int, with_color: bool) -> None:
    header = _HEADER.format(count=count)
    if with_color:
        header += _COLOR_PROPS
    header += "end_header\n"
    fh.write(header.encode("ascii"))


def height_colors(
    points: Iterable[Point], lo: Optional[float] = None, hi: Optional[float] = None
) -> list:
    """Gruener Hoehenverlauf ueber Z - naeher am Look der Vorlage als Grau."""
    pts = list(points)
    if not pts:
        return []
    zs = [p[2] for p in pts]
    lo = min(zs) if lo is None else lo
    hi = max(zs) if hi is None else hi
    span = (hi - lo) or 1.0
    out = []
    for p in pts:
        t = max(0.0, min(1.0, (p[2] - lo) / span))
        out.append((int(40 * t), int(90 + 165 * t), int(60 + 60 * t)))
    return out
