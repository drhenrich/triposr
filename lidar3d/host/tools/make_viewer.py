"""PLY -> eigenstaendige HTML-Seite mit drehender Punktwolke.

    python3 tools/make_viewer.py scan.ply -o viewer.html

Die Seite braucht nichts ausser einem Browser: die Punkte stecken als
Base64-Float32Array darin, gezeichnet wird mit three.js von cdnjs. Sie dreht
sich von selbst und laesst sich mit der Maus schwenken.

Neben der Ansicht steht ein kurzer Befund: die Kennzahlen, an denen sich
ablesen laesst, ob der Scan etwas taugt. Der wichtigste davon ist die
Rotationssymmetrie - eine Wolke, die um die Drehachse herum ueberall gleich
aussieht, enthaelt keine Information aus der Drehung.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import struct
import sys
from typing import List, Optional, Tuple

# Das Paket liegt eine Ebene ueber diesem Ordner.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scan3d.alignment import estimate_alpha_zero, rotate_cloud  # noqa: E402
from scan3d.quality import analyse, verdict  # noqa: E402

Point = Tuple[float, float, float]


# ---------------------------------------------------------------------------
# PLY lesen
# ---------------------------------------------------------------------------


def read_ply(path: str) -> List[Point]:
    """Liest binaere (little endian) und ASCII-PLY-Dateien mit x/y/z."""
    with open(path, "rb") as fh:
        raw = fh.read()

    marker = b"end_header"
    end = raw.find(marker)
    if end < 0:
        raise ValueError("kein PLY-Header gefunden")
    header = raw[:end].decode("ascii", "replace")
    body_at = raw.index(b"\n", end) + 1

    fmt = None
    count = 0
    props: List[str] = []
    for line in header.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element" and parts[1] == "vertex":
            count = int(parts[2])
        elif parts[0] == "property" and len(parts) >= 3:
            props.append(parts[2])

    if count == 0:
        raise ValueError("die Datei enthaelt keine Punkte")

    if fmt == "ascii":
        points = []
        for line in raw[body_at:].decode("ascii", "replace").splitlines():
            values = line.split()
            if len(values) < 3:
                continue
            points.append((float(values[0]), float(values[1]), float(values[2])))
            if len(points) == count:
                break
        return points

    if fmt != "binary_little_endian":
        raise ValueError(f"Format {fmt} wird nicht unterstuetzt")

    # Nur die hier vorkommenden Typen; float/uchar deckt unsere Dateien ab.
    sizes = {"float": 4, "float32": 4, "double": 8, "uchar": 1, "uint8": 1,
             "char": 1, "int8": 1, "short": 2, "ushort": 2, "int": 4, "uint": 4}
    types = []
    for line in header.splitlines():
        parts = line.split()
        if parts and parts[0] == "property" and len(parts) >= 3:
            types.append(parts[1])
    stride = sum(sizes.get(t, 4) for t in types)
    if stride == 0:
        raise ValueError("Satzlaenge liess sich nicht bestimmen")

    unpack = struct.Struct("<fff").unpack_from
    return [unpack(raw, body_at + k * stride) for k in range(count)]


# ---------------------------------------------------------------------------
# Seite bauen
# ---------------------------------------------------------------------------

TEMPLATE = r"""<title>__TITLE__</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
  /* Bewusst nur ein Erscheinungsbild: eine Punktwolke ist ein
     Instrumentendisplay und lebt vom dunklen Grund. Alle Farben werden
     deshalb explizit gesetzt, damit die Seite auf jedem Untergrund haelt. */
  :root {
    --ground: #0B1016;
    --surface: #131C24;
    --surface-2: #1B2732;
    --line: #263543;
    --ink: #E4EAF0;
    --muted: #8395A6;
    --accent: #E8A33D;
    --accent-dim: #8A6425;
    --warn: #E2574C;
    --ok: #6FB3A8;
    --sans: "IBM Plex Sans", ui-sans-serif, system-ui, sans-serif;
    --mono: "IBM Plex Mono", ui-monospace, "SFMono-Regular", monospace;
  }

  * { box-sizing: border-box; }

  body {
    margin: 0;
    background: var(--ground);
    color: var(--ink);
    font-family: var(--sans);
    height: 100vh;
    overflow: hidden;
  }

  #stage { position: fixed; inset: 0; }
  canvas { display: block; touch-action: none; }

  .panel {
    position: fixed;
    background: color-mix(in srgb, var(--surface) 88%, transparent);
    border: 1px solid var(--line);
    border-radius: 3px;
    backdrop-filter: blur(12px);
  }

  /* -- Kopf ------------------------------------------------------------- */

  #head {
    top: 16px; left: 16px;
    padding: 14px 18px 12px;
    max-width: 320px;
  }
  #head h1 {
    margin: 0;
    font-size: 15px;
    font-weight: 600;
    letter-spacing: -0.01em;
    text-wrap: balance;
  }
  #head .eyebrow {
    font-family: var(--mono);
    font-size: 10px;
    letter-spacing: 0.14em;
    text-transform: uppercase;
    color: var(--accent);
    margin-bottom: 6px;
  }

  .readout {
    display: grid;
    grid-template-columns: auto 1fr;
    gap: 3px 14px;
    margin-top: 12px;
    padding-top: 12px;
    border-top: 1px solid var(--line);
    font-family: var(--mono);
    font-size: 11px;
    font-variant-numeric: tabular-nums;
  }
  .readout dt { color: var(--muted); }
  .readout dd { margin: 0; text-align: right; }

  /* -- Befund ----------------------------------------------------------- */

  #finding {
    top: 16px; right: 16px;
    width: 330px;
    padding: 14px 18px 16px;
    border-left: 2px solid var(--stripe, var(--muted));
  }
  /* Der Wert ist ein ASCII-Schluessel, kein Anzeigetext: in <style> werden
     Zeichenreferenzen nicht aufgeloest, ein Umlaut im Selektor wuerde also
     nicht mehr treffen, sobald die Seite nach ASCII maskiert wird. */
  #finding[data-level="kritisch"]   { --stripe: var(--warn); }
  #finding[data-level="auffaellig"] { --stripe: var(--accent); }
  #finding[data-level="plausibel"]  { --stripe: var(--ok); }

  #finding .level {
    font-family: var(--mono);
    font-size: 10px;
    letter-spacing: 0.14em;
    text-transform: uppercase;
    color: var(--stripe, var(--muted));
  }
  #finding h2 {
    margin: 6px 0 8px;
    font-size: 14px;
    font-weight: 600;
    text-wrap: balance;
  }
  #finding p {
    margin: 0;
    font-size: 12.5px;
    line-height: 1.55;
    color: var(--muted);
  }
  #finding p strong { color: var(--ink); font-weight: 500; }

  /* -- Bedienung -------------------------------------------------------- */

  #controls {
    bottom: 16px; left: 50%;
    transform: translateX(-50%);
    display: flex;
    align-items: center;
    gap: 18px;
    padding: 10px 16px;
    flex-wrap: wrap;
    justify-content: center;
    max-width: calc(100vw - 32px);
  }
  .control { display: flex; align-items: center; gap: 8px; }
  .control label {
    font-family: var(--mono);
    font-size: 10px;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: var(--muted);
    white-space: nowrap;
  }

  button {
    font-family: var(--mono);
    font-size: 11px;
    letter-spacing: 0.06em;
    color: var(--ink);
    background: var(--surface-2);
    border: 1px solid var(--line);
    border-radius: 2px;
    padding: 6px 12px;
    cursor: pointer;
  }
  button:hover { border-color: var(--accent-dim); }
  button[aria-pressed="true"] {
    color: var(--ground);
    background: var(--accent);
    border-color: var(--accent);
  }
  button:focus-visible, select:focus-visible, input:focus-visible {
    outline: 2px solid var(--accent);
    outline-offset: 2px;
  }

  select {
    font-family: var(--mono);
    font-size: 11px;
    color: var(--ink);
    background: var(--surface-2);
    border: 1px solid var(--line);
    border-radius: 2px;
    padding: 5px 8px;
  }

  input[type="range"] {
    width: 92px;
    accent-color: var(--accent);
    background: transparent;
  }

  #hint {
    position: fixed;
    bottom: 16px; left: 16px;
    font-family: var(--mono);
    font-size: 10px;
    color: var(--muted);
    letter-spacing: 0.04em;
  }

  @media (max-width: 900px) {
    #finding { display: none; }
    #head { max-width: 240px; }
  }
  @media (prefers-reduced-motion: reduce) {
    .panel { backdrop-filter: none; }
  }
</style>

<div id="stage"></div>

<div class="panel" id="head">
  <div class="eyebrow">__EYEBROW__</div>
  <h1>__HEADLINE__</h1>
  <dl class="readout" id="readout"></dl>
</div>

<div class="panel" id="finding" data-level="__LEVEL_KEY__">
  <div class="level">Befund · __LEVEL__</div>
  <h2>__VERDICT_TITLE__</h2>
  <p>__VERDICT_TEXT__</p>
</div>

<div class="panel" id="controls">
  <div class="control">
    <button id="spin" aria-pressed="true">Drehung</button>
    <input type="range" id="speed" min="0" max="100" value="35" aria-label="Drehgeschwindigkeit">
  </div>
  <div class="control">
    <label for="colour">Färbung</label>
    <select id="colour">
      <option value="radius">Abstand zur Achse</option>
      <option value="height">Höhe</option>
      <option value="plane">Scanebene</option>
    </select>
  </div>
  <div class="control">
    <label for="size">Punkt</label>
    <input type="range" id="size" min="1" max="12" value="4" aria-label="Punktgröße">
  </div>
  <div class="control">
    <button id="top">Von oben</button>
    <button id="reset">Zurücksetzen</button>
  </div>
</div>

<div id="hint">Ziehen dreht · Rad zoomt</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
const DATA = "__DATA__";
const INFO = __INFO__;

// Base64 -> Float32Array (x, y, z je Punkt)
function decode(b64) {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return new Float32Array(bytes.buffer);
}

const positions = decode(DATA);
const count = positions.length / 3;

// -- Kennzahlen einsetzen ---------------------------------------------------

const rows = [
  ["Punkte", INFO.count.toLocaleString("de-DE")],
  ["Scanebenen", String(INFO.planes)],
  ["Ausdehnung x", INFO.extent[0].toFixed(2) + " m"],
  ["Ausdehnung y", INFO.extent[1].toFixed(2) + " m"],
  ["Ausdehnung z", INFO.extent[2].toFixed(2) + " m"],
  ["Median-Radius", INFO.median_radius.toFixed(2) + " m"],
  ["Streuung Sektoren", INFO.sector_spread === null
      ? "–" : (INFO.sector_spread * 1000).toFixed(0) + " mm"],
];
// Wurde die Nulllage nachtraeglich gerichtet, muss das auf der Seite stehen -
// sonst sieht sie aus wie die Rohaufnahme und ist es nicht.
if (INFO.applied_alpha_zero_deg) {
  rows.push(["Nulllage korrigiert",
             INFO.applied_alpha_zero_deg.toFixed(0) + "°"]);
}
document.getElementById("readout").innerHTML =
  rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");

// -- Szene ------------------------------------------------------------------

const stage = document.getElementById("stage");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x0B1016, 1);
stage.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x0B1016, 0.035);
const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 500);

// Schwerpunkt und Groesse aus den Daten
let cx = 0, cy = 0, cz = 0;
for (let i = 0; i < count; i++) {
  cx += positions[i * 3]; cy += positions[i * 3 + 1]; cz += positions[i * 3 + 2];
}
cx /= count; cy /= count; cz /= count;

let reach = 0;
for (let i = 0; i < count; i++) {
  const dx = positions[i * 3] - cx;
  const dy = positions[i * 3 + 1] - cy;
  const dz = positions[i * 3 + 2] - cz;
  reach = Math.max(reach, Math.sqrt(dx * dx + dy * dy + dz * dz));
}

const geometry = new THREE.BufferGeometry();
geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
const colours = new Float32Array(count * 3);
geometry.setAttribute("color", new THREE.BufferAttribute(colours, 3));

const material = new THREE.PointsMaterial({
  size: 0.02, vertexColors: true, sizeAttenuation: true,
});
const cloud = new THREE.Points(geometry, material);
scene.add(cloud);

// Drehachse als Orientierung: der Scanner sass im Ursprung.
const axis = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0, -reach * 1.1), new THREE.Vector3(0, 0, reach * 1.1)]),
  new THREE.LineBasicMaterial({ color: 0x8A6425, transparent: true, opacity: 0.5 }));
scene.add(axis);

// -- Farbrampe: Petrol -> Bernstein ----------------------------------------

const RAMP = [
  [0.09, 0.25, 0.31], [0.18, 0.48, 0.55],
  [0.44, 0.70, 0.66], [0.85, 0.76, 0.44], [0.91, 0.64, 0.24],
];

function ramp(t, out, i) {
  t = Math.max(0, Math.min(0.9999, t));
  const scaled = t * (RAMP.length - 1);
  const k = Math.floor(scaled);
  const f = scaled - k;
  const a = RAMP[k], b = RAMP[k + 1];
  out[i] = a[0] + (b[0] - a[0]) * f;
  out[i + 1] = a[1] + (b[1] - a[1]) * f;
  out[i + 2] = a[2] + (b[2] - a[2]) * f;
}

function paint(mode) {
  const values = new Float32Array(count);
  for (let i = 0; i < count; i++) {
    const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
    if (mode === "height") values[i] = z;
    else if (mode === "plane") {
      let a = Math.atan2(y, x) * 180 / Math.PI;
      values[i] = ((a % 180) + 180) % 180;
    } else values[i] = Math.sqrt(x * x + y * y);
  }
  // Robuste Grenzen: die aeussersten 2 % duerfen die Rampe nicht auffressen.
  const sorted = Float32Array.from(values).sort();
  const lo = sorted[Math.floor(count * 0.02)];
  const hi = sorted[Math.floor(count * 0.98)];
  const span = (hi - lo) || 1;
  for (let i = 0; i < count; i++) ramp((values[i] - lo) / span, colours, i * 3);
  geometry.attributes.color.needsUpdate = true;
}
paint("radius");

// -- Kamera: eigener Orbit, damit keine weitere Abhaengigkeit noetig ist ----

const target = new THREE.Vector3(cx, cy, cz);
const home = { azimuth: 0.9, elevation: 0.35, distance: reach * 2.4 };
let view = Object.assign({}, home);

function place() {
  const e = Math.max(-1.5, Math.min(1.5, view.elevation));
  camera.position.set(
    target.x + view.distance * Math.cos(e) * Math.sin(view.azimuth),
    target.y + view.distance * Math.cos(e) * Math.cos(view.azimuth),
    target.z + view.distance * Math.sin(e));
  camera.up.set(0, 0, 1);
  camera.lookAt(target);
}

let dragging = false, lastX = 0, lastY = 0;
renderer.domElement.addEventListener("pointerdown", (e) => {
  dragging = true; lastX = e.clientX; lastY = e.clientY;
  renderer.domElement.setPointerCapture(e.pointerId);
});
renderer.domElement.addEventListener("pointerup", (e) => {
  dragging = false;
  try { renderer.domElement.releasePointerCapture(e.pointerId); } catch (_) {}
});
renderer.domElement.addEventListener("pointermove", (e) => {
  if (!dragging) return;
  view.azimuth -= (e.clientX - lastX) * 0.006;
  view.elevation = Math.max(-1.5, Math.min(1.5,
    view.elevation + (e.clientY - lastY) * 0.006));
  lastX = e.clientX; lastY = e.clientY;
});
renderer.domElement.addEventListener("wheel", (e) => {
  e.preventDefault();
  view.distance = Math.max(reach * 0.15, Math.min(reach * 8,
    view.distance * (1 + Math.sign(e.deltaY) * 0.12)));
}, { passive: false });

// -- Bedienung --------------------------------------------------------------

const calm = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
let spinning = !calm;
let speed = 0.35;

const spinButton = document.getElementById("spin");
spinButton.setAttribute("aria-pressed", String(spinning));
spinButton.addEventListener("click", () => {
  spinning = !spinning;
  spinButton.setAttribute("aria-pressed", String(spinning));
});
document.getElementById("speed").addEventListener("input", (e) => {
  speed = e.target.value / 100;
});
document.getElementById("colour").addEventListener("change", (e) => paint(e.target.value));
document.getElementById("size").addEventListener("input", (e) => {
  material.size = e.target.value / 200;
});
document.getElementById("reset").addEventListener("click", () => {
  view = Object.assign({}, home);
});
document.getElementById("top").addEventListener("click", () => {
  // Draufsicht: hier zeigt sich, ob eine Raumkontur Ecken hat oder rund ist.
  view = { azimuth: 0, elevation: 1.49, distance: reach * 2.2 };
});

function resize() {
  const w = window.innerWidth, h = window.innerHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);
resize();

let previous = performance.now();
function frame(now) {
  const dt = Math.min(0.1, (now - previous) / 1000);
  previous = now;
  if (spinning && !dragging) view.azimuth += dt * speed;
  place();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
</script>
"""


def to_ascii(html: str) -> str:
    """Alle Zeichen jenseits von ASCII maskieren.

    Notwendig, weil sich die Seite nicht darauf verlassen kann, dass ihre
    Zeichensatzangabe gelesen wird: beim Veroeffentlichen als Artifact wird ein
    Vorspann eingesetzt, wodurch das <meta charset> erst bei Byte 13000 steht.
    Browser suchen es aber nur in den ersten 1024 Byte und fallen sonst auf
    windows-1252 zurueck - aus "Zuruecksetzen" wird dann "ZurÃ¼cksetzen".

    Im Markup gehen dafuer Zeichenreferenzen, in <script> nicht: dort werden sie
    nicht aufgeloest und muessten als \\uXXXX in den Zeichenketten stehen.
    Deshalb wird der Text stueckweise behandelt.
    """
    out = []
    for index, part in enumerate(re.split(r"(<script\b[^>]*>.*?</script>|<style\b[^>]*>.*?</style>)", html,
                                          flags=re.DOTALL | re.IGNORECASE)):
        if index % 2 == 1:  # ein <script>- oder <style>-Block
            if part.lstrip()[:6].lower() == "<style":
                # In CSS gaebe es zwar \NN-Escapes, aber die Regeln zu Trennung
                # und Gross-/Kleinschreibung sind heikel. Der Stil hier ist
                # unser eigener und soll schlicht ASCII bleiben.
                stray = sorted({c for c in part if ord(c) >= 128})
                if stray:
                    raise ValueError(
                        "nicht-ASCII im <style>: " + " ".join(stray) +
                        " - im Stylesheet wirken weder Zeichenreferenzen noch "
                        "\\uXXXX; bitte umschreiben")
                out.append(part)
                continue
            out.append("".join(c if ord(c) < 128 else f"\\u{ord(c):04x}"
                               for c in part))
        else:
            out.append("".join(c if ord(c) < 128 else f"&#{ord(c)};"
                               for c in part))
    return "".join(out)


def build(points: List[Point], label: str, info: dict) -> str:
    flat = struct.pack(f"<{len(points) * 3}f",
                       *[c for p in points for c in p])
    level, title, text = verdict(info)

    page = (TEMPLATE
            .replace("__TITLE__", f"LiDAR-Scan {label}")
            .replace("__EYEBROW__", "Punktwolke · Handdrehung")
            .replace("__HEADLINE__", f"Scan {label}")
            .replace("__LEVEL_KEY__", level.replace("ä", "ae"))
            .replace("__LEVEL__", level)
            .replace("__VERDICT_TITLE__", title)
            .replace("__VERDICT_TEXT__", text)
            # json.dumps maskiert selbst schon nach ASCII.
            .replace("__INFO__", json.dumps(info))
            .replace("__DATA__", base64.b64encode(flat).decode("ascii")))
    return to_ascii(page)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ply", help="Eingabedatei")
    parser.add_argument("-o", "--output", default="viewer.html")
    parser.add_argument("--label", default=None,
                        help="Beschriftung; sonst der Dateiname")
    parser.add_argument("--alpha0", type=float, default=None, metavar="GRAD",
                        help="Nulllage des Scanwinkels nachtraeglich korrigieren: "
                             "jede Scanebene wird um diesen Winkel in sich "
                             "gedreht. 'auto' entspricht --auto-alpha0.")
    parser.add_argument("--auto-alpha0", action="store_true",
                        help="Nulllage aus den Daten schaetzen und anwenden - "
                             "gesucht wird der Winkel, bei dem Decke und Boden "
                             "waagerecht werden.")
    args = parser.parse_args(argv)

    points = read_ply(args.ply)

    delta = args.alpha0
    if args.auto_alpha0:
        guess = estimate_alpha_zero(points)
        if not guess["confident"]:
            print("Nulllage nicht schaetzbar: keine ausreichend grosse ebene "
                  "Flaeche gefunden. Unveraendert uebernommen.")
        else:
            delta = guess["alpha_zero_deg"]
            print(f"Nulllage geschaetzt: alpha_zero = {delta:.0f} Grad "
                  f"({guess['surface_points']} Punkte auf einer Hoehe, "
                  f"{guess['surface_height_m']:+.2f} m). Auf dem Kopf waere es "
                  f"{guess['mirrored_deg']:.0f} Grad.")
    if delta:
        points = rotate_cloud(points, delta)

    info = analyse(points)
    info["applied_alpha_zero_deg"] = delta or 0.0
    label = args.label or os.path.splitext(os.path.basename(args.ply))[0]

    with open(args.output, "w", encoding="utf-8") as fh:
        fh.write(build(points, label, info))

    level, title, _ = verdict(info)
    print(f"{info['count']} Punkte, {info['planes']} Ebenen -> {args.output}")
    print(f"Ausdehnung {info['extent'][0]:.2f} x {info['extent'][1]:.2f} x "
          f"{info['extent'][2]:.2f} m")
    if info["sector_spread"] is not None:
        print(f"Streuung ueber die Sektoren: {info['sector_spread'] * 1000:.0f} mm")
    print(f"Befund [{level}]: {title}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
