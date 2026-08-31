"""Streamlit-Oberflaeche fuer den RPLIDAR C1 am USB-Adapter.

    cd host
    pip install -r requirements-app.txt
    streamlit run app/streamlit_app.py

Drei Ansichten:

* **Live 2D** - die aktuelle Umdrehung als Schnitt durch den Raum.
* **3D aufnehmen** - Schritt fuer Schritt: Gierwinkel eintragen, Ebene
  aufnehmen, Wolke waechst. Genau der Ablauf, den die Firmware spaeter
  automatisch macht - nur dass die Achse hier von Hand gedreht wird.
* **Diagnose** - Geraeteinfo, Health, Resyncs, Messrate.

Ohne angeschlossene Hardware laeuft alles gegen einen simulierten Raum;
in der Seitenleiste umschaltbar.
"""

from __future__ import annotations

import csv
import io
import math
import os
import sys
import time

import plotly.graph_objects as go
import streamlit as st

# Das Paket liegt eine Ebene ueber diesem Ordner.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scan3d import ply, rplidar  # noqa: E402
from scan3d.geometry import MountGeometry, to_cartesian  # noqa: E402
from scan3d.reader import LidarReader  # noqa: E402
from scan3d.serial_source import BAUDRATE_C1  # noqa: E402

st.set_page_config(page_title="lidar3d - RPLIDAR C1", page_icon="🟢", layout="wide")


@st.cache_resource(show_spinner=False)
def get_reader(port: str, baudrate: int, simulate: bool, motor_rpm: int,
               room: tuple, scan_hz: float) -> LidarReader:
    reader = LidarReader(port, baudrate, simulate, motor_rpm, room, scan_hz)
    reader.start()
    return reader


# ---------------------------------------------------------------------------
# Seitenleiste
# ---------------------------------------------------------------------------

st.sidebar.title("lidar3d")
st.sidebar.caption("RPLIDAR C1 am USB-Adapter")

simulate = st.sidebar.toggle(
    "Simulierter Raum", value=True,
    help="Ohne Hardware ausprobieren. Ausschalten, sobald der C1 am USB haengt.")

@st.cache_data(ttl=5, show_spinner=False)
def available_ports() -> list:
    """Vorhandene serielle Ports. Leer, wenn pyserial fehlt."""
    try:
        from serial.tools import list_ports
    except Exception:
        return []
    return [(p.device, p.description or "") for p in list_ports.comports()]


MANUAL = "manuell eingeben …"
found = available_ports()

if simulate:
    port = ""
    st.sidebar.caption("Port wird im simulierten Betrieb nicht gebraucht.")
elif found:
    labels = [f"{dev}  ({desc})" if desc else dev for dev, desc in found]
    choice = st.sidebar.selectbox(
        "Serieller Port", labels + [MANUAL], index=0,
        help="Erkannte Ports. Der C1 heisst auf dem Mac meist "
             "/dev/tty.usbserial-XXXX, unter Linux /dev/ttyUSB0.")
    if choice == MANUAL:
        port = st.sidebar.text_input("Port von Hand", value=found[0][0])
    else:
        port = found[labels.index(choice)][0]
else:
    st.sidebar.warning("Kein serieller Port gefunden. Steckt der Adapter? "
                       "Ist pyserial installiert?")
    port = st.sidebar.text_input(
        "Serieller Port", value="/dev/ttyUSB0",
        help="Linux /dev/ttyUSB0, macOS /dev/tty.usbserial-XXXX, Windows COM5")
baudrate = st.sidebar.number_input(
    "Baudrate", value=BAUDRATE_C1, step=1,
    help="C1: 460800. S2/S3: 1000000.", disabled=simulate)
motor_rpm = st.sidebar.number_input(
    "Motordrehzahl setzen (0 = nicht senden)", value=0, min_value=0, max_value=1200,
    step=10, help="Der C1 laeuft normalerweise von selbst an. Nur setzen, wenn er stillsteht.",
    disabled=simulate)

st.sidebar.divider()
live = st.sidebar.toggle("Live aktualisieren", value=True)
refresh_s = st.sidebar.slider("Aktualisierung (s)", 0.1, 2.0, 0.3, 0.1)

st.sidebar.divider()
autoscale = st.sidebar.toggle(
    "Ansicht an Daten anpassen", value=True,
    help="Zoomt auf das, was tatsaechlich gemessen wird. Ausschalten, um den "
         "Massstab ueber mehrere Aufnahmen hinweg fest zu halten.")
max_range_m = st.sidebar.slider("Max. Reichweite (m)", 1.0, 15.0, 12.0, 0.5,
                                help="Filtert weiter entfernte Messungen weg. "
                                     "Der C1 schafft 12 m auf weissen Flaechen.")
min_range_mm = st.sidebar.slider("Blindzone (mm)", 0, 500, 150, 10)
min_quality = st.sidebar.slider("Mindestguete", 0, 63, 0,
                               help="0 laesst alles durch. Hoeher filtert schwache Echos.")
point_size = st.sidebar.slider("Punktgroesse", 1, 8, 3)

# Aendert sich eine dieser Einstellungen, muss der alte Leser weg, bevor der
# neue startet - sonst haelt sein Thread die serielle Schnittstelle weiter
# offen und der naechste Verbindungsversuch scheitert mit "resource busy".
settings = (port, int(baudrate), simulate, int(motor_rpm))
if st.session_state.get("_settings") not in (None, settings):
    previous = st.session_state.get("_reader")
    if previous is not None:
        previous.stop()
    get_reader.clear()
st.session_state._settings = settings

reader = get_reader(port, int(baudrate), simulate, int(motor_rpm),
                    (6.0, 4.0, 2.6), 10.0)
st.session_state._reader = reader

if st.sidebar.button("Neu verbinden", use_container_width=True):
    reader.stop()          # Serielle Schnittstelle freigeben ...
    get_reader.clear()     # ... bevor ein neuer Leser sie oeffnet.
    st.rerun()
stats = reader.stats()
revolution = reader.latest()


def keep(sample: rplidar.Sample) -> bool:
    return (sample.distance_mm > 0
            and sample.distance_mm >= min_range_mm
            and sample.distance_mm <= max_range_m * 1000
            and sample.quality >= min_quality)


# ---------------------------------------------------------------------------
# Kopfzeile
# ---------------------------------------------------------------------------

if stats.error:
    st.error(f"**Leser gestoppt:** {stats.error}")
elif not reader.running:
    st.warning("Der Leser laeuft nicht. Bitte auf 'Neu verbinden' druecken.")

cols = st.columns(5)
cols[0].metric("Umdrehungen", f"{stats.revolutions}")
cols[1].metric("Umdrehungen/s", f"{stats.rate_hz:.1f}")
cols[2].metric("Punkte je Umdrehung", f"{len(revolution) if revolution else 0}")
valid_share = (100.0 * stats.valid / stats.samples) if stats.samples else 0.0
cols[3].metric("Gueltige Messungen", f"{valid_share:.0f} %")
cols[4].metric("Resyncs", f"{stats.resyncs}")

tab_live, tab_3d, tab_diag = st.tabs(["Live 2D", "3D aufnehmen", "Diagnose"])


# ---------------------------------------------------------------------------
# Live 2D
# ---------------------------------------------------------------------------

with tab_live:
    if not revolution:
        st.info("Warte auf die erste vollstaendige Umdrehung …")
    else:
        pts = [s for s in revolution if keep(s)]
        if not pts:
            st.warning("Keine Messung ueberlebt die Filter. Blindzone oder "
                       "Mindestguete herabsetzen.")
        else:
            xs = [s.distance_mm / 1000 * math.sin(math.radians(s.angle_deg)) for s in pts]
            ys = [s.distance_mm / 1000 * math.cos(math.radians(s.angle_deg)) for s in pts]
            dist = [s.distance_mm / 1000 for s in pts]

            if autoscale:
                # Auf halbe Meter aufrunden, damit die Achse beim Live-Betrieb
                # nicht von Bild zu Bild zappelt.
                extent = max(max(abs(v) for v in xs), max(abs(v) for v in ys))
                view = max(0.5, math.ceil(extent * 2.2) / 2)
            else:
                view = max_range_m

            ring_step = 1 if view <= 6 else 2
            figure = go.Figure()
            # Entfernungsringe als Orientierung.
            for radius in range(ring_step, int(view) + 1, ring_step):
                figure.add_shape(type="circle", xref="x", yref="y",
                                 x0=-radius, y0=-radius, x1=radius, y1=radius,
                                 line=dict(color="rgba(120,120,120,0.35)", width=1))
            figure.add_trace(go.Scattergl(
                x=xs, y=ys, mode="markers",
                marker=dict(size=point_size, color=dist, colorscale="Viridis",
                            cmin=0, cmax=max(dist) if autoscale else max_range_m,
                            colorbar=dict(title="m")),
                hovertemplate="%{customdata[0]:.2f}° · %{customdata[1]:.2f} m"
                              "<extra></extra>",
                customdata=[[s.angle_deg, s.distance_mm / 1000] for s in pts],
            ))
            figure.update_layout(
                height=620, margin=dict(l=10, r=10, t=10, b=10),
                xaxis=dict(title="m", range=[-view, view],
                           scaleanchor="y", scaleratio=1, zeroline=True),
                yaxis=dict(title="m", range=[-view, view], zeroline=True),
                showlegend=False,
            )
            st.plotly_chart(figure, use_container_width=True)
            st.caption(
                f"{len(pts)} von {len(revolution)} Messungen dargestellt. "
                f"Der Sensor sitzt im Ursprung; die Ringe stehen alle "
                f"{ring_step} m. Weiteste Messung {max(dist):.2f} m.")


# ---------------------------------------------------------------------------
# 3D aufnehmen
# ---------------------------------------------------------------------------

if "cloud" not in st.session_state:
    st.session_state.cloud = []       # (x, y, z) in Metern
if "planes" not in st.session_state:
    st.session_state.planes = []      # (yaw_deg, punkte)
if "rows" not in st.session_state:
    st.session_state.rows = []        # Rohzeilen fuer den CSV-Export

with tab_3d:
    st.markdown(
        "Ablauf wie bei der Firmware, nur von Hand: Achse auf einen Gierwinkel "
        "stellen, Winkel hier eintragen, **Ebene aufnehmen**. "
        "**180° genuegen für die volle Kugel**, weil der LiDAR in seiner Ebene "
        "bereits 360° misst.")

    left, right = st.columns([1, 2])

    with left:
        yaw = st.number_input("Gierwinkel (Grad)", step=1.0,
                              min_value=-360.0, max_value=360.0, key="yaw_deg")
        yaw_step = st.number_input("Schrittweite (Grad)", value=5.0, step=1.0,
                                   min_value=0.1, max_value=45.0)
        offset_radial = st.number_input(
            "Versatz radial (mm)", value=0.0, step=1.0,
            help="Abstand des optischen Zentrums von der Drehachse. Falsch "
                 "eingestellt kruemmt er ebene Waende - siehe docs/02-geometrie.md.")
        offset_axial = st.number_input("Versatz axial (mm)", value=0.0, step=1.0)
        alpha_zero = st.number_input("alpha_zero (Grad)", value=0.0, step=1.0,
                                     help="LiDAR-Winkel, der nach oben zeigt.")
        alpha_sign = 1 if st.radio("Drehsinn alpha", ["normal", "gespiegelt"],
                                   horizontal=True) == "normal" else -1

        capture = st.button("Ebene aufnehmen", type="primary",
                            use_container_width=True, disabled=not revolution)
        advance = st.checkbox("Winkel danach weiterzaehlen", value=True)
        if st.button("Wolke leeren", use_container_width=True):
            st.session_state.cloud = []
            st.session_state.planes = []
            st.session_state.rows = []
            st.rerun()

    if capture and revolution:
        mount = MountGeometry(offset_radial_mm=offset_radial,
                              offset_axial_mm=offset_axial,
                              alpha_zero_deg=alpha_zero,
                              alpha_sign=alpha_sign)
        added = 0
        for s in revolution:
            if not keep(s):
                continue
            st.session_state.cloud.append(
                to_cartesian(s.distance_mm, s.angle_deg, yaw, mount))
            st.session_state.rows.append(
                [s.quality, f"{s.angle_deg:.4f}", f"{s.distance_mm:.1f}", f"{yaw:.3f}"])
            added += 1
        st.session_state.planes.append((yaw, added))
        if advance:
            # Wert des Widgets fuer den naechsten Durchlauf setzen.
            st.session_state.yaw_deg = min(360.0, yaw + yaw_step)
        st.rerun()

    with right:
        cloud = st.session_state.cloud
        st.metric("Punkte in der Wolke", f"{len(cloud)}")
        if st.session_state.planes:
            angles = [f"{a:g}°" for a, _ in st.session_state.planes]
            st.caption(f"{len(angles)} Ebenen: {', '.join(angles[:20])}"
                       + (" …" if len(angles) > 20 else ""))

        if cloud:
            # Im Browser nicht mehr als noetig zeichnen.
            limit = 60000
            shown = cloud[::max(1, len(cloud) // limit)]
            figure3d = go.Figure(go.Scatter3d(
                x=[p[0] for p in shown], y=[p[1] for p in shown],
                z=[p[2] for p in shown], mode="markers",
                marker=dict(size=max(1, point_size - 1),
                            color=[p[2] for p in shown],
                            colorscale="Viridis", opacity=0.85),
            ))
            figure3d.update_layout(
                height=560, margin=dict(l=0, r=0, t=0, b=0),
                scene=dict(aspectmode="data",
                           xaxis_title="x (m)", yaxis_title="y (m)",
                           zaxis_title="z (m)"))
            st.plotly_chart(figure3d, use_container_width=True)
            if len(shown) < len(cloud):
                st.caption(f"{len(shown)} von {len(cloud)} Punkten gezeichnet "
                           "(Anzeige ausgeduennt, der Export enthaelt alles).")

            ply_bytes = ply.dumps_ply(cloud, ply.height_colors(cloud))

            dl1, dl2 = st.columns(2)
            dl1.download_button("PLY herunterladen", ply_bytes,
                                file_name="scan.ply", mime="application/octet-stream",
                                use_container_width=True)

            csv_buffer = io.StringIO()
            writer = csv.writer(csv_buffer)
            writer.writerow(["Quality", "Angle (degrees)", "Distance (mm)", "Rotation"])
            writer.writerows(st.session_state.rows)
            dl2.download_button(
                "CSV herunterladen", csv_buffer.getvalue(),
                file_name="scanData.csv", mime="text/csv", use_container_width=True,
                help="Gleiche Spalten wie das Instructables-Skript, damit "
                     "convertAdjust.py die Datei direkt lesen kann.")
        else:
            st.info("Noch keine Ebene aufgenommen.")


# ---------------------------------------------------------------------------
# Diagnose
# ---------------------------------------------------------------------------

with tab_diag:
    st.subheader("Geraet")
    if stats.info:
        st.json(stats.info)
    else:
        st.caption("Keine Geraeteinfo gelesen.")

    st.subheader("Zahlen zum C1")
    st.markdown(
        """
| | RPLIDAR C1 |
|---|---|
| Messrate | 5.000/s |
| Scanrate | 8–12 Hz (typisch 10) |
| Punkte je Umdrehung | ~500 bei 10 Hz |
| Winkelaufloesung | ~0,72° |
| Reichweite | 12 m (weiss) |
| Schnittstelle | TTL UART, 460.800 Baud |

Bei 5 Byte je Messung sind das **25 kB/s** von rund 46 kB/s Leitungskapazität —
der einfache Scanmodus genügt, ein Express-Modus wird nicht gebraucht.
        """)

    if revolution:
        qualities = [s.quality for s in revolution]
        dists = [s.distance_mm for s in revolution if s.distance_mm > 0]
        c1, c2 = st.columns(2)
        with c1:
            st.caption("Verteilung der Signalgüte")
            st.plotly_chart(go.Figure(go.Histogram(x=qualities, nbinsx=32))
                            .update_layout(height=260,
                                           margin=dict(l=10, r=10, t=10, b=10)),
                            use_container_width=True)
        with c2:
            st.caption("Verteilung der Distanzen (mm)")
            st.plotly_chart(go.Figure(go.Histogram(x=dists, nbinsx=40))
                            .update_layout(height=260,
                                           margin=dict(l=10, r=10, t=10, b=10)),
                            use_container_width=True)


# ---------------------------------------------------------------------------
# Live-Schleife
# ---------------------------------------------------------------------------

if live and reader.running and not stats.error:
    time.sleep(refresh_s)
    st.rerun()
