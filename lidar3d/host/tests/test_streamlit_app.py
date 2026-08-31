"""Die Streamlit-App einmal komplett durchlaufen lassen.

Gegen eine Attrappe (tests/fake_streamlit.py), die vor allem die
Zustandsregeln nachbildet. Faengt genau die Fehlerklasse ab, die sonst erst
beim Benutzer auftritt: Reihenfolgefehler beim Session-State und falsche
Widget-Vorgaben.
"""

import os
import runpy
import sys
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_streamlit import (  # noqa: E402
    FakeGraphObjects, FakeStreamlit, Rerun, StreamlitAPIException,
)

APP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "app", "streamlit_app.py")


def run_app(st: FakeStreamlit) -> FakeStreamlit:
    """Die App einmal ausfuehren. Ein st.rerun() beendet den Durchlauf."""
    plotly = types.ModuleType("plotly")
    graph_objects = types.ModuleType("plotly.graph_objects")
    for name in ("Figure", "Scattergl", "Scatter3d", "Histogram"):
        setattr(graph_objects, name, getattr(FakeGraphObjects, name))
    plotly.graph_objects = graph_objects

    st.begin_run()
    saved = {k: sys.modules.get(k) for k in ("streamlit", "plotly", "plotly.graph_objects")}
    sys.modules["streamlit"] = st
    sys.modules["plotly"] = plotly
    sys.modules["plotly.graph_objects"] = graph_objects
    try:
        runpy.run_path(APP, run_name="__main__")
    except Rerun:
        pass
    finally:
        for key, module in saved.items():
            if module is None:
                sys.modules.pop(key, None)
            else:
                sys.modules[key] = module
    return st


class TestAppRuns(unittest.TestCase):
    def setUp(self):
        self.st = FakeStreamlit()
        # Ohne Live-Schleife, sonst endet jeder Durchlauf in st.rerun().
        self.st.overrides["Live aktualisieren"] = False

    def test_runs_end_to_end_in_simulation(self):
        run_app(self.st)
        self.assertEqual(self.st.errors, [], f"unerwartete Fehler: {self.st.errors}")
        self.assertGreater(self.st.charts, 0, "es wurde nichts gezeichnet")

    def test_yaw_starts_at_zero_not_at_the_minimum(self):
        """number_input ohne value beginnt sonst bei min_value, also -360."""
        run_app(self.st)
        self.assertEqual(self.st.session_state["yaw_deg"], 0.0)

    def test_capturing_a_plane_advances_the_yaw_angle(self):
        """Der Fehler, der beim Benutzer auftrat: Widget-Key nach dem Widget
        gesetzt. Zulaessig ist nur das Vormerken plus Neulauf."""
        self.st.button_returns["Ebene aufnehmen"] = True
        self.st.overrides["Schrittweite (Grad)"] = 5.0

        run_app(self.st)   # nimmt auf, merkt vor, loest Neulauf aus
        self.assertEqual(self.st.rerun_count, 1)
        self.assertEqual(self.st.session_state["_pending_yaw"], 5.0)

        # Zweiter Durchlauf: der vorgemerkte Winkel wird angewendet.
        self.st.button_returns["Ebene aufnehmen"] = False
        run_app(self.st)
        self.assertEqual(self.st.session_state["yaw_deg"], 5.0)
        self.assertNotIn("_pending_yaw", self.st.session_state)

    def test_capture_fills_the_cloud(self):
        self.st.button_returns["Ebene aufnehmen"] = True
        run_app(self.st)
        cloud = self.st.session_state["cloud"]
        rows = self.st.session_state["rows"]
        self.assertGreater(len(cloud), 400, "eine Ebene sollte ~500 Punkte liefern")
        self.assertEqual(len(cloud), len(rows))
        # Punkte muessen in Metern und im simulierten Raum liegen (6 x 4 x 2.6).
        for x, y, z in cloud[:50]:
            self.assertLessEqual(abs(x), 3.1)
            self.assertLessEqual(abs(z), 1.4)

    def _trace_kinds(self):
        return [kind for fig in self.st.figures for kind in fig.kinds()]

    def test_3d_view_exists_even_before_the_first_plane(self):
        """Die drehbare Ansicht muss von Anfang an da sein, sonst wirkt sie
        schlicht verschwunden."""
        run_app(self.st)
        self.assertEqual(self.st.session_state["cloud"], [])
        self.assertIn("scatter3d", self._trace_kinds())

    def test_3d_view_shows_the_cloud_after_capturing(self):
        self.st.button_returns["Ebene aufnehmen"] = True
        run_app(self.st)                                  # nimmt auf, rerun
        self.st.button_returns["Ebene aufnehmen"] = False
        run_app(self.st)                                  # zeichnet

        figures3d = [f for f in self.st.figures if "scatter3d" in f.kinds()]
        self.assertEqual(len(figures3d), 1, "genau eine 3D-Ansicht erwartet")
        trace = figures3d[0].traces[0]
        self.assertGreater(len(trace.kw["x"]), 400)
        # Gleiche Achsenmasstaebe, sonst ist der Raum verzerrt.
        self.assertEqual(figures3d[0].layout["scene"]["aspectmode"], "data")

    def test_every_chart_is_drawn_exactly_once(self):
        """Gegenprobe zur Frage, ob etwas doppelt erscheint."""
        run_app(self.st)
        kinds = self._trace_kinds()
        self.assertEqual(kinds.count("scattergl"), 1, "Live-2D genau einmal")
        self.assertEqual(kinds.count("scatter3d"), 1, "3D genau einmal")
        self.assertEqual(kinds.count("histogram"), 2, "zwei Diagnose-Histogramme")

    def test_capture_twice_accumulates(self):
        self.st.button_returns["Ebene aufnehmen"] = True
        run_app(self.st)
        first = len(self.st.session_state["cloud"])
        run_app(self.st)
        self.assertGreater(len(self.st.session_state["cloud"]), first)
        self.assertEqual(len(self.st.session_state["planes"]), 2)

    def test_setting_a_widget_key_after_instantiation_is_caught(self):
        """Beweis, dass die Attrappe die Regel wirklich durchsetzt."""
        self.st._widget("Test", "demo", 1.0)
        with self.assertRaises(StreamlitAPIException):
            self.st.session_state.demo = 2.0


if __name__ == "__main__":
    unittest.main()
