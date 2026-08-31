"""Minimale Streamlit-Attrappe, um die App einmal komplett durchlaufen zu lassen.

Streamlit selbst laesst sich in einem Test nicht sinnvoll starten. Die Fehler,
die in dieser App auftreten koennen, haengen aber fast alle am Zustandsmodell
und nicht am Rendern - vor allem an dieser Regel:

    Ein Widget-Key darf nicht mehr veraendert werden, nachdem das Widget im
    selben Durchlauf erzeugt wurde.

Genau die bildet die Attrappe nach, samt Vorgabewerten der Widgets. Damit
laufen Reihenfolgefehler im Test auf, statt erst beim Benutzer.
"""

from __future__ import annotations

from collections.abc import MutableMapping
from typing import Any, Callable, Dict, List, Optional


class StreamlitAPIException(Exception):
    pass


class Rerun(Exception):
    """st.rerun() bricht den Durchlauf ab - hier ueber eine Ausnahme."""


class SessionState(MutableMapping):
    def __init__(self) -> None:
        self._data: Dict[str, Any] = {}
        self.instantiated: set = set()

    # -- Mapping -----------------------------------------------------------

    def __getitem__(self, key): return self._data[key]
    def __iter__(self): return iter(self._data)
    def __len__(self): return len(self._data)
    def __delitem__(self, key): del self._data[key]

    def __setitem__(self, key, value):
        if key in self.instantiated:
            raise StreamlitAPIException(
                f"`st.session_state.{key}` cannot be modified after the widget "
                f"with key `{key}` is instantiated.")
        self._data[key] = value

    # -- Attributzugriff ---------------------------------------------------

    def __getattr__(self, key):
        try:
            return self.__dict__["_data"][key]
        except KeyError as exc:
            raise AttributeError(key) from exc

    def __setattr__(self, key, value):
        if key in ("_data", "instantiated"):
            super().__setattr__(key, value)
        else:
            self[key] = value


class FakeStreamlit:
    """Bildet die von der App benutzte Oberflaeche nach."""

    def __init__(self) -> None:
        self.session_state = SessionState()
        self.errors: List[str] = []
        self.warnings: List[str] = []
        #: Label -> Rueckgabewert, um Knopfdruecke zu erzwingen.
        self.button_returns: Dict[str, bool] = {}
        #: Label -> Wert, um Eingaben zu ueberschreiben.
        self.overrides: Dict[str, Any] = {}
        self.rerun_count = 0
        self.charts = 0
        self.figures: List[Any] = []
        self.fragments = 0
        self.sidebar = self  # dieselbe Oberflaeche, das genuegt hier

    def begin_run(self) -> None:
        """Vor jedem Skriptlauf aufrufen.

        Die Sperre gilt in Streamlit je Durchlauf: die Werte im Session-State
        ueberleben, die Markierung "Widget wurde erzeugt" nicht.
        """
        self.session_state.instantiated.clear()
        self.errors.clear()
        self.warnings.clear()
        self.charts = 0
        self.figures.clear()

    # -- Zustand der Widgets ----------------------------------------------

    def _widget(self, label: str, key: Optional[str], default: Any) -> Any:
        if label in self.overrides:
            default = self.overrides[label]
        if key is None:
            return default
        if key in self.session_state:
            value = self.session_state[key]
        else:
            self.session_state._data[key] = default
            value = default
        # Ab hier ist der Key gesperrt - genau wie im echten Streamlit.
        self.session_state.instantiated.add(key)
        return value

    # -- Layout ------------------------------------------------------------

    def set_page_config(self, **kw): pass
    def title(self, *a, **kw): pass
    def caption(self, *a, **kw): pass
    def markdown(self, *a, **kw): pass
    def subheader(self, *a, **kw): pass
    def divider(self, *a, **kw): pass
    def json(self, *a, **kw): pass
    def metric(self, *a, **kw): pass
    def info(self, *a, **kw): pass
    def progress(self, *a, **kw): pass

    def warning(self, msg="", *a, **kw): self.warnings.append(str(msg))
    def error(self, msg="", *a, **kw): self.errors.append(str(msg))

    def columns(self, spec, **kw):
        count = spec if isinstance(spec, int) else len(spec)
        return [self] * count

    def tabs(self, labels, **kw):
        return [self] * len(labels)

    def plotly_chart(self, figure=None, *a, **kw):
        self.charts += 1
        self.figures.append(figure)

    def __enter__(self): return self
    def __exit__(self, *exc): return False

    # -- Eingaben ----------------------------------------------------------

    def toggle(self, label, value=False, key=None, **kw):
        return self._widget(label, key, value)

    def checkbox(self, label, value=False, key=None, **kw):
        return self._widget(label, key, value)

    def button(self, label, *a, key=None, **kw):
        return self.button_returns.get(label, False)

    def download_button(self, label, data, *a, **kw):
        return self.button_returns.get(label, False)

    def text_input(self, label, value="", key=None, **kw):
        return self._widget(label, key, value)

    def number_input(self, label, value=None, min_value=None, key=None, **kw):
        # Streamlit nimmt ohne value das Minimum - deshalb hier genauso.
        default = value if value is not None else (
            min_value if min_value is not None else 0.0)
        return self._widget(label, key, default)

    def slider(self, label, min_value=None, max_value=None, value=None,
               step=None, key=None, **kw):
        default = value if value is not None else min_value
        return self._widget(label, key, default)

    def selectbox(self, label, options, index=0, key=None, **kw):
        options = list(options)
        default = options[index] if options else None
        return self._widget(label, key, default)

    def radio(self, label, options, index=0, key=None, **kw):
        options = list(options)
        return self._widget(label, key, options[index] if options else None)

    # -- Sonstiges ---------------------------------------------------------

    def fragment(self, func: Optional[Callable] = None, run_every=None, **kw):
        """Im Test einfach durchreichen - der Rumpf soll ganz normal laufen."""
        self.fragments += 1

        def decorate(fn):
            return fn

        return decorate if func is None else decorate(func)

    def rerun(self):
        self.rerun_count += 1
        raise Rerun()

    def cache_resource(self, func: Optional[Callable] = None, **kw):
        return self._cache(func, **kw)

    def cache_data(self, func: Optional[Callable] = None, **kw):
        return self._cache(func, **kw)

    def _cache(self, func=None, **kw):
        store: Dict[Any, Any] = {}

        def decorate(fn):
            def wrapper(*args, **kwargs):
                key = (args, tuple(sorted(kwargs.items())))
                if key not in store:
                    store[key] = fn(*args, **kwargs)
                return store[key]
            wrapper.clear = store.clear
            wrapper.__wrapped__ = fn
            return wrapper

        return decorate if func is None else decorate(func)


class _Trace:
    kind = "trace"

    def __init__(self, *a, **kw):
        self.kw = kw


class FakeGraphObjects:
    """So viel plotly, wie die App anfasst."""

    class Figure:
        def __init__(self, trace=None, *a, **kw):
            self.traces = [trace] if trace is not None else []
            self.layout = {}

        def add_trace(self, trace, **kw):
            self.traces.append(trace)
            return self

        def add_shape(self, **kw): return self

        def update_layout(self, **kw):
            self.layout.update(kw)
            return self

        def kinds(self): return [t.kind for t in self.traces]

    class Scattergl(_Trace): kind = "scattergl"
    class Scatter3d(_Trace): kind = "scatter3d"
    class Histogram(_Trace): kind = "histogram"
