"""Die Seite, die der Scanner ausliefert, auf grobe Fehler pruefen.

Sie steckt als Zeichenkette in firmware/src/web_ui.h und wird von nichts
uebersetzt - kein Compiler sagt einem, wenn sie kaputt ist. Aufgefallen waere
es erst am iPhone, mit dem Geraet auf dem Tisch.

Das hier ersetzt kein Ausprobieren im Browser. Es faengt die Fehlerklasse ab,
die beim Umbauen entsteht und die man auf einem kleinen Bildschirm leicht
uebersieht: doppelte Bezeichner, ins Leere greifende Zugriffe, kaputte
Verschachtelung. Genau so einer war schon drin - ein Knopf und ein neuer
Container hiessen beide "top", und der Knopf haette nichts mehr getan.
"""

import collections
import html.parser
import os
import re
import unittest

PAGE_SOURCE = os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir,
    "firmware", "src", "web_ui.h"
)


def load_page():
    with open(PAGE_SOURCE, "r", encoding="utf-8") as fh:
        source = fh.read()
    return source.split('R"HTML(')[1].rsplit(')HTML"', 1)[0]


class _Nesting(html.parser.HTMLParser):
    """Nur so viel Pruefung, wie ohne echten Parser sinnvoll ist."""

    VOID = {"meta", "link", "br", "img", "input", "hr", "source"}

    def __init__(self):
        super().__init__()
        self.stack = []
        self.mismatched = []

    def handle_starttag(self, tag, attrs):
        if tag not in self.VOID:
            self.stack.append(tag)

    def handle_endtag(self, tag):
        if self.stack and self.stack[-1] == tag:
            self.stack.pop()
        else:
            self.mismatched.append(tag)


class TestWebUi(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.page = load_page()
        cls.ids = re.findall(r'\bid="([^"]+)"', cls.page)

    def test_ids_are_unique(self):
        counts = collections.Counter(self.ids)
        self.assertEqual([name for name, n in counts.items() if n > 1], [])

    def test_every_lookup_finds_an_element(self):
        used = set(re.findall(r'\$\("([^"]+)"\)', self.page))
        used |= set(re.findall(r'getElementById\("([^"]+)"\)', self.page))
        self.assertEqual(sorted(used - set(self.ids)), [])

    def test_tags_are_balanced(self):
        check = _Nesting()
        check.feed(self.page)
        self.assertEqual(check.mismatched, [])
        self.assertEqual(check.stack, [])

    def test_nothing_is_loaded_from_the_network(self):
        """Am Accesspoint des Scanners gibt es kein Internet.

        Ein <script src> oder eine Schriftart von einem CDN wuerde die Seite
        nicht kaputtmachen, sondern schlimmer: sie laedt, sieht falsch aus
        oder bleibt leer, und niemand weiss warum.
        """
        for pattern in (r'src\s*=\s*"https?:', r'href\s*=\s*"https?:',
                        r'@import', r'fonts\.googleapis'):
            self.assertIsNone(re.search(pattern, self.page), pattern)

    def test_page_fits_comfortably_in_flash(self):
        # Sie liegt als PROGMEM-Zeichenkette in der Firmware. 64 kB waeren
        # noch kein Problem, aber ein Sprung darueber waere ein Versehen.
        self.assertLess(len(self.page), 64 * 1024)

    def test_websocket_uses_the_serving_host(self):
        """Fest verdrahtet waere die Adresse beim ersten Umzug falsch."""
        self.assertIn("location.hostname", self.page)

    def test_points_arrive_as_binary_frames(self):
        """Ein Textframe je Punkt waere bei 5000 Messungen/s nicht zu halten."""
        self.assertIn("arraybuffer", self.page)
        self.assertIn("getFloat32", self.page)


if __name__ == "__main__":
    unittest.main()
