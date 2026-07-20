#!/usr/bin/env python3
"""Erzeugt bedruckbare AprilTag-Bahnen aus einer YAML-Konfiguration.

Das Feld besteht aus ``n_lanes`` gleich breiten Bahnen, die entlang der x-Achse
verlaufen (Laenge ``length``) und in y-Richtung nebeneinander liegen (je
``lane_width`` breit). Feldgeometrie, Render-Optionen und die Markerliste
(Position und Groesse je Tag) werden aus einer YAML-Datei gelesen. Jede Bahn
wird als eigene PDF-Seite (Laenge x Bahnbreite) ausgegeben, sodass sie direkt
auf eine entsprechend breite Plotterrolle passt und kein Tag eine Bahngrenze
kreuzt.

Aufruf
------
    python3 Marker_generierung.py [konfig.yaml]

Ohne Argument wird ``marker_config.yaml`` im aktuellen Verzeichnis gelesen.

Konventionen
------------
Koordinatensystem
    Globaler Ursprung unten links. ``x`` entlang der Bahnlaenge, ``y`` ueber
    alle Bahnen hinweg. Markerposition = Tag-Mitte. Bahn b deckt
    y in [b*lane_width, (b+1)*lane_width].

Markergroesse
    ``size`` ist die Kantenlaenge des schwarzen Tag-Quadrats (= ``tagsize`` in
    Isaac ROS). Die weisse Quiet-Zone wird zusaetzlich nach aussen gezeichnet.

180-Grad-Rotation
    Jeder Marker wird standardmaessig um 180 Grad gedreht (Feld
    ``rotate_180``). Das laesst die ID unveraendert, dreht aber die kanonische
    Orientierung um pi um die z-Achse.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from itertools import combinations
from typing import Any, Iterable, Optional

import cv2
import numpy as np
import yaml
from PIL import Image
from reportlab.pdfgen import canvas
from reportlab.lib.colors import black, white, Color
from reportlab.lib.utils import ImageReader

# Punkte pro Meter (PostScript-Punkt = 1/72 Zoll).
M_TO_PT: float = 72.0 / 0.0254

# Schriftgroesse der Tag-Unterschrift (ID + Groesse) in Punkten.
# Zentrale Stelle: hier aendern, um alle Tag-Unterschriften zu skalieren.
# Dieser Wert ist massgeblich; ein ``output.label_pt`` in der YAML wird
# bewusst ignoriert, damit es genau eine Stellschraube gibt.
LABEL_PT: float = 15.0

_FAMILY_TO_DICT: dict[str, int] = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


@dataclass
class LaneField:
    """Bodenfeld aus mehreren nebeneinander liegenden Bahnen (in Metern).

    Attributes
    ----------
    length : float
        Laenge der Bahnen entlang der x-Achse.
    lane_width : float
        Breite einer einzelnen Bahn (y-Richtung).
    n_lanes : int
        Anzahl der Bahnen. Gesamtbreite = ``n_lanes * lane_width``.
    """

    length: float
    lane_width: float
    n_lanes: int

    @property
    def width(self) -> float:
        """Gesamtbreite des Feldes in Metern."""
        return self.lane_width * self.n_lanes

    @property
    def edges(self) -> list[float]:
        """Alle y-Linien inkl. Aussenraender (Laenge n_lanes+1)."""
        return [k * self.lane_width for k in range(self.n_lanes + 1)]

    @property
    def internal_boundaries(self) -> list[float]:
        """Innere Bahngrenzen (Stosskanten) in y."""
        return [k * self.lane_width for k in range(1, self.n_lanes)]


@dataclass
class Marker:
    """Ein AprilTag-Marker im globalen Feld-Koordinatensystem.

    Attributes
    ----------
    id : int
        Tag-ID innerhalb der Familie.
    size : float
        Kantenlaenge des schwarzen Tag-Quadrats in Metern.
    x : float
        x-Koordinate der Markermitte in Metern.
    y : float
        globale y-Koordinate der Markermitte in Metern.
    family : str
        AprilTag-Familie.
    rotate_180 : bool
        Marker um 180 Grad drehen (aendert nur die Orientierung, nicht die ID).
    """

    id: int
    size: float
    x: float
    y: float
    family: str = "tag36h11"
    rotate_180: bool = True


_DICTS: dict[str, cv2.aruco.Dictionary] = {}


def _dictionary(family: str) -> cv2.aruco.Dictionary:
    """Liefert (gecached) das aruco-Dictionary einer AprilTag-Familie.

    Parameters
    ----------
    family : str
        AprilTag-Familie.

    Returns
    -------
    cv2.aruco.Dictionary
        Vordefiniertes OpenCV-Dictionary.

    Raises
    ------
    ValueError
        Wenn die Familie nicht unterstuetzt wird.
    """
    if family not in _FAMILY_TO_DICT:
        raise ValueError(f"Familie {family!r} nicht unterstuetzt. "
                         f"Moeglich: {sorted(_FAMILY_TO_DICT)}")
    if family not in _DICTS:
        _DICTS[family] = cv2.aruco.getPredefinedDictionary(
            _FAMILY_TO_DICT[family])
    return _DICTS[family]


def _n_modules(family: str) -> int:
    """Modulanzahl der schwarzen Kante (Datenbits + 1 Randmodul je Seite)."""
    return _dictionary(family).markerSize + 2


def _marker_image(family: str, tag_id: int, px_per_module: int,
                  rotate_180: bool, quiet_modules: float
                  ) -> tuple[np.ndarray, int, int]:
    """Erzeugt das Markerbild eines Tags mit dem OpenCV-aruco-Modul.

    ``generateImageMarker`` mit ``borderBits=1``, optionale 180-Grad-Rotation,
    danach weisser Rand (Quiet-Zone) via ``copyMakeBorder``.

    Parameters
    ----------
    family : str
        AprilTag-Familie.
    tag_id : int
        Tag-ID.
    px_per_module : int
        Pixel pro Modul. Die schwarze Kante ist ein ganzzahliges Vielfaches
        der Modulanzahl, damit alle Module exakt gleich breit sind.
    rotate_180 : bool
        Marker um 180 Grad drehen.
    quiet_modules : float
        Breite des weissen Randes in Modulen (je Seite).

    Returns
    -------
    tuple[numpy.ndarray, int, int]
        RGBA-Bild (uint8). Die Quiet-Zone ist transparent (alpha=0), damit sie
        beim dichten Platzieren keine benachbarten Tags uebermalt; auf weissem
        Hintergrund wirkt sie als weisser Rand. Zweiter und dritter Rueckgabe-
        wert: Kantenlaenge der schwarzen Region und Randbreite in Pixeln.

    Raises
    ------
    ValueError
        Wenn ``tag_id`` ausserhalb des gueltigen Bereichs der Familie liegt.
    """
    ad = _dictionary(family)
    n_ids = ad.bytesList.shape[0]
    if not (0 <= tag_id < n_ids):
        raise ValueError(f"ID {tag_id} ausserhalb 0..{n_ids - 1} "
                         f"fuer Familie {family}.")
    side_px = _n_modules(family) * int(px_per_module)
    tag = cv2.aruco.generateImageMarker(ad, tag_id, side_px, borderBits=1)
    if rotate_180:
        tag = cv2.rotate(tag, cv2.ROTATE_180)
    border_px = int(round(quiet_modules * px_per_module))
    if border_px > 0:
        rgb = cv2.copyMakeBorder(tag, border_px, border_px, border_px,
                                 border_px, cv2.BORDER_CONSTANT, value=255)
        # Tag-Region opak, Quiet-Zone transparent.
        alpha = cv2.copyMakeBorder(np.full_like(tag, 255), border_px, border_px,
                                   border_px, border_px, cv2.BORDER_CONSTANT,
                                   value=0)
    else:
        rgb = tag
        alpha = np.full_like(tag, 255)
    img = np.dstack([rgb, rgb, rgb, alpha])
    return img, side_px, border_px


def _grid_values(center: float, pitch: float, lo: float, hi: float
                 ) -> np.ndarray:
    """Rasterwerte ``center + k*pitch`` im Intervall [lo, hi]."""
    k = np.arange(np.ceil((lo - center) / pitch),
                  np.floor((hi - center) / pitch) + 1)
    return np.round(center + k * pitch, 6)


def build_grid_markers(field: LaneField, tag0_pos: tuple[float, float],
                       pitch: float, small_size: float,
                       family: str = "tag36h11", gap: float = 0.02,
                       large_id_start: int = 100, rotate_180: bool = True,
                       add_large: bool = True
                       ) -> tuple[list[Marker], float]:
    """Erzeugt Standard-Raster-Tags und grosse Zwischen-Tags (Hilfsfunktion).

    Praktisch, um mit ``save_markers_yaml`` eine Start-YAML zu erzeugen, die
    danach von Hand angepasst werden kann. Die Standard-Tags liegen im Raster
    ``tag0_pos + k*pitch`` und nur dort, wo ihre schwarze Kante vollstaendig
    auf einer Bahn liegt. In die Rasterluecken (um ``pitch/2`` versetzt) werden
    moeglichst grosse, einheitlich grosse Tags gesetzt. Tag 0 erhaelt
    ``tag0_pos``.

    Parameters
    ----------
    field : LaneField
        Feldgeometrie.
    tag0_pos : tuple[float, float]
        Position von Tag 0 (definiert zugleich das Raster).
    pitch : float
        Rasterabstand in Metern.
    small_size : float
        Kantenlaenge der Standard-Tags in Metern.
    family : str
        AprilTag-Familie.
    gap : float
        Mindestabstand zwischen schwarzen Kanten benachbarter Tags und zu
        Bahngrenzen/Feldraendern, in Metern.
    large_id_start : int
        Erste ID fuer die grossen Zwischen-Tags.
    rotate_180 : bool
        180-Grad-Rotation fuer alle erzeugten Marker.
    add_large : bool
        Grosse Zwischen-Tags erzeugen.

    Returns
    -------
    tuple[list[Marker], float]
        Markerliste und die gewaehlte einheitliche Groesse der grossen Tags
        (0.0, falls keine erzeugt wurden).
    """
    W, H = field.length, field.width
    edges = field.edges
    internal = field.internal_boundaries

    def clear(c: float, lines: list[float]) -> float:
        """Kleinster Abstand von c zu den gegebenen Linien."""
        return min(abs(c - l) for l in lines)

    def crosses_boundary(y: float, half: float) -> bool:
        """True, wenn [y-half, y+half] eine innere Bahngrenze enthaelt."""
        return any(y - half < b < y + half for b in internal)

    def in_field(c: float, half: float, hi: float) -> bool:
        """True, wenn [c-half, c+half] in [0, hi] liegt."""
        return c - half >= -1e-9 and c + half <= hi + 1e-9

    # --- Standard-Raster -------------------------------------------------
    xs = _grid_values(tag0_pos[0], pitch, 0, W)
    ys = _grid_values(tag0_pos[1], pitch, 0, H)
    half_s = small_size / 2.0
    std_pos = [(float(x), float(y)) for y in ys for x in xs
               if in_field(x, half_s, W) and in_field(y, half_s, H)
               and not crosses_boundary(y, half_s)]

    # --- Zwischenplaetze --------------------------------------------------
    large_size = 0.0
    inter_pos: list[tuple[float, float]] = []
    if add_large:
        xs2 = _grid_values(tag0_pos[0] + pitch / 2, pitch, 0, W)
        ys2 = _grid_values(tag0_pos[1] + pitch / 2, pitch, 0, H)
        min_clear = half_s + gap
        inter_pos = [(float(x), float(y)) for y in ys2 for x in xs2
                     if clear(y, edges) >= min_clear
                     and clear(x, [0, W]) >= min_clear]

        def max_size_at(x: float, y: float) -> float:
            """Maximale schwarze Kantenlaenge an (x, y) ohne Konflikt."""
            lims = [2 * (clear(y, edges) - gap), 2 * (clear(x, [0, W]) - gap)]
            for ox, oy in std_pos:
                d = max(abs(x - ox), abs(y - oy))
                if d < pitch:
                    lims.append(2 * d - small_size - 2 * gap)
            return min(lims)

        if inter_pos:
            place_lim = min(max_size_at(x, y) for x, y in inter_pos)
            gg = min(max(abs(a[0] - b[0]), abs(a[1] - b[1]))
                     for a, b in combinations(inter_pos, 2)) \
                if len(inter_pos) > 1 else float("inf")
            large_size = round(min(place_lim, gg - 2 * gap), 3)

    # --- IDs vergeben -----------------------------------------------------
    markers: list[Marker] = []
    markers.append(Marker(0, small_size, tag0_pos[0], tag0_pos[1], family,
                          rotate_180))
    rest = sorted((p for p in std_pos if p != tuple(tag0_pos)),
                  key=lambda p: (p[1], p[0]))
    for i, (x, y) in enumerate(rest, start=1):
        markers.append(Marker(i, small_size, x, y, family, rotate_180))
    if large_size > 0:
        for j, (x, y) in enumerate(sorted(inter_pos, key=lambda p: (p[1], p[0])),
                                   start=large_id_start):
            markers.append(Marker(j, large_size, x, y, family, rotate_180))
    return markers, large_size


def _lane_index(y: float, size: float, field: LaneField) -> Optional[int]:
    """Index der Bahn, deren Bereich die schwarze Kante vollstaendig enthaelt.

    Parameters
    ----------
    y : float
        globale y-Mitte des Tags.
    size : float
        schwarze Kantenlaenge.
    field : LaneField
        Feldgeometrie.

    Returns
    -------
    Optional[int]
        Bahnindex oder None, falls der Tag eine Bahngrenze kreuzt oder
        ausserhalb des Feldes liegt.
    """
    half = size / 2.0
    b = int(np.floor(y / field.lane_width + 1e-9))
    lo, hi = b * field.lane_width, (b + 1) * field.lane_width
    if 0 <= b < field.n_lanes and y - half >= lo - 1e-9 and y + half <= hi + 1e-9:
        return b
    return None


def validate_markers(field: LaneField, markers: list[Marker]) -> list[str]:
    """Prueft die Markerliste auf typische Konfigurationsfehler.

    Parameters
    ----------
    field : LaneField
        Feldgeometrie.
    markers : list[Marker]
        Zu pruefende Marker.

    Returns
    -------
    list[str]
        Warntexte (doppelte IDs, Feldueberschreitung, Bahngrenz-Kreuzung,
        ueberlappende schwarze Kanten). Leer, wenn nichts auffaellt.
    """
    warnings: list[str] = []
    seen_ids: dict[int, int] = {}
    for m in markers:
        seen_ids[m.id] = seen_ids.get(m.id, 0) + 1
    for tag_id, n in seen_ids.items():
        if n > 1:
            warnings.append(f"ID {tag_id} kommt {n}-mal vor (sollte eindeutig sein).")

    for m in markers:
        half = m.size / 2.0
        if m.x - half < -1e-9 or m.x + half > field.length + 1e-9:
            warnings.append(
                f"ID {m.id} ragt in x ueber das Feld hinaus "
                f"(x={m.x:.3f}, size={m.size:.3f}, Laenge={field.length}).")
        if _lane_index(m.y, m.size, field) is None:
            warnings.append(
                f"ID {m.id} kreuzt eine Bahngrenze oder liegt ausserhalb in y "
                f"(y={m.y:.3f}, size={m.size:.3f}).")

    for a, b in combinations(markers, 2):
        d = max(abs(a.x - b.x), abs(a.y - b.y))
        if d < (a.size + b.size) / 2.0 - 1e-9:
            warnings.append(
                f"ID {a.id} und ID {b.id} ueberlappen sich "
                f"(schwarze Kanten).")
    return warnings


def _draw_marker(c: canvas.Canvas, m: Marker, y_offset: float,
                 px_per_module: int, quiet_modules: float, label: bool,
                 label_pt: float, label_gap: float) -> None:
    """Zeichnet einen Marker (eingebettetes Bild) in lokale Bahnkoordinaten.

    Skaliert so, dass die schwarze Kante genau ``m.size`` misst; die Quiet-Zone
    ragt darueber hinaus. ``y_offset`` verschiebt die globale y-Koordinate in
    das lokale Bahnsystem.

    Parameters
    ----------
    c : reportlab.pdfgen.canvas.Canvas
        Ziel-Canvas (Einheit Punkte).
    m : Marker
        Zu zeichnender Marker (globale Koordinaten).
    y_offset : float
        Subtrahierter y-Versatz (untere Kante der Bahn) in Metern.
    px_per_module : int
        Pixel pro Modul des eingebetteten Bildes.
    quiet_modules : float
        Breite der Quiet-Zone in Modulen.
    label : bool
        Beschriftung unter dem Tag.
    label_pt : float
        Schriftgroesse in Punkten.
    label_gap : float
        Abstand der Beschriftung zur Quiet-Zone in Metern.
    """
    img, side_px, border_px = _marker_image(
        m.family, m.id, px_per_module, m.rotate_180, quiet_modules)
    yl = m.y - y_offset
    m_per_px = m.size / side_px
    total_m = (side_px + 2 * border_px) * m_per_px
    x0 = (m.x - total_m / 2.0) * M_TO_PT
    y0 = (yl - total_m / 2.0) * M_TO_PT
    side_total_pt = total_m * M_TO_PT
    c.drawImage(ImageReader(Image.fromarray(img)), x0, y0,
                width=side_total_pt, height=side_total_pt, mask="auto")
    if label:
        text = f"ID {m.id} | {m.size:.3f} m"
        ty = (yl - m.size / 2.0 - label_gap) * M_TO_PT - label_pt
        c.setFillColor(black)
        c.setFont("Helvetica", label_pt)
        c.drawCentredString(m.x * M_TO_PT, ty, text)


def _draw_registration_marks(c: canvas.Canvas, w_pt: float, h_pt: float,
                             arm_m: float = 0.05) -> None:
    """Zeichnet duenne Eckwinkel an den vier Seitenecken (Masskontrolle)."""
    a = arm_m * M_TO_PT
    c.setStrokeColor(Color(0.6, 0.6, 0.6))
    c.setLineWidth(0.3)
    for cx, cy, sx, sy in [(0, 0, 1, 1), (w_pt, 0, -1, 1),
                           (0, h_pt, 1, -1), (w_pt, h_pt, -1, -1)]:
        c.line(cx, cy, cx + sx * a, cy)
        c.line(cx, cy, cx, cy + sy * a)


def _draw_alignment_ticks(c: canvas.Canvas, field: LaneField,
                          spacing_m: float = 0.5, tick_m: float = 0.03,
                          label: bool = False, label_pt: float = 6.0,
                          tick_width: float = 1.5) -> None:
    """Zeichnet Ausricht-Striche senkrecht zu den Bahnkanten.

    An beiden Laengskanten der Bahn (lokal y=0 und y=lane_width) wird alle
    ``spacing_m`` entlang der Bahnlaenge ein kurzer Strich senkrecht zur Kante
    gesetzt (Laenge ``tick_m``, nach innen). Da jede Bahn-Seite dasselbe
    x-Raster nutzt, treffen sich beim Aneinanderlegen die Striche der oberen
    Kante von Bahn b und der unteren Kante von Bahn b+1 an derselben
    x-Position, sodass die Bahnen deckungsgleich ausgerichtet werden koennen.
    Zusaetzlich werden an den kurzen Kanten (x=0 und x=length) Striche
    senkrecht dazu gesetzt, um den Feldanfang/-abschluss ueber alle Bahnen
    auszurichten.

    Parameters
    ----------
    c : reportlab.pdfgen.canvas.Canvas
        Ziel-Canvas (Einheit Punkte).
    field : LaneField
        Feldgeometrie.
    spacing_m : float
        Abstand der Striche in Metern.
    tick_m : float
        Strichlaenge (senkrecht zur Kante, nach innen) in Metern.
    label : bool
        x- bzw. y-Wert klein neben den Strich schreiben.
    label_pt : float
        Schriftgroesse der Beschriftung in Punkten.
    tick_width : float
        Strichstaerke (Linienbreite) in PostScript-Punkten. Groessere Werte
        zeichnen die Ausricht-Striche dicker.
    """
    w = field.length * M_TO_PT
    h = field.lane_width * M_TO_PT
    t = tick_m * M_TO_PT
    c.setStrokeColor(black)
    c.setLineWidth(tick_width)

    # Laengskanten: Striche alle spacing_m entlang x, senkrecht (in y).
    n_x = int(round(field.length / spacing_m))
    for k in range(n_x + 1):
        xm = k * spacing_m
        xp = xm * M_TO_PT
        c.line(xp, 0.0, xp, t)          # untere Laengskante -> nach oben
        c.line(xp, h, xp, h - t)        # obere Laengskante  -> nach unten
        if label:
            c.setFont("Helvetica", label_pt)
            c.setFillColor(black)
            c.drawCentredString(xp, t + 1.0, f"{xm:.1f}")

    # Kurze Kanten: Striche alle spacing_m entlang y, senkrecht (in x).
    ys: list[float] = [k * spacing_m for k in
                       range(int(round(field.lane_width / spacing_m)) + 1)]
    if abs(ys[-1] - field.lane_width) > 1e-9:
        ys.append(field.lane_width)     # Kante der Bahn stets markieren
    for ym in ys:
        yp = ym * M_TO_PT
        c.line(0.0, yp, t, yp)          # linke kurze Kante  -> nach rechts
        c.line(w, yp, w - t, yp)        # rechte kurze Kante -> nach links


def create_lane_pdf(path: str, field: LaneField, markers: Iterable[Marker],
                    px_per_module: int = 100, quiet_modules: float = 1.0,
                    label: bool = True, label_pt: float = LABEL_PT,
                    label_gap: float = 0.008,
                    registration_marks: bool = True,
                    alignment_ticks: bool = True, tick_spacing: float = 0.5,
                    tick_len: float = 0.03, tick_label: bool = False,
                    tick_width: float = 1.5
                    ) -> list[str]:
    """Erzeugt die PDF mit einer Seite je Bahn.

    Jede Seite ist ``length`` x ``lane_width`` Meter gross. Jeder Marker wird
    der Bahn zugeordnet, die seine schwarze Kante vollstaendig enthaelt; kreuzt
    er eine Bahngrenze, wird er nicht gezeichnet und gemeldet.

    Parameters
    ----------
    path : str
        Ausgabepfad der PDF.
    field : LaneField
        Feldgeometrie.
    markers : Iterable[Marker]
        Zu platzierende Marker (globale Koordinaten).
    px_per_module : int
        Pixel pro Modul des eingebetteten Markerbildes.
    quiet_modules : float
        Breite der Quiet-Zone in Modulen.
    label : bool
        Beschriftung je Tag.
    label_pt : float
        Schriftgroesse der Beschriftung in Punkten.
    label_gap : float
        Abstand der Beschriftung zum Tag in Metern.
    registration_marks : bool
        Eckwinkel je Bahn-Seite.
    alignment_ticks : bool
        Ausricht-Striche senkrecht zu den Kanten (alle ``tick_spacing``).
    tick_spacing : float
        Abstand der Ausricht-Striche in Metern.
    tick_len : float
        Laenge der Ausricht-Striche (nach innen) in Metern.
    tick_label : bool
        Ausricht-Striche mit x-/y-Wert beschriften.
    tick_width : float
        Strichstaerke (Linienbreite) der Ausricht-Striche in PostScript-
        Punkten. Groesser = dicker.

    Returns
    -------
    list[str]
        Warnungen (Tags auf Bahngrenze). Leer, wenn alle Tags sauber liegen.
    """
    markers = list(markers)
    warnings: list[str] = []
    page = (field.length * M_TO_PT, field.lane_width * M_TO_PT)
    c = canvas.Canvas(path, pagesize=page)

    for b in range(field.n_lanes):
        y_off = b * field.lane_width
        c.setFillColor(white)
        c.rect(0, 0, page[0], page[1], stroke=0, fill=1)
        if registration_marks:
            _draw_registration_marks(c, page[0], page[1])
        if alignment_ticks:
            _draw_alignment_ticks(c, field, spacing_m=tick_spacing,
                                  tick_m=tick_len, label=tick_label,
                                  tick_width=tick_width)
        # Bahnbeschriftung oben links
        c.setFillColor(Color(0.5, 0.5, 0.5))
        c.setFont("Helvetica", 8)
        c.drawString(0.02 * M_TO_PT, (field.lane_width - 0.03) * M_TO_PT,
                     f"Bahn {b}  (global y = {y_off:.2f}..{y_off + field.lane_width:.2f} m)")
        for m in markers:
            if _lane_index(m.y, m.size, field) == b:
                _draw_marker(c, m, y_off, px_per_module, quiet_modules,
                             label, label_pt, label_gap)
        c.showPage()

    c.setTitle("AprilTag Lanes")
    c.save()

    for m in markers:
        if _lane_index(m.y, m.size, field) is None:
            warnings.append(
                f"Marker ID {m.id} (y={m.y:.3f}, size={m.size:.3f}) kreuzt eine "
                f"Bahngrenze und wurde nicht gezeichnet.")
    return warnings


def load_config(path: str) -> tuple[LaneField, list[Marker], dict[str, Any]]:
    """Liest Feldgeometrie, Marker und Render-Optionen aus einer YAML-Datei.

    Erwartetes YAML-Schema (siehe Beispieldatei)::

        field:    {length, lane_width, n_lanes}
        defaults: {family, rotate_180}          # optional
        output:   {path, px_per_module, ...}    # optional
        markers:
          - {id, x, y, size, family?, rotate_180?}

    ``family`` und ``rotate_180`` je Marker sind optional und fallen auf die
    Werte unter ``defaults`` zurueck.

    Parameters
    ----------
    path : str
        Pfad zur YAML-Datei.

    Returns
    -------
    tuple[LaneField, list[Marker], dict[str, Any]]
        Feldgeometrie, Markerliste und das ``output``-Dictionary (Render-
        Optionen; leer, falls nicht angegeben).

    Raises
    ------
    KeyError
        Wenn Pflichtfelder fehlen.
    """
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    if "field" not in cfg:
        raise KeyError("YAML: Abschnitt 'field' fehlt.")
    fc = cfg["field"]
    field = LaneField(length=float(fc["length"]),
                      lane_width=float(fc["lane_width"]),
                      n_lanes=int(fc["n_lanes"]))

    defaults = cfg.get("defaults") or {}
    def_family = defaults.get("family", "tag36h11")
    def_rotate = bool(defaults.get("rotate_180", True))

    if "markers" not in cfg or not cfg["markers"]:
        raise KeyError("YAML: Abschnitt 'markers' fehlt oder ist leer.")
    markers: list[Marker] = []
    for entry in cfg["markers"]:
        markers.append(Marker(
            id=int(entry["id"]),
            size=float(entry["size"]),
            x=float(entry["x"]),
            y=float(entry["y"]),
            family=entry.get("family", def_family),
            rotate_180=bool(entry.get("rotate_180", def_rotate)),
        ))

    output = cfg.get("output") or {}
    return field, markers, output


def save_markers_yaml(path: str, field: LaneField, markers: list[Marker],
                      output: Optional[dict[str, Any]] = None,
                      default_family: str = "tag36h11",
                      default_rotate_180: bool = True) -> None:
    """Schreibt Feldgeometrie und Marker als gut lesbare YAML-Datei.

    Erzeugt eine Datei im selben Schema, das ``load_config`` liest. Familie und
    ``rotate_180`` werden nur dann je Marker ausgegeben, wenn sie von den
    Defaults abweichen.

    Parameters
    ----------
    path : str
        Ausgabepfad.
    field : LaneField
        Feldgeometrie.
    markers : list[Marker]
        Zu schreibende Marker.
    output : Optional[dict[str, Any]]
        Render-Optionen fuer den ``output``-Abschnitt.
    default_family : str
        Familie fuer den ``defaults``-Abschnitt.
    default_rotate_180 : bool
        rotate_180 fuer den ``defaults``-Abschnitt.
    """
    out = output or {"path": "apriltag_lanes.pdf"}
    lines: list[str] = []
    lines.append("# AprilTag-Bahnen - Konfiguration")
    lines.append("# Alle Laengen in Metern. Markerposition = Tag-Mitte,")
    lines.append("# size = Kantenlaenge des schwarzen Quadrats (= tagsize).")
    lines.append("")
    lines.append("field:")
    lines.append(f"  length: {field.length}")
    lines.append(f"  lane_width: {field.lane_width}")
    lines.append(f"  n_lanes: {field.n_lanes}")
    lines.append("")
    lines.append("defaults:")
    lines.append(f"  family: {default_family}")
    lines.append(f"  rotate_180: {str(default_rotate_180).lower()}")
    lines.append("")
    lines.append("output:")
    for key, val in out.items():
        v = str(val).lower() if isinstance(val, bool) else val
        lines.append(f"  {key}: {v}")
    lines.append("")
    lines.append("markers:")
    for m in sorted(markers, key=lambda mk: mk.id):
        extra = ""
        if m.family != default_family:
            extra += f", family: {m.family}"
        if m.rotate_180 != default_rotate_180:
            extra += f", rotate_180: {str(m.rotate_180).lower()}"
        lines.append(f"  - {{id: {m.id:>3}, x: {m.x:.3f}, y: {m.y:.3f}, "
                     f"size: {m.size:.3f}{extra}}}")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def verify_lane_pdf(path: str, field: LaneField, markers: Iterable[Marker],
                    dpi: int = 150, tol_m: float = 0.003,
                    tile_m: float = 1.0, overlap_m: float = 0.3) -> bool:
    """Prueft die Bahn-PDF gegen einen echten AprilTag-Detektor.

    Jede Bahn-Seite wird in ueberlappenden Kacheln entlang x gerastert (sonst
    waeren die Bilder bei mehreren Metern Laenge zu gross), die Tags detektiert,
    in globale Koordinaten zurueckgerechnet und ueber die Kacheln dedupliziert.
    Verglichen werden ID, Mitte und schwarze Kante; zudem wird geprueft, dass
    jeder Tag genau einmal und nur auf seiner Bahn erscheint.

    Parameters
    ----------
    path : str
        Pfad der PDF.
    field : LaneField
        Feldgeometrie.
    markers : Iterable[Marker]
        Soll-Marker.
    dpi : int
        Rasteraufloesung je Kachel.
    tol_m : float
        Toleranz fuer Mitte und Groesse in Metern.
    tile_m : float
        Kachelbreite entlang x in Metern.
    overlap_m : float
        Ueberlappung benachbarter Kacheln in Metern.

    Returns
    -------
    bool
        True, wenn alle Tags korrekt, vollstaendig und bahnrein erkannt wurden.
    """
    import fitz
    from pupil_apriltags import Detector

    markers = list(markers)
    family = markers[0].family if markers else "tag36h11"
    det = Detector(families=family)
    doc = fitz.open(path)
    px_per_m = dpi / 0.0254

    seen: dict[int, list[tuple[float, float, float]]] = {}
    for b in range(field.n_lanes):
        page = doc[b]
        x = 0.0
        while x < field.length - 1e-9:
            x1 = min(x + tile_m, field.length)
            clip = fitz.Rect(x * M_TO_PT, 0, x1 * M_TO_PT,
                             field.lane_width * M_TO_PT)
            pix = page.get_pixmap(dpi=dpi, clip=clip)
            arr = np.frombuffer(pix.samples, np.uint8).reshape(
                pix.height, pix.width, pix.n)
            gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY) if pix.n >= 3 else arr
            for r in det.detect(gray):
                gx = x + r.center[0] / px_per_m
                gy = b * field.lane_width + (pix.height - r.center[1]) / px_per_m
                edge = np.mean([np.linalg.norm(r.corners[(k + 1) % 4] - r.corners[k])
                                for k in range(4)]) / px_per_m
                hits = seen.setdefault(r.tag_id, [])
                if not any(abs(gx - hx) < 0.05 and abs(gy - hy) < 0.05
                           for hx, hy, _ in hits):
                    hits.append((gx, gy, edge))
            x += tile_m - overlap_m

    expected = [m for m in markers if _lane_index(m.y, m.size, field) is not None]
    all_ok = True
    for m in expected:
        hits = seen.get(m.id, [])
        if len(hits) != 1:
            print(f"FEHLER: ID {m.id} {len(hits)}x erkannt (erwartet 1x).")
            all_ok = False
            continue
        gx, gy, edge = hits[0]
        ok = (abs(gx - m.x) <= tol_m and abs(gy - m.y) <= tol_m
              and abs(edge - m.size) <= tol_m)
        if not ok:
            print(f"ABWEICHUNG ID {m.id}: ({gx:.3f},{gy:.3f}) soll "
                  f"({m.x},{m.y}); size {edge:.3f} soll {m.size}")
            all_ok = False
    extra = set(seen) - {m.id for m in expected}
    if extra:
        print(f"FEHLER: unerwartete IDs erkannt: {sorted(extra)}")
        all_ok = False
    print(f"Erkannt: {len(seen)} IDs / {len(expected)} erwartet")
    return all_ok


if __name__ == "__main__":
    config_path = sys.argv[1] if len(sys.argv) > 1 else "marker_config.yaml"

    field, markers, output = load_config(config_path)

    for w in validate_markers(field, markers):
        print("WARNUNG:", w)

    pdf_path = output.get("path", "apriltag_lanes.pdf")
    warns = create_lane_pdf(
        pdf_path, field, markers,
        px_per_module=int(output.get("px_per_module", 100)),
        quiet_modules=float(output.get("quiet_modules", 1.0)),
        label=bool(output.get("label", True)),
        label_pt=LABEL_PT,
        label_gap=float(output.get("label_gap", 0.008)),
        registration_marks=bool(output.get("registration_marks", True)),
        alignment_ticks=bool(output.get("alignment_ticks", True)),
        tick_spacing=float(output.get("tick_spacing", 0.5)),
        tick_len=float(output.get("tick_len", 0.03)),
        tick_label=bool(output.get("tick_label", False)),
        tick_width=float(output.get("tick_width", 1.5)),
    )
    for w in warns:
        print("WARNUNG:", w)

    print(f"{len(markers)} Marker gelesen aus {config_path}")
    print(f"PDF geschrieben: {pdf_path}")