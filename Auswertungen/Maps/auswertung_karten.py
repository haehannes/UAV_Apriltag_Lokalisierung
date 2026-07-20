#!/usr/bin/env python3
"""Vergleich mehrerer AprilTag-/TagSLAM-Karten in x-y-Draufsicht.

Das Skript liest bis zu vier Karten im TagSLAM-YAML-Format ein und erzeugt:
  1. eine ueberlagerte Grafik (alle Karten in einem Plot, je Karte eine Farbe),
  2. eine Teilplot-Grafik (jede Karte einzeln, gemeinsame Achsengrenzen),
  3. eine Overlay-Grafik mit eingezeichneten Entfernungsringen um den Ursprung,
  4. Balkendiagramme der mittleren Abweichung je Markergroesse und je
     Entfernungsring,
  5. zwei Balkendiagramme mit einem Balken je Tag, aufsteigend sortiert nach
     Entfernung des Tags zum Ursprung, einmal fuer die Position und einmal
     fuer die Orientierung,
  5a. ein Balken- und ein Punktdiagramm der euklidischen Distanz d_xyz
     zwischen Karte und Referenz ueber dem Abstand des Tags zum Bezugstag
     aus ``DISTANCE_REFERENCE_TAG``,
  6. einen numerischen Vergleich der Tag-Positionen und -Orientierungen
     gleicher IDs gegen eine Referenzkarte als Konsolenausgabe, CSV und
     Textreport,
  7. einen Einzelvergleich ausgewaehlter Tags (Standard: Tag 1) ueber alle
     Karten hinweg gegen die Referenz,
  8. eine Auswertung gruppiert nach Markergroesse,
  9. eine Auswertung gestaffelt nach Entfernung des Tags zum Ursprung.

Verglichen werden ausschliesslich die Tags des ``tag_map``-Bodys.
Die Draufsichten zeigen nur x und y; z und Orientierung fliessen in den
numerischen Vergleich ein, damit die Plots uebersichtlich bleiben.

Kennzahlen der Position (in Millimetern):
  dx, dy, dz   vorzeichenbehaftete Differenz je Achse,
  d_xy         euklidischer Abstand in der x-y-Ebene, sqrt(dx^2 + dy^2),
  d_xyz        euklidischer Abstand im Raum, sqrt(dx^2 + dy^2 + dz^2).
Beide Distanzen werden je Tag gebildet und erst danach gemittelt.

Kennzahlen der Orientierung (in Grad):
  droll, dpitch, dyaw   extrinsische RPY-Zerlegung (Drehung um x, y, z,
                        ROS-Konvention) der Relativrotation
                        R_rel = R_ref^T * R_karte,
  dangle                Gesamtdrehwinkel von R_rel, unabhaengig von der
                        Zerlegung und damit das robusteste Betragsmass.
Die Rotationen der YAML-Dateien werden als Rodrigues-Vektoren (Achse-Winkel,
Betrag gleich Drehwinkel in Radiant) interpretiert.

Zusaetzlich werden die euklidischen Abstaende aller Tag-Paare untereinander
gegen die Referenz geprueft (Kennzahl ``d_pair`` in Millimetern, positiv
bedeutet zu grosser Abstand in der Karte). Dieses Mass ist unabhaengig von
einer Verschiebung oder Verdrehung der gesamten Karte und zeigt die innere
Massstabstreue; daraus wird auch ein Massstabsfaktor geschaetzt.

Statistik je Gruppe:
  bias         vorzeichenbehafteter Mittelwert, zeigt die Richtung eines
               systematischen Versatzes,
  mean         Mittelwert des Betrags,
  rms          quadratisches Mittel,
  max          groesster Betrag,
  max_signed   derselbe Extremwert mit Vorzeichen.
mean, rms und max werden aus den Betraegen gebildet und sind daher nie
negativ. Bias und max_signed gibt es nur bei vorzeichenbehafteten Kennzahlen,
also nicht bei d_xy, d_xyz und dangle.

Markergroesse und Entfernung zum Ursprung werden immer aus der Referenzkarte
(Ground Truth) uebernommen, damit die Gruppenzuordnung fuer alle Karten
identisch ist.

Konfiguration erfolgt ausschliesslich ueber den Block ``KONFIGURATION`` weiter
unten. Es sind keine Kommandozeilen-Argumente noetig.
"""

import csv
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")  # kein Fenster noetig, nur Dateiausgabe
import matplotlib.pyplot as plt


# ===========================================================================
# KONFIGURATION  --  hier alles einstellen
# ===========================================================================

# Bis zu vier Karten. Je Eintrag:
#   "path"  : Pfad zur YAML-Datei der Karte (Pflicht)
#   "color" : Farbe der Markierungen (jeder gueltige matplotlib-Farbwert)
#   "label" : Titel in der Legende
MAPS: List[dict] = [
    {"path": "Dein_Pfad/ground_truth.yaml", "color": "black",     "label": "Ground Truth"},
    {"path": "Dein_Pfad/Karte_1.yaml",     "color": "tab:blue",  "label": "Kartierung Versuch 1"},
    {"path": "Dein_Pfad/Karte_2.yaml",     "color": "tab:orange","label": "Kartierung Versuch 2"},
    {"path": "Dein_Pfad/Karte_3.yaml",     "color": "tab:green", "label": "Kartierung Versuch 3"},
]

# Referenzkarte fuer den numerischen Vergleich (Index in MAPS, 0 = erste Karte).
REFERENCE_INDEX: int = 0

# Legende in den Grafiken anzeigen?
SHOW_LEGEND: bool = True

# Tags, die zusaetzlich einzeln ueber alle Karten verglichen werden sollen.
SINGLE_TAG_IDS: List[int] = [1]

# Grenzen der Entfernungsringe um den Ursprung in Metern (aufsteigend).
# Aus dieser Liste entstehen die Intervalle [0, 1), [1, 1.5), ... [2.5, 3].
RING_EDGES: List[float] = [0.0, 1.0, 1.5, 2.0, 2.5, 3.0]

# Entfernung zum Ursprung in der x-y-Ebene (True) oder im Raum (False)?
RING_DISTANCE_2D: bool = True

# Ursprung, auf den sich die Entfernungen beziehen (x, y, z) in Metern.
RING_ORIGIN: Tuple[float, float, float] = (0.0, 0.0, 0.0)

# Klassengrenzen der Basislaenge fuer die paarweise Abstandspruefung in Metern.
# Eine Basislaenge ist der Abstand zweier Tags in der Referenzkarte.
PAIR_EDGES: List[float] = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

# Paarweise Abstaende in der x-y-Ebene (True) oder im Raum (False)?
PAIR_DISTANCE_2D: bool = False

# Kennzahl in den Balkendiagrammen je Groessenklasse und je Entfernungsring.
# Moeglich: "dx", "dy", "dz", "d_xy", "d_xyz"
BAR_METRIC: str = "d_xyz"

# Kennzahlen in den Balkendiagrammen je Tag (sortiert nach Entfernung).
# Position moeglich: "dx", "dy", "dz", "d_xy", "d_xyz"
# Orientierung moeglich: "droll", "dpitch", "dyaw", "dangle"
PER_TAG_POS_METRIC: str = "d_xyz"
PER_TAG_ROT_METRIC: str = "dangle"

# Hoechstzahl beschrifteter x-Achsen-Ticks in den Diagrammen je Tag.
# Es wird gleichmaessig ausgeduennt, damit die Achse lesbar bleibt.
PER_TAG_MAX_XTICKS: int = 18

# Bezugstag fuer die Diagramme "Abweichung ueber dem Abstand zu Tag N".
# Standard ist Tag 0, der im Ursprung liegt und die Karte fixiert.
DISTANCE_REFERENCE_TAG: int = 0

# Ausgleichsgerade im Punktdiagramm einzeichnen?
SCATTER_TRENDLINE: bool = True

# Ausgabe
OUTPUT_DIR: str = "."             # Zielordner fuer alle Ausgabedateien
OUTPUT_BASENAME: str = "karten_vergleich"  # Praefix aller Ausgabedateien
SAVE_PDF: bool = True             # Grafiken als PDF (vektorbasiert)
SAVE_PNG: bool = True             # Grafiken als PNG
PNG_DPI: int = 300

# Darstellung
MARKER: str = "x"                 # Markerform
MARKER_SIZE: float = 60.0         # Markergroesse (scatter-"s")
MARKER_LINEWIDTH: float = 1.2     # Strichstaerke der Marker
MARKER_ALPHA: float = 0.85        # Deckkraft (hilft bei Ueberlappung)
AXIS_PADDING: float = 0.15        # Rand um die Punkte in Metern
FIG_TITLE_OVERLAY: str = "Kartenvergleich (Draufsicht x-y)"
FIG_TITLE_RINGS: str = "Kartenvergleich mit Entfernungsringen (Draufsicht x-y)"

# Schriftgroessen aller Grafiken, zentral einstellbar (Angaben in Punkt).
# Diese Werte werden in matplotlib.rcParams gesetzt und gelten damit fuer
# alle erzeugten Grafiken. Fuer eine einheitliche Skalierung genuegt es, alle
# Werte gemeinsam zu vergroessern oder zu verkleinern; fuer eine feinere
# Abstimmung lassen sich die Eintraege einzeln aendern.
FONT_SIZES: Dict[str, float] = {
    "title": 20.0,            # Titel je Achse (ax.set_title)
    "suptitle": 15.0,         # uebergeordneter Titel der Teilplots (fig.suptitle)
    "axis_label": 16.0,       # Achsenbeschriftung, etwa "x [m]" oder "y [m]"
    "tick_label": 11.0,       # Zahlen an den Achsen
    "legend": 11.0,           # Legende
    "annotation": 8.0,        # Werte ueber den Balken und Ringbeschriftungen
    "tick_label_dense": 8.0,  # Tick-Beschriftung der gedraengten x-Achse
                              # (Diagramme je Tag, wenn nur jeder n-te Tick
                              # beschriftet und gedreht wird)
}

# ===========================================================================
# Ab hier keine Anpassung noetig
# ===========================================================================

# Reihenfolge der ausgewerteten Kennzahlen
POS_KEYS: List[str] = ["dx", "dy", "dz", "d_xy", "d_xyz"]
ROT_KEYS: List[str] = ["droll", "dpitch", "dyaw", "dangle"]
METRIC_KEYS: List[str] = POS_KEYS + ROT_KEYS

# Kennzahlen mit Vorzeichen; nur fuer sie sind Bias und max_signed sinnvoll.
# "d_pair" ist der Fehler eines paarweisen Tag-Tag-Abstands und traegt ein
# Vorzeichen, da die Karte einen Abstand sowohl zu gross als auch zu klein
# abbilden kann.
SIGNED_KEYS = {"dx", "dy", "dz", "droll", "dpitch", "dyaw", "d_pair"}

# Einheit je Kennzahl
UNIT_OF: Dict[str, str] = {key: "mm" for key in POS_KEYS}
UNIT_OF.update({key: "deg" for key in ROT_KEYS})
UNIT_OF["d_pair"] = "mm"


def apply_font_sizes() -> None:
    """Uebertraegt die Werte aus ``FONT_SIZES`` in die matplotlib-rcParams.

    Nach dem Aufruf verwenden alle folgenden Grafiken diese Schriftgroessen
    fuer Titel, Achsenbeschriftungen, Achsenzahlen und Legende. Die Funktion
    wird einmal zu Beginn von ``main`` aufgerufen. Einzelne Sonderstellen
    (Werte ueber Balken, Ringbeschriftungen, gedraengte Tick-Achsen) greifen
    zusaetzlich direkt auf ``FONT_SIZES`` zu.
    """
    plt.rcParams.update({
        "axes.titlesize": FONT_SIZES["title"],
        "figure.titlesize": FONT_SIZES["suptitle"],
        "axes.labelsize": FONT_SIZES["axis_label"],
        "xtick.labelsize": FONT_SIZES["tick_label"],
        "ytick.labelsize": FONT_SIZES["tick_label"],
        "legend.fontsize": FONT_SIZES["legend"],
    })


def rodrigues_to_matrix(rvec: np.ndarray) -> np.ndarray:
    """Wandelt einen Rodrigues-Vektor in eine Rotationsmatrix um.

    Der Betrag des Vektors ist der Drehwinkel in Radiant, die Richtung die
    Drehachse. Fuer sehr kleine Winkel wird die Einheitsmatrix zurueckgegeben,
    um eine Division durch null zu vermeiden.

    Args:
        rvec: Array der Form ``(3,)`` mit dem Rodrigues-Vektor.

    Returns:
        Rotationsmatrix der Form ``(3, 3)``.
    """
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return np.eye(3)
    axis = np.asarray(rvec, dtype=float) / theta
    skew = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return (np.eye(3) + np.sin(theta) * skew
            + (1.0 - np.cos(theta)) * (skew @ skew))


def matrix_to_rpy(matrix: np.ndarray) -> np.ndarray:
    """Zerlegt eine Rotationsmatrix in extrinsische RPY-Winkel.

    Die Zerlegung entspricht der ROS-Konvention (Drehung um x, dann y, dann z
    der festen Achsen, identisch zu ``tf2::Matrix3x3::getRPY``).

    Args:
        matrix: Rotationsmatrix der Form ``(3, 3)``.

    Returns:
        Array ``[roll, pitch, yaw]`` in Radiant.
    """
    sy = float(np.hypot(matrix[2, 1], matrix[2, 2]))
    if sy < 1e-9:
        # Gimbal-Lock: Roll auf null setzen, Rest in Yaw abbilden
        roll = 0.0
        pitch = float(np.arctan2(-matrix[2, 0], sy))
        yaw = float(np.arctan2(-matrix[0, 1], matrix[1, 1]))
    else:
        roll = float(np.arctan2(matrix[2, 1], matrix[2, 2]))
        pitch = float(np.arctan2(-matrix[2, 0], sy))
        yaw = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
    return np.array([roll, pitch, yaw])


def matrix_to_angle(matrix: np.ndarray) -> float:
    """Bestimmt den Gesamtdrehwinkel einer Rotationsmatrix.

    Args:
        matrix: Rotationsmatrix der Form ``(3, 3)``.

    Returns:
        Drehwinkel in Radiant im Bereich ``[0, pi]``.
    """
    cos_theta = (float(np.trace(matrix)) - 1.0) / 2.0
    return float(np.arccos(np.clip(cos_theta, -1.0, 1.0)))


def relative_rotation(ref_rvec: np.ndarray,
                      map_rvec: np.ndarray) -> Tuple[np.ndarray, float]:
    """Berechnet die Relativrotation zwischen Referenz- und Kartentag.

    Es wird ``R_rel = R_ref^T * R_karte`` gebildet und daraus die extrinsischen
    RPY-Winkel sowie der Gesamtdrehwinkel abgeleitet. Damit funktioniert die
    Auswertung auch dann korrekt, wenn die Referenz selbst rotiert ist.

    Args:
        ref_rvec: Rodrigues-Vektor des Tags in der Referenzkarte.
        map_rvec: Rodrigues-Vektor des Tags in der zu pruefenden Karte.

    Returns:
        Tuple aus ``[roll, pitch, yaw]`` in Grad und dem Gesamtdrehwinkel
        in Grad.
    """
    r_ref = rodrigues_to_matrix(ref_rvec)
    r_map = rodrigues_to_matrix(map_rvec)
    r_rel = r_ref.T @ r_map
    rpy = np.degrees(matrix_to_rpy(r_rel))
    angle = float(np.degrees(matrix_to_angle(r_rel)))
    return rpy, angle


def load_map(path: str) -> Tuple[Dict[int, np.ndarray],
                                 Dict[int, np.ndarray],
                                 Dict[int, float]]:
    """Liest eine TagSLAM-YAML-Karte ein.

    Es werden nur Bodys vom Typ ``tag_map`` beruecksichtigt; der ``camera``-Body
    wird ignoriert. Eine eventuelle Translation der Body-Pose wird auf die
    Tag-Positionen addiert (eine Body-Rotation wird nicht angewandt, da die
    Karten in diesem Projekt achsenparallel im selben Weltframe vorliegen).
    Fehlt bei einem Tag das Feld ``size``, wird ``default_tag_size`` des Bodys
    verwendet.

    Args:
        path: Pfad zur YAML-Datei.

    Returns:
        Tuple aus ``{tag_id: array([x, y, z])}`` mit den Positionen in Metern,
        ``{tag_id: array([rx, ry, rz])}`` mit den Rodrigues-Vektoren in Radiant
        und ``{tag_id: size}`` mit der Kantenlaenge der Tags in Metern.
    """
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if not isinstance(data, dict) or "bodies" not in data:
        raise ValueError(f"'{path}' enthaelt keinen 'bodies'-Block.")

    positions: Dict[int, np.ndarray] = {}
    rotations: Dict[int, np.ndarray] = {}
    sizes: Dict[int, float] = {}
    for body in data.get("bodies", []):
        if not isinstance(body, dict):
            continue
        for body_name, body_content in body.items():
            if body_name != "tag_map" or not isinstance(body_content, dict):
                continue

            default_size = body_content.get("default_tag_size")

            # Optionaler Translationsoffset der Body-Pose
            body_pose = body_content.get("pose", {}) or {}
            body_pos = body_pose.get("position", {}) or {}
            offset = np.array([
                float(body_pos.get("x", 0.0)),
                float(body_pos.get("y", 0.0)),
                float(body_pos.get("z", 0.0)),
            ])

            for tag in body_content.get("tags", []) or []:
                tag_id = int(tag["id"])
                pose = tag["pose"]
                pos = pose["position"]
                xyz = np.array([
                    float(pos["x"]),
                    float(pos["y"]),
                    float(pos["z"]),
                ]) + offset
                rot = pose.get("rotation", {}) or {}
                rvec = np.array([
                    float(rot.get("x", 0.0)),
                    float(rot.get("y", 0.0)),
                    float(rot.get("z", 0.0)),
                ])
                if tag_id in positions:
                    print(f"  Warnung: Tag-ID {tag_id} in '{path}' mehrfach "
                          f"vorhanden, letzter Eintrag wird verwendet.")
                positions[tag_id] = xyz
                rotations[tag_id] = rvec
                size = tag.get("size", default_size)
                if size is not None:
                    sizes[tag_id] = float(size)

    if not positions:
        raise ValueError(f"'{path}' enthaelt keine Tags in einem 'tag_map'-Body.")
    return positions, rotations, sizes


def tag_distances(positions: Dict[int, np.ndarray]) -> Dict[int, float]:
    """Berechnet den Abstand jedes Tags zum konfigurierten Ursprung.

    Je nach ``RING_DISTANCE_2D`` wird der Abstand in der x-y-Ebene oder im
    Raum bestimmt.

    Args:
        positions: Positionen der Tags, ueblicherweise die der Referenzkarte.

    Returns:
        Dictionary ``{tag_id: abstand_in_metern}``.
    """
    origin = np.asarray(RING_ORIGIN, dtype=float)
    distances: Dict[int, float] = {}
    for tag_id, xyz in positions.items():
        delta = xyz - origin
        if RING_DISTANCE_2D:
            distances[tag_id] = float(np.linalg.norm(delta[:2]))
        else:
            distances[tag_id] = float(np.linalg.norm(delta))
    return distances


def ring_label(lower: float, upper: float) -> str:
    """Erzeugt die Beschriftung eines Entfernungsrings.

    Args:
        lower: Untere Ringgrenze in Metern.
        upper: Obere Ringgrenze in Metern.

    Returns:
        Beschriftung der Form ``"1-1.5 m"``.
    """
    return f"{lower:g}-{upper:g} m"


def assign_ring(distance: float) -> Optional[int]:
    """Ordnet einen Abstand einem Entfernungsring zu.

    Die Ringe sind halboffen ``[lower, upper)``; der letzte Ring schliesst die
    obere Grenze mit ein. Abstaende ausserhalb aller Ringe ergeben ``None``.

    Args:
        distance: Abstand zum Ursprung in Metern.

    Returns:
        Index des Rings in ``RING_EDGES`` oder ``None``.
    """
    for idx in range(len(RING_EDGES) - 1):
        lower = RING_EDGES[idx]
        upper = RING_EDGES[idx + 1]
        is_last = idx == len(RING_EDGES) - 2
        if lower <= distance < upper or (is_last and distance == upper):
            return idx
    return None


def tag_metrics(ref_pos: np.ndarray, ref_rot: np.ndarray,
                map_pos: np.ndarray, map_rot: np.ndarray) -> Dict[str, float]:
    """Berechnet alle Kennzahlen fuer ein einzelnes Tag.

    Die Positionsdifferenzen werden in Millimetern, die Winkel in Grad
    zurueckgegeben.

    Args:
        ref_pos: Position des Tags in der Referenzkarte in Metern.
        ref_rot: Rodrigues-Vektor des Tags in der Referenzkarte.
        map_pos: Position des Tags in der zu pruefenden Karte in Metern.
        map_rot: Rodrigues-Vektor des Tags in der zu pruefenden Karte.

    Returns:
        Dictionary mit den Schluesseln aus ``METRIC_KEYS``.
    """
    diff = np.asarray(map_pos, dtype=float) - np.asarray(ref_pos, dtype=float)
    rpy, angle = relative_rotation(ref_rot, map_rot)
    return {
        "dx": float(diff[0]) * 1000.0,
        "dy": float(diff[1]) * 1000.0,
        "dz": float(diff[2]) * 1000.0,
        "d_xy": float(np.linalg.norm(diff[:2])) * 1000.0,
        "d_xyz": float(np.linalg.norm(diff)) * 1000.0,
        "droll": float(rpy[0]),
        "dpitch": float(rpy[1]),
        "dyaw": float(rpy[2]),
        "dangle": angle,
    }


def collect_metrics(ref_pos: Dict[int, np.ndarray],
                    ref_rot: Dict[int, np.ndarray],
                    map_pos: Dict[int, np.ndarray],
                    map_rot: Dict[int, np.ndarray],
                    tag_ids: Sequence[int]) -> Dict[str, np.ndarray]:
    """Bildet die Kennzahlen fuer eine Menge von Tags als Arrays.

    Args:
        ref_pos: Positionen der Referenzkarte.
        ref_rot: Rotationen der Referenzkarte.
        map_pos: Positionen der zu pruefenden Karte.
        map_rot: Rotationen der zu pruefenden Karte.
        tag_ids: Auszuwertende Tag-IDs, muessen in beiden Karten vorhanden sein.

    Returns:
        Dictionary ``{kennzahl: array}`` mit je einem Wert pro Tag.
    """
    rows = [tag_metrics(ref_pos[t], ref_rot[t], map_pos[t], map_rot[t])
            for t in tag_ids]
    return {key: np.array([row[key] for row in rows]) for key in METRIC_KEYS}


def stat_full(values: np.ndarray) -> Tuple[float, float, float, float, float]:
    """Berechnet Bias, Betragsmittel, RMS, Maximalbetrag und Vorzeichenmaximum.

    Der Bias ist der vorzeichenbehaftete Mittelwert und zeigt damit die
    Richtung eines systematischen Versatzes. Der Betragsmittelwert bleibt
    daneben erhalten, weil sich gegenlaeufige Abweichungen im Bias sonst
    gegenseitig aufheben wuerden. ``mean``, ``rms`` und ``max`` sind
    konstruktionsbedingt nie negativ, da sie aus den Betraegen gebildet
    werden; ``max_signed`` gibt denselben Extremwert mit seinem Vorzeichen
    zurueck und zeigt damit die Richtung des groessten Ausreissers.

    Args:
        values: Array der Abweichungen in der Einheit der Kennzahl.

    Returns:
        Tuple ``(bias, mean, rms, max, max_signed)``.
    """
    absolute = np.abs(values)
    bias = float(values.mean())
    mean = float(absolute.mean())
    rms = float(np.sqrt(np.mean(values ** 2)))
    index = int(np.argmax(absolute))
    maximum = float(absolute[index])
    max_signed = float(values[index])
    return bias, mean, rms, maximum, max_signed


def format_stats(key: str, values: np.ndarray, unit: str = "") -> str:
    """Formatiert die Statistik einer Kennzahl als Textzeile.

    Args:
        key: Name der Kennzahl, ueblicherweise aus ``METRIC_KEYS``.
        values: Array der Abweichungen.
        unit: Einheit; leer bedeutet Nachschlagen in ``UNIT_OF``.

    Returns:
        Formatierter String inklusive Einheit; Bias und Vorzeichenmaximum
        entfallen bei vorzeichenfreien Kennzahlen.
    """
    bias, mean, rms, maximum, max_signed = stat_full(values)
    signed = key in SIGNED_KEYS
    bias_text = f"{bias:+9.2f}" if signed else f"{'-':>9s}"
    max_signed_text = f"{max_signed:+9.2f}" if signed else f"{'-':>9s}"
    unit_text = unit or UNIT_OF[key]
    return (f"{key:7s} bias={bias_text}   mean={mean:8.2f}   "
            f"rms={rms:8.2f}   max={maximum:8.2f}   "
            f"max_signed={max_signed_text}   [{unit_text}]")


def compute_axis_limits(maps_xyz: List[Dict[int, np.ndarray]],
                        padding: float) -> Tuple[float, float, float, float]:
    """Bestimmt gemeinsame x-y-Achsengrenzen ueber alle Karten.

    Args:
        maps_xyz: Liste der eingelesenen Karten (Position-Dictionaries).
        padding: Zusaetzlicher Rand in Metern.

    Returns:
        Tuple ``(xmin, xmax, ymin, ymax)`` inklusive Rand.
    """
    all_points = np.vstack([np.stack(list(m.values())) for m in maps_xyz if m])
    xmin, ymin = all_points[:, 0].min(), all_points[:, 1].min()
    xmax, ymax = all_points[:, 0].max(), all_points[:, 1].max()
    return (xmin - padding, xmax + padding, ymin - padding, ymax + padding)


def _save_figure(fig: plt.Figure, basename: str) -> List[str]:
    """Speichert eine Figure je nach Konfiguration als PDF und/oder PNG.

    Args:
        fig: Die zu speichernde Matplotlib-Figure.
        basename: Dateiname ohne Endung (relativ zu OUTPUT_DIR).

    Returns:
        Liste der geschriebenen Dateipfade.
    """
    written: List[str] = []
    stem = os.path.join(OUTPUT_DIR, basename)
    if SAVE_PDF:
        path = stem + ".pdf"
        fig.savefig(path, bbox_inches="tight")
        written.append(path)
    if SAVE_PNG:
        path = stem + ".png"
        fig.savefig(path, dpi=PNG_DPI, bbox_inches="tight")
        written.append(path)
    return written


def plot_overlay(maps_xyz: List[Dict[int, np.ndarray]],
                 configs: List[dict],
                 limits: Tuple[float, float, float, float]) -> List[str]:
    """Erzeugt die ueberlagerte Grafik aller Karten in einem Plot.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        configs: Zugehoerige Konfigurationseintraege (Farbe, Label).
        limits: Gemeinsame Achsengrenzen ``(xmin, xmax, ymin, ymax)``.

    Returns:
        Liste der geschriebenen Dateipfade.
    """
    fig, ax = plt.subplots(figsize=(8, 8))
    for positions, cfg in zip(maps_xyz, configs):
        pts = np.stack(list(positions.values()))
        ax.scatter(pts[:, 0], pts[:, 1], s=MARKER_SIZE, marker=MARKER,
                   linewidths=MARKER_LINEWIDTH, alpha=MARKER_ALPHA,
                   color=cfg["color"], label=cfg["label"])

    ax.set_xlim(limits[0], limits[1])
    ax.set_ylim(limits[2], limits[3])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(FIG_TITLE_OVERLAY)
    ax.grid(True, linestyle=":", alpha=0.5)
    if SHOW_LEGEND:
        ax.legend(loc="best", framealpha=0.9)
    fig.tight_layout()
    written = _save_figure(fig, OUTPUT_BASENAME + "_ueberlagert")
    plt.close(fig)
    return written


def plot_subplots(maps_xyz: List[Dict[int, np.ndarray]],
                  configs: List[dict],
                  limits: Tuple[float, float, float, float]) -> List[str]:
    """Erzeugt eine Teilplot-Grafik mit je einer Karte pro Achse.

    Alle Teilplots nutzen dieselben Achsengrenzen, damit die Karten direkt
    vergleichbar sind.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        configs: Zugehoerige Konfigurationseintraege (Farbe, Label).
        limits: Gemeinsame Achsengrenzen ``(xmin, xmax, ymin, ymax)``.

    Returns:
        Liste der geschriebenen Dateipfade.
    """
    count = len(maps_xyz)
    ncols = 1 if count == 1 else 2
    nrows = int(np.ceil(count / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 6 * nrows),
                             squeeze=False)
    flat_axes = axes.flatten()

    for idx, (positions, cfg) in enumerate(zip(maps_xyz, configs)):
        ax = flat_axes[idx]
        pts = np.stack(list(positions.values()))
        ax.scatter(pts[:, 0], pts[:, 1], s=MARKER_SIZE, marker=MARKER,
                   linewidths=MARKER_LINEWIDTH, alpha=MARKER_ALPHA,
                   color=cfg["color"])
        ax.set_xlim(limits[0], limits[1])
        ax.set_ylim(limits[2], limits[3])
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(f"{cfg['label']}  ({len(positions)} Tags)")
        ax.grid(True, linestyle=":", alpha=0.5)

    # Ueberzaehlige leere Achsen ausblenden
    for idx in range(count, len(flat_axes)):
        flat_axes[idx].axis("off")

    fig.suptitle("Kartenvergleich je Karte (Draufsicht x-y)")
    fig.tight_layout()
    written = _save_figure(fig, OUTPUT_BASENAME + "_teilplots")
    plt.close(fig)
    return written


def plot_overlay_with_rings(maps_xyz: List[Dict[int, np.ndarray]],
                            configs: List[dict],
                            limits: Tuple[float, float, float, float]
                            ) -> List[str]:
    """Erzeugt eine zusaetzliche Overlay-Grafik mit Entfernungsringen.

    Die Ringe entsprechen den Grenzen aus ``RING_EDGES`` und werden um
    ``RING_ORIGIN`` gezeichnet. Der bestehende Overlay-Plot bleibt unveraendert
    erhalten, diese Grafik ist eine zusaetzliche Ausgabe.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        configs: Zugehoerige Konfigurationseintraege (Farbe, Label).
        limits: Gemeinsame Achsengrenzen ``(xmin, xmax, ymin, ymax)``.

    Returns:
        Liste der geschriebenen Dateipfade.
    """
    fig, ax = plt.subplots(figsize=(8, 8))

    for radius in RING_EDGES:
        if radius <= 0.0:
            continue
        circle = plt.Circle((RING_ORIGIN[0], RING_ORIGIN[1]), radius,
                            fill=False, color="gray", linestyle="--",
                            linewidth=0.9, alpha=0.8, zorder=1)
        ax.add_patch(circle)
        ax.annotate(f"{radius:g} m",
                    xy=(RING_ORIGIN[0], RING_ORIGIN[1] + radius),
                    xytext=(2, 2), textcoords="offset points",
                    fontsize=FONT_SIZES["annotation"], color="gray", zorder=2)

    ax.plot(RING_ORIGIN[0], RING_ORIGIN[1], marker="+", color="gray",
            markersize=10, zorder=2)

    for positions, cfg in zip(maps_xyz, configs):
        pts = np.stack(list(positions.values()))
        ax.scatter(pts[:, 0], pts[:, 1], s=MARKER_SIZE, marker=MARKER,
                   linewidths=MARKER_LINEWIDTH, alpha=MARKER_ALPHA,
                   color=cfg["color"], label=cfg["label"], zorder=3)

    ax.set_xlim(limits[0], limits[1])
    ax.set_ylim(limits[2], limits[3])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(FIG_TITLE_RINGS)
    ax.grid(True, linestyle=":", alpha=0.5)
    if SHOW_LEGEND:
        ax.legend(loc="best", framealpha=0.9)
    fig.tight_layout()
    written = _save_figure(fig, OUTPUT_BASENAME + "_ringe")
    plt.close(fig)
    return written


def _plot_grouped_bars(group_labels: List[str],
                       series: List[Tuple[str, str, List[Optional[float]]]],
                       title: str,
                       xlabel: str,
                       ylabel: str,
                       basename: str,
                       annotate: bool = True,
                       xtick_step: int = 1,
                       figsize: Optional[Tuple[float, float]] = None
                       ) -> List[str]:
    """Zeichnet ein gruppiertes Balkendiagramm.

    Args:
        group_labels: Beschriftung der Gruppen auf der x-Achse.
        series: Liste aus ``(label, farbe, werte)`` je Karte; ``None`` in den
            Werten bedeutet, dass die Gruppe fuer diese Karte leer ist.
        title: Titel der Grafik.
        xlabel: Beschriftung der x-Achse.
        ylabel: Beschriftung der y-Achse.
        basename: Dateiname ohne Endung (relativ zu OUTPUT_DIR).
        annotate: Zahlenwerte ueber die Balken schreiben?
        xtick_step: Nur jeder n-te Tick wird beschriftet.
        figsize: Optionale Groesse der Figure in Zoll.

    Returns:
        Liste der geschriebenen Dateipfade.
    """
    if not group_labels or not series:
        return []

    x = np.arange(len(group_labels), dtype=float)
    width = 0.8 / len(series)

    if figsize is None:
        figsize = (max(7.0, 1.8 * len(group_labels)), 5.0)

    fig, ax = plt.subplots(figsize=figsize)
    for idx, (label, color, values) in enumerate(series):
        offset = (idx - (len(series) - 1) / 2.0) * width
        heights = [0.0 if v is None else v for v in values]
        bars = ax.bar(x + offset, heights, width=width, label=label,
                      color=color, alpha=0.9)
        if not annotate:
            continue
        for bar, value in zip(bars, values):
            if value is None:
                continue
            ax.annotate(f"{value:.1f}",
                        xy=(bar.get_x() + bar.get_width() / 2.0, value),
                        xytext=(0, 2), textcoords="offset points",
                        ha="center", va="bottom",
                        fontsize=FONT_SIZES["annotation"])

    tick_positions = x[::xtick_step]
    tick_labels = group_labels[::xtick_step]
    ax.set_xticks(tick_positions)
    ax.set_xticklabels(tick_labels, rotation=90 if xtick_step > 1 else 0,
                       fontsize=(FONT_SIZES["tick_label_dense"] if xtick_step > 1
                                 else FONT_SIZES["tick_label"]))
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, axis="y", linestyle=":", alpha=0.5)
    if SHOW_LEGEND:
        ax.legend(loc="best", framealpha=0.9)
    fig.tight_layout()
    written = _save_figure(fig, basename)
    plt.close(fig)
    return written


def plot_per_tag_distance(maps_xyz: List[Dict[int, np.ndarray]],
                          maps_rot: List[Dict[int, np.ndarray]],
                          configs: List[dict],
                          reference_index: int) -> List[str]:
    """Erzeugt Balkendiagramme mit einem Balken je Tag ueber der Entfernung.

    Die Tags werden aufsteigend nach ihrer Entfernung zum Ursprung sortiert
    (Referenzposition). Die x-Achse ist kategorial, beschriftet wird sie mit
    der Entfernung in Metern; um die Achse lesbar zu halten, wird nur jeder
    n-te Tick beschriftet. Es entstehen zwei Grafiken, eine fuer die Position
    und eine fuer die Orientierung.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        maps_rot: Liste der Rotationen je Karte.
        configs: Zugehoerige Konfigurationseintraege (Farbe, Label).
        reference_index: Index der Referenzkarte in den Listen.

    Returns:
        Liste der geschriebenen Dateipfade.
    """
    ref_pos = maps_xyz[reference_index]
    ref_rot = maps_rot[reference_index]
    distances = tag_distances(ref_pos)

    # Alle Tags, die in der Referenz und in mindestens einer weiteren Karte sind
    relevant = set()
    for idx, positions in enumerate(maps_xyz):
        if idx == reference_index:
            continue
        relevant |= set(ref_pos.keys()) & set(positions.keys())

    if not relevant:
        return []

    ordered = sorted(relevant, key=lambda t: (distances[t], t))
    labels = [f"{distances[t]:.2f}" for t in ordered]
    step = max(1, int(np.ceil(len(ordered) / max(1, PER_TAG_MAX_XTICKS))))

    series_pos: List[Tuple[str, str, List[Optional[float]]]] = []
    series_rot: List[Tuple[str, str, List[Optional[float]]]] = []

    for idx, (positions, cfg) in enumerate(zip(maps_xyz, configs)):
        if idx == reference_index:
            continue
        rotations = maps_rot[idx]
        values_pos: List[Optional[float]] = []
        values_rot: List[Optional[float]] = []
        for tag_id in ordered:
            if tag_id not in positions:
                values_pos.append(None)
                values_rot.append(None)
                continue
            metrics = tag_metrics(ref_pos[tag_id], ref_rot[tag_id],
                                  positions[tag_id], rotations[tag_id])
            values_pos.append(abs(metrics[PER_TAG_POS_METRIC]))
            values_rot.append(abs(metrics[PER_TAG_ROT_METRIC]))
        series_pos.append((cfg["label"], cfg["color"], values_pos))
        series_rot.append((cfg["label"], cfg["color"], values_rot))

    # Breite an die Zahl der Balken koppeln, aber nach oben begrenzen, damit
    # die Grafik auch bei vier Karten noch handhabbar bleibt.
    width_inch = float(np.clip(0.16 * len(ordered), 9.0, 22.0))
    figsize = (width_inch, 5.5)

    written = _plot_grouped_bars(
        group_labels=labels,
        series=series_pos,
        title="Abweichung der Position je Tag ueber der Entfernung zum Ursprung",
        xlabel="Entfernung des Markers zum Ursprung [m]",
        ylabel=f"Abweichung {PER_TAG_POS_METRIC} [mm]",
        basename=OUTPUT_BASENAME + "_je_tag_position",
        annotate=False,
        xtick_step=step,
        figsize=figsize,
    )
    written += _plot_grouped_bars(
        group_labels=labels,
        series=series_rot,
        title=("Abweichung der Orientierung je Tag ueber der Entfernung "
               "zum Ursprung"),
        xlabel="Entfernung des Markers zum Ursprung [m]",
        ylabel=f"Abweichung {PER_TAG_ROT_METRIC} [deg]",
        basename=OUTPUT_BASENAME + "_je_tag_orientierung",
        annotate=False,
        xtick_step=step,
        figsize=figsize,
    )
    return written


def distances_to_tag(positions: Dict[int, np.ndarray],
                     tag_id: int) -> Dict[int, float]:
    """Berechnet den raeumlichen Abstand aller Tags zu einem Bezugstag.

    Der Abstand wird immer dreidimensional gebildet, damit er zur ebenfalls
    dreidimensionalen Kennzahl d_xyz passt.

    Args:
        positions: Positionen der Tags, ueblicherweise die der Referenzkarte.
        tag_id: ID des Bezugstags.

    Returns:
        Dictionary ``{tag_id: abstand_in_metern}``; leer, wenn der Bezugstag
        nicht vorhanden ist.
    """
    if tag_id not in positions:
        return {}
    anchor = positions[tag_id]
    return {other: float(np.linalg.norm(xyz - anchor))
            for other, xyz in positions.items()}


def _plot_scatter(x_values: np.ndarray,
                  series: List[Tuple[str, str, np.ndarray, np.ndarray]],
                  title: str,
                  xlabel: str,
                  ylabel: str,
                  basename: str) -> List[str]:
    """Zeichnet ein Punktdiagramm mit optionaler Ausgleichsgerade.

    Args:
        x_values: Wird nur fuer die Achsengrenzen ausgewertet.
        series: Liste aus ``(label, farbe, x_array, y_array)`` je Karte.
        title: Titel der Grafik.
        xlabel: Beschriftung der x-Achse.
        ylabel: Beschriftung der y-Achse.
        basename: Dateiname ohne Endung (relativ zu OUTPUT_DIR).

    Returns:
        Liste der geschriebenen Dateipfade.
    """
    if not series:
        return []

    fig, ax = plt.subplots(figsize=(9, 5.5))
    for label, color, x_data, y_data in series:
        ax.scatter(x_data, y_data, s=28, marker="o", alpha=0.7,
                   edgecolors="none", color=color, label=label)

        if not SCATTER_TRENDLINE or len(x_data) < 2:
            continue
        # Ausgleichsgerade durch den Ursprung: bei einem reinen
        # Massstabsfehler ist die Abweichung proportional zum Abstand.
        denominator = float(np.sum(x_data ** 2))
        if denominator < 1e-12:
            continue
        slope = float(np.sum(x_data * y_data) / denominator)
        x_line = np.array([0.0, float(x_data.max())])
        ax.plot(x_line, slope * x_line, linestyle="--", linewidth=1.2,
                color=color, alpha=0.9,
                label=f"{label}: Ausgleichsgerade {slope:.2f} mm/m")

    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, linestyle=":", alpha=0.5)
    if SHOW_LEGEND:
        ax.legend(loc="best", framealpha=0.9)
    fig.tight_layout()
    written = _save_figure(fig, basename)
    plt.close(fig)
    return written


def plot_distance_to_tag(maps_xyz: List[Dict[int, np.ndarray]],
                         maps_rot: List[Dict[int, np.ndarray]],
                         configs: List[dict],
                         reference_index: int) -> List[str]:
    """Stellt die Abweichung d_xyz ueber dem Abstand zum Bezugstag dar.

    Auf der y-Achse steht die euklidische Distanz zwischen der Tag-Position in
    der Karte und der in der Referenz, also d_xyz. Auf der x-Achse steht der
    Abstand des Tags zum Bezugstag aus ``DISTANCE_REFERENCE_TAG``.

    Es entstehen zwei Grafiken mit denselben Daten:
      * ein Balkendiagramm mit einem Balken je Tag, aufsteigend sortiert,
      * ein Punktdiagramm mit stetiger x-Achse und Ausgleichsgerade durch den
        Ursprung; deren Steigung ist der radiale Fehlerzuwachs in mm je Meter.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        maps_rot: Liste der Rotationen je Karte.
        configs: Zugehoerige Konfigurationseintraege (Farbe, Label).
        reference_index: Index der Referenzkarte in den Listen.

    Returns:
        Liste der geschriebenen Dateipfade.
    """
    ref_pos = maps_xyz[reference_index]
    ref_rot = maps_rot[reference_index]
    anchor = DISTANCE_REFERENCE_TAG

    distances = distances_to_tag(ref_pos, anchor)
    if not distances:
        print(f"Hinweis: Bezugstag {anchor} ist in der Referenz nicht "
              f"vorhanden, die Abstandsdiagramme entfallen.")
        return []

    relevant = set()
    for idx, positions in enumerate(maps_xyz):
        if idx == reference_index:
            continue
        relevant |= set(ref_pos.keys()) & set(positions.keys())
    if not relevant:
        return []

    ordered = sorted(relevant, key=lambda t: (distances[t], t))
    labels = [f"{distances[t]:.2f}" for t in ordered]
    step = max(1, int(np.ceil(len(ordered) / max(1, PER_TAG_MAX_XTICKS))))

    bar_series: List[Tuple[str, str, List[Optional[float]]]] = []
    scatter_series: List[Tuple[str, str, np.ndarray, np.ndarray]] = []

    for idx, (positions, cfg) in enumerate(zip(maps_xyz, configs)):
        if idx == reference_index:
            continue
        rotations = maps_rot[idx]

        values: List[Optional[float]] = []
        x_data: List[float] = []
        y_data: List[float] = []
        for tag_id in ordered:
            if tag_id not in positions:
                values.append(None)
                continue
            metrics = tag_metrics(ref_pos[tag_id], ref_rot[tag_id],
                                  positions[tag_id], rotations[tag_id])
            values.append(metrics["d_xyz"])
            x_data.append(distances[tag_id])
            y_data.append(metrics["d_xyz"])

        bar_series.append((cfg["label"], cfg["color"], values))
        scatter_series.append((cfg["label"], cfg["color"],
                               np.array(x_data), np.array(y_data)))

    xlabel = f"Abstand des Markers zu Tag {anchor} [m]"
    ylabel = "euklidische Distanz Karte zu Ground Truth, d_xyz [mm]"
    width_inch = float(np.clip(0.16 * len(ordered), 9.0, 22.0))

    written = _plot_grouped_bars(
        group_labels=labels,
        series=bar_series,
        title=f"Euklidische Distanz zur Ground Truth Über dem Abstand "
              f"zum Ursprung",
        xlabel=xlabel,
        ylabel=ylabel,
        basename=OUTPUT_BASENAME + f"_dxyz_ueber_tag{anchor}_balken",
        annotate=False,
        xtick_step=step,
        figsize=(width_inch, 5.5),
    )
    written += _plot_scatter(
        x_values=np.array([distances[t] for t in ordered]),
        series=scatter_series,
        title=f"Euklidische Distanz zur Ground Truth Über dem Abstand "
              f"zum Ursprung",
        xlabel=xlabel,
        ylabel=ylabel,
        basename=OUTPUT_BASENAME + f"_dxyz_ueber_tag{anchor}_punkte",
    )
    return written


def compare_numeric(maps_xyz: List[Dict[int, np.ndarray]],
                    maps_rot: List[Dict[int, np.ndarray]],
                    configs: List[dict],
                    reference_index: int) -> Tuple[str, str]:
    """Vergleicht Position und Orientierung jeder Karte mit der Referenzkarte.

    Fuer jede Tag-ID, die in beiden Karten vorkommt, werden alle Kennzahlen aus
    ``METRIC_KEYS`` berechnet. Ausgegeben werden Textzeilen fuer den Report und
    eine ausfuehrliche CSV je Tag.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        maps_rot: Liste der Rotationen je Karte.
        configs: Zugehoerige Konfigurationseintraege (Label).
        reference_index: Index der Referenzkarte in den Listen.

    Returns:
        Tuple ``(berichtstext, csv_pfad)``.
    """
    ref_pos = maps_xyz[reference_index]
    ref_rot = maps_rot[reference_index]
    ref_label = configs[reference_index]["label"]
    csv_path = os.path.join(OUTPUT_DIR, OUTPUT_BASENAME + "_abweichungen.csv")
    distances = tag_distances(ref_pos)

    csv_rows: List[list] = []
    lines: List[str] = []

    header = f"Numerischer Vergleich gegen Referenz: '{ref_label}'"
    lines.append(header)
    lines.append("=" * len(header))

    for idx, (positions, cfg) in enumerate(zip(maps_xyz, configs)):
        if idx == reference_index:
            continue
        rotations = maps_rot[idx]

        common = sorted(set(ref_pos.keys()) & set(positions.keys()))
        only_ref = sorted(set(ref_pos.keys()) - set(positions.keys()))
        only_map = sorted(set(positions.keys()) - set(ref_pos.keys()))

        lines.append(f"\n[{cfg['label']}]  vs.  [{ref_label}]")

        if not common:
            lines.append("  Keine gemeinsamen Tag-IDs vorhanden.")
            continue

        values = collect_metrics(ref_pos, ref_rot, positions, rotations, common)

        for row_idx, tag_id in enumerate(common):
            row = [cfg["label"], tag_id, f"{distances[tag_id]:.4f}"]
            row += [f"{values[key][row_idx]:.4f}" for key in METRIC_KEYS]
            csv_rows.append(row)

        lines.append(f"  Gemeinsame Tags: {len(common)}   "
                     f"nur in Referenz: {len(only_ref)}   "
                     f"nur in dieser Karte: {len(only_map)}")
        for key in METRIC_KEYS:
            lines.append("  " + format_stats(key, values[key]))
        if only_ref:
            lines.append(f"  IDs nur in Referenz: {only_ref}")
        if only_map:
            lines.append(f"  IDs nur in dieser Karte: {only_map}")

    header_row = ["karte", "tag_id", "entfernung_m"]
    header_row += [f"{key}_{UNIT_OF[key]}" for key in METRIC_KEYS]
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header_row)
        writer.writerows(csv_rows)

    return "\n".join(lines), csv_path


def compare_single_tags(maps_xyz: List[Dict[int, np.ndarray]],
                        maps_rot: List[Dict[int, np.ndarray]],
                        maps_sizes: List[Dict[int, float]],
                        configs: List[dict],
                        reference_index: int,
                        tag_ids: Sequence[int]) -> Tuple[str, str]:
    """Vergleicht einzelne, explizit gewaehlte Tags ueber alle Karten hinweg.

    Fuer jede angegebene Tag-ID werden Position und Orientierung in der
    Referenzkarte sowie die Abweichungen in jeder weiteren Karte ausgegeben.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        maps_rot: Liste der Rotationen je Karte.
        maps_sizes: Liste der Tag-Groessen je Karte.
        configs: Zugehoerige Konfigurationseintraege (Label).
        reference_index: Index der Referenzkarte in den Listen.
        tag_ids: Zu vergleichende Tag-IDs.

    Returns:
        Tuple ``(berichtstext, csv_pfad)``.
    """
    ref_pos = maps_xyz[reference_index]
    ref_rot = maps_rot[reference_index]
    ref_sizes = maps_sizes[reference_index]
    ref_label = configs[reference_index]["label"]
    csv_path = os.path.join(OUTPUT_DIR, OUTPUT_BASENAME + "_einzeltags.csv")
    distances = tag_distances(ref_pos)

    csv_rows: List[list] = []
    lines: List[str] = []

    header = f"Einzelvergleich ausgewaehlter Tags gegen '{ref_label}'"
    lines.append(header)
    lines.append("=" * len(header))

    for tag_id in tag_ids:
        size = ref_sizes.get(tag_id)
        size_text = f"{size:.3f} m" if size is not None else "unbekannt"

        if tag_id not in ref_pos:
            lines.append(f"\n[Tag {tag_id}]")
            lines.append(f"  Tag {tag_id} in der Referenz nicht vorhanden.")
            continue

        lines.append(f"\n[Tag {tag_id}]  (Kantenlaenge {size_text}, "
                     f"Entfernung zum Ursprung {distances[tag_id]:.3f} m)")
        ref_xyz = ref_pos[tag_id]
        lines.append(f"  Referenz  x={ref_xyz[0]:9.5f} m  "
                     f"y={ref_xyz[1]:9.5f} m  z={ref_xyz[2]:9.5f} m")

        lines.append(f"  {'Karte':<26s}{'dx':>9s}{'dy':>9s}{'dz':>9s}"
                     f"{'d_xy':>9s}{'d_xyz':>9s}{'droll':>9s}{'dpitch':>9s}"
                     f"{'dyaw':>9s}{'dangle':>9s}")
        lines.append(f"  {'':<26s}{'[mm]':>9s}{'[mm]':>9s}{'[mm]':>9s}"
                     f"{'[mm]':>9s}{'[mm]':>9s}{'[deg]':>9s}{'[deg]':>9s}"
                     f"{'[deg]':>9s}{'[deg]':>9s}")

        for idx, (positions, cfg) in enumerate(zip(maps_xyz, configs)):
            if idx == reference_index:
                continue
            if tag_id not in positions:
                lines.append(f"  {cfg['label']:<26s}"
                             f"{'Tag in dieser Karte nicht vorhanden':>60s}")
                continue

            metrics = tag_metrics(ref_pos[tag_id], ref_rot[tag_id],
                                  positions[tag_id], maps_rot[idx][tag_id])
            lines.append(f"  {cfg['label']:<26s}" +
                         "".join(f"{metrics[key]:9.2f}" for key in METRIC_KEYS))

            xyz = positions[tag_id]
            row = [
                tag_id,
                f"{size:.6f}" if size is not None else "",
                f"{distances[tag_id]:.4f}",
                cfg["label"],
                f"{xyz[0]:.6f}", f"{xyz[1]:.6f}", f"{xyz[2]:.6f}",
                f"{ref_xyz[0]:.6f}", f"{ref_xyz[1]:.6f}", f"{ref_xyz[2]:.6f}",
            ]
            row += [f"{metrics[key]:.4f}" for key in METRIC_KEYS]
            csv_rows.append(row)

    header_row = ["tag_id", "size_m", "entfernung_m", "karte",
                  "x_m", "y_m", "z_m", "ref_x_m", "ref_y_m", "ref_z_m"]
    header_row += [f"{key}_{UNIT_OF[key]}" for key in METRIC_KEYS]
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header_row)
        writer.writerows(csv_rows)

    return "\n".join(lines), csv_path


def _group_report_block(title: str,
                        group_key_label: str,
                        groups: List[Tuple[str, List[int]]],
                        maps_xyz: List[Dict[int, np.ndarray]],
                        maps_rot: List[Dict[int, np.ndarray]],
                        configs: List[dict],
                        reference_index: int,
                        csv_path: str,
                        csv_group_header: str,
                        bar_title: str,
                        bar_xlabel: str,
                        bar_basename: str) -> Tuple[str, List[str]]:
    """Wertet Abweichungen fuer beliebig definierte Tag-Gruppen aus.

    Fuer jede Gruppe und jede Karte werden Bias, Betragsmittel, RMS und Maximum
    aller Kennzahlen berechnet, in eine CSV geschrieben und als Balkendiagramm
    dargestellt.

    Args:
        title: Ueberschrift des Reportblocks.
        group_key_label: Bezeichnung der Gruppierung im Text.
        groups: Liste aus ``(gruppenname, tag_ids)`` in Ausgabereihenfolge.
        maps_xyz: Liste der eingelesenen Karten.
        maps_rot: Liste der Rotationen je Karte.
        configs: Zugehoerige Konfigurationseintraege (Farbe, Label).
        reference_index: Index der Referenzkarte in den Listen.
        csv_path: Zielpfad der CSV-Datei.
        csv_group_header: Spaltenname der Gruppenspalte in der CSV.
        bar_title: Titel des Balkendiagramms.
        bar_xlabel: Beschriftung der x-Achse des Balkendiagramms.
        bar_basename: Dateiname des Balkendiagramms ohne Endung.

    Returns:
        Tuple ``(berichtstext, liste_der_dateipfade)``.
    """
    ref_pos = maps_xyz[reference_index]
    ref_rot = maps_rot[reference_index]
    ref_label = configs[reference_index]["label"]

    lines: List[str] = [title, "=" * len(title)]
    csv_rows: List[list] = []
    series: List[Tuple[str, str, List[Optional[float]]]] = []
    group_labels = [name for name, _ in groups]

    for idx, (positions, cfg) in enumerate(zip(maps_xyz, configs)):
        if idx == reference_index:
            continue
        rotations = maps_rot[idx]

        lines.append(f"\n[{cfg['label']}]  vs.  [{ref_label}]")
        bar_values: List[Optional[float]] = []

        for group_name, tag_ids in groups:
            common = [t for t in tag_ids if t in ref_pos and t in positions]
            if not common:
                lines.append(f"\n  {group_key_label} {group_name}   "
                             f"keine gemeinsamen Tags")
                bar_values.append(None)
                continue

            values = collect_metrics(ref_pos, ref_rot, positions, rotations,
                                     common)
            lines.append(f"\n  {group_key_label} {group_name}   "
                         f"(n = {len(common)})")

            csv_row: List[str] = [cfg["label"], group_name, str(len(common))]
            for key in METRIC_KEYS:
                bias, mean, rms, maximum, max_signed = stat_full(values[key])
                lines.append("    " + format_stats(key, values[key]))
                if key in SIGNED_KEYS:
                    csv_row.append(f"{bias:.4f}")
                csv_row.extend([f"{mean:.4f}", f"{rms:.4f}", f"{maximum:.4f}"])
                if key in SIGNED_KEYS:
                    csv_row.append(f"{max_signed:.4f}")

            csv_rows.append(csv_row)
            bar_values.append(stat_full(values[BAR_METRIC])[1])

        series.append((cfg["label"], cfg["color"], bar_values))

    header_row = ["karte", csv_group_header, "n"]
    for key in METRIC_KEYS:
        unit = UNIT_OF[key]
        if key in SIGNED_KEYS:
            header_row.append(f"{key}_bias_{unit}")
        header_row.extend([f"{key}_mean_{unit}", f"{key}_rms_{unit}",
                           f"{key}_max_{unit}"])
        if key in SIGNED_KEYS:
            header_row.append(f"{key}_max_signed_{unit}")

    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header_row)
        writer.writerows(csv_rows)

    written = [csv_path]
    written += _plot_grouped_bars(
        group_labels=group_labels,
        series=series,
        title=bar_title,
        xlabel=bar_xlabel,
        ylabel=f"mittlere Abweichung {BAR_METRIC} [{UNIT_OF[BAR_METRIC]}]",
        basename=bar_basename,
    )
    return "\n".join(lines), written


def compare_by_size(maps_xyz: List[Dict[int, np.ndarray]],
                    maps_rot: List[Dict[int, np.ndarray]],
                    maps_sizes: List[Dict[int, float]],
                    configs: List[dict],
                    reference_index: int) -> Tuple[str, List[str]]:
    """Wertet die Abweichungen gruppiert nach Markergroesse aus.

    Die Groessenklassen werden aus der Referenzkarte uebernommen, da die
    Kantenlaenge eines Tags in allen Karten identisch ist.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        maps_rot: Liste der Rotationen je Karte.
        maps_sizes: Liste der Tag-Groessen je Karte.
        configs: Zugehoerige Konfigurationseintraege (Farbe, Label).
        reference_index: Index der Referenzkarte in den Listen.

    Returns:
        Tuple ``(berichtstext, liste_der_dateipfade)``.
    """
    ref_sizes = maps_sizes[reference_index]

    groups_by_size: Dict[float, List[int]] = {}
    for tag_id, size in ref_sizes.items():
        groups_by_size.setdefault(size, []).append(tag_id)

    groups = [(f"{size:.3f} m", sorted(tag_ids))
              for size, tag_ids in sorted(groups_by_size.items())]

    return _group_report_block(
        title="Abweichungen je Markergroesse",
        group_key_label="Kantenlaenge",
        groups=groups,
        maps_xyz=maps_xyz,
        maps_rot=maps_rot,
        configs=configs,
        reference_index=reference_index,
        csv_path=os.path.join(OUTPUT_DIR,
                              OUTPUT_BASENAME + "_groessenklassen.csv"),
        csv_group_header="kantenlaenge_m",
        bar_title="Mittlere Abweichung je Markergroesse",
        bar_xlabel="Kantenlaenge des Markers",
        bar_basename=OUTPUT_BASENAME + "_groessenklassen",
    )


def compare_by_ring(maps_xyz: List[Dict[int, np.ndarray]],
                    maps_rot: List[Dict[int, np.ndarray]],
                    configs: List[dict],
                    reference_index: int) -> Tuple[str, List[str]]:
    """Wertet die Abweichungen gestaffelt nach Entfernung zum Ursprung aus.

    Die Zuordnung eines Tags zu einem Entfernungsring erfolgt anhand seiner
    Position in der Referenzkarte, damit alle Karten dieselbe Gruppierung
    verwenden.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        maps_rot: Liste der Rotationen je Karte.
        configs: Zugehoerige Konfigurationseintraege (Farbe, Label).
        reference_index: Index der Referenzkarte in den Listen.

    Returns:
        Tuple ``(berichtstext, liste_der_dateipfade)``.
    """
    ref_pos = maps_xyz[reference_index]
    distances = tag_distances(ref_pos)

    groups_by_ring: Dict[int, List[int]] = {}
    outside: List[int] = []
    for tag_id, distance in distances.items():
        ring_idx = assign_ring(distance)
        if ring_idx is None:
            outside.append(tag_id)
            continue
        groups_by_ring.setdefault(ring_idx, []).append(tag_id)

    groups = []
    for ring_idx in range(len(RING_EDGES) - 1):
        name = ring_label(RING_EDGES[ring_idx], RING_EDGES[ring_idx + 1])
        groups.append((name, sorted(groups_by_ring.get(ring_idx, []))))

    text, written = _group_report_block(
        title=("Abweichungen je Entfernungsring um den Ursprung "
               f"({'x-y' if RING_DISTANCE_2D else '3D'}-Abstand)"),
        group_key_label="Ring",
        groups=groups,
        maps_xyz=maps_xyz,
        maps_rot=maps_rot,
        configs=configs,
        reference_index=reference_index,
        csv_path=os.path.join(OUTPUT_DIR,
                              OUTPUT_BASENAME + "_entfernungsringe.csv"),
        csv_group_header="ring",
        bar_title="Mittlere Abweichung je Entfernungsring",
        bar_xlabel="Entfernung des Markers zum Ursprung",
        bar_basename=OUTPUT_BASENAME + "_entfernungsringe",
    )

    if outside:
        text += (f"\n\n  Hinweis: {len(outside)} Tag(s) liegen ausserhalb des "
                 f"letzten Rings und fehlen in der Auswertung: "
                 f"{sorted(outside)}")

    return text, written


def pairwise_errors(ref_pos: Dict[int, np.ndarray],
                    map_pos: Dict[int, np.ndarray],
                    tag_ids: Sequence[int]
                    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Vergleicht die Abstaende aller Tag-Paare mit denen der Referenzkarte.

    Fuer jedes Paar (i, j) mit i < j wird der euklidische Abstand der beiden
    Tags einmal in der Karte und einmal in der Referenz gebildet. Der Fehler
    ist die vorzeichenbehaftete Differenz, positiv bedeutet, dass die Karte
    den Abstand zu gross abbildet.

    Dieses Mass ist unabhaengig von einer Verschiebung oder Verdrehung der
    gesamten Karte gegenueber der Referenz und zeigt damit die innere
    Massstabstreue der Kartierung.

    Args:
        ref_pos: Positionen der Referenzkarte in Metern.
        map_pos: Positionen der zu pruefenden Karte in Metern.
        tag_ids: Auszuwertende Tag-IDs, muessen in beiden Karten vorhanden sein.

    Returns:
        Tuple aus dem Indexpaar-Array der Form ``(2, P)`` (Indizes in
        ``tag_ids``), den Referenzabstaenden in Metern, den Kartenabstaenden
        in Metern und den Fehlern in Millimetern.
    """
    ref = np.stack([ref_pos[t] for t in tag_ids])
    mapped = np.stack([map_pos[t] for t in tag_ids])
    if PAIR_DISTANCE_2D:
        ref = ref[:, :2]
        mapped = mapped[:, :2]

    rows, cols = np.triu_indices(len(tag_ids), k=1)
    d_ref = np.linalg.norm(ref[rows] - ref[cols], axis=1)
    d_map = np.linalg.norm(mapped[rows] - mapped[cols], axis=1)
    errors = (d_map - d_ref) * 1000.0
    return np.vstack([rows, cols]), d_ref, d_map, errors


def scale_factor(d_ref: np.ndarray, d_map: np.ndarray) -> float:
    """Schaetzt den Massstabsfaktor der Karte gegenueber der Referenz.

    Der Faktor ist die Loesung der Ausgleichsrechnung ``min ||s * d_ref -
    d_map||`` und damit das Verhaeltnis, mit dem die Karte alle Abstaende
    gegenueber der Referenz skaliert. Ein Wert kleiner eins bedeutet, dass die
    Karte insgesamt zu klein ist.

    Args:
        d_ref: Referenzabstaende in Metern.
        d_map: Kartenabstaende in Metern.

    Returns:
        Massstabsfaktor, dimensionslos.
    """
    denominator = float(np.sum(d_ref ** 2))
    if denominator < 1e-12:
        return 1.0
    return float(np.sum(d_ref * d_map) / denominator)


def pair_class(length: float) -> Optional[int]:
    """Ordnet eine Basislaenge einer Klasse aus ``PAIR_EDGES`` zu.

    Die Klassen sind halboffen ``[lower, upper)``; die letzte Klasse schliesst
    die obere Grenze mit ein.

    Args:
        length: Basislaenge in Metern.

    Returns:
        Index der Klasse in ``PAIR_EDGES`` oder ``None``.
    """
    for idx in range(len(PAIR_EDGES) - 1):
        lower = PAIR_EDGES[idx]
        upper = PAIR_EDGES[idx + 1]
        is_last = idx == len(PAIR_EDGES) - 2
        if lower <= length < upper or (is_last and length == upper):
            return idx
    return None


def compare_pairwise(maps_xyz: List[Dict[int, np.ndarray]],
                     configs: List[dict],
                     reference_index: int) -> Tuple[str, List[str]]:
    """Wertet die euklidischen Abstaende der Tags untereinander aus.

    Verglichen wird immer gegen die Referenzkarte. Ausgegeben werden die
    Gesamtstatistik je Karte, der geschaetzte Massstabsfaktor, eine Staffelung
    nach Basislaenge, eine CSV mit allen Paaren und ein Balkendiagramm.

    Args:
        maps_xyz: Liste der eingelesenen Karten.
        configs: Zugehoerige Konfigurationseintraege (Farbe, Label).
        reference_index: Index der Referenzkarte in den Listen.

    Returns:
        Tuple ``(berichtstext, liste_der_dateipfade)``.
    """
    ref_pos = maps_xyz[reference_index]
    ref_label = configs[reference_index]["label"]
    csv_path = os.path.join(OUTPUT_DIR, OUTPUT_BASENAME + "_paarweise.csv")

    dimension = "x-y" if PAIR_DISTANCE_2D else "3D"
    title = (f"Paarweise Abstaende der Tags untereinander gegen "
             f"'{ref_label}' ({dimension})")
    lines: List[str] = [title, "=" * len(title)]
    lines.append("Positiver Fehler bedeutet, dass die Karte den Abstand "
                 "zu gross abbildet.")

    class_labels = [ring_label(PAIR_EDGES[i], PAIR_EDGES[i + 1])
                    for i in range(len(PAIR_EDGES) - 1)]
    series: List[Tuple[str, str, List[Optional[float]]]] = []
    csv_rows: List[list] = []

    for idx, (positions, cfg) in enumerate(zip(maps_xyz, configs)):
        if idx == reference_index:
            continue

        common = sorted(set(ref_pos.keys()) & set(positions.keys()))
        lines.append(f"\n[{cfg['label']}]  vs.  [{ref_label}]")
        if len(common) < 2:
            lines.append("  Weniger als zwei gemeinsame Tags, "
                         "kein Paarvergleich moeglich.")
            series.append((cfg["label"], cfg["color"],
                           [None] * len(class_labels)))
            continue

        pairs, d_ref, d_map, errors = pairwise_errors(ref_pos, positions,
                                                      common)
        factor = scale_factor(d_ref, d_map)

        lines.append(f"  Gemeinsame Tags: {len(common)}   "
                     f"Paare: {pairs.shape[1]}")
        lines.append("  " + format_stats("d_pair", errors))
        lines.append(f"  Massstabsfaktor: {factor:.6f}   "
                     f"entspricht {(factor - 1.0) * 1000.0:+.3f} mm je Meter "
                     f"Basislaenge")

        classes = np.array([pair_class(length) if pair_class(length) is not None
                            else -1 for length in d_ref])

        lines.append(f"\n  {'Basislaenge':<12s}{'n':>7s}{'bias':>10s}"
                     f"{'mean':>10s}{'rms':>10s}{'max':>10s}"
                     f"{'max_signed':>12s}   [mm]")
        bar_values: List[Optional[float]] = []
        for class_idx, label in enumerate(class_labels):
            mask = classes == class_idx
            count = int(mask.sum())
            if count == 0:
                lines.append(f"  {label:<12s}{0:>7d}"
                             f"{'keine Paare':>42s}")
                bar_values.append(None)
                continue
            bias, mean, rms, maximum, max_signed = stat_full(errors[mask])
            lines.append(f"  {label:<12s}{count:>7d}{bias:>+10.2f}"
                         f"{mean:>10.2f}{rms:>10.2f}{maximum:>10.2f}"
                         f"{max_signed:>+12.2f}")
            bar_values.append(mean)

        outside = int((classes == -1).sum())
        if outside:
            lines.append(f"  Hinweis: {outside} Paar(e) liegen ausserhalb "
                         f"der letzten Klasse und fehlen in der Staffelung.")

        series.append((cfg["label"], cfg["color"], bar_values))

        for pair_idx in range(pairs.shape[1]):
            class_idx = classes[pair_idx]
            relative = (errors[pair_idx] / (d_ref[pair_idx] * 1000.0) * 1e6
                        if d_ref[pair_idx] > 1e-9 else 0.0)
            csv_rows.append([
                cfg["label"],
                common[pairs[0, pair_idx]],
                common[pairs[1, pair_idx]],
                class_labels[class_idx] if class_idx >= 0 else "",
                f"{d_ref[pair_idx]:.6f}",
                f"{d_map[pair_idx]:.6f}",
                f"{errors[pair_idx]:.4f}",
                f"{relative:.2f}",
            ])

    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["karte", "tag_a", "tag_b", "basislaenge_klasse",
                         "d_ref_m", "d_karte_m", "fehler_mm",
                         "rel_fehler_ppm"])
        writer.writerows(csv_rows)

    written = [csv_path]
    written += _plot_grouped_bars(
        group_labels=class_labels,
        series=series,
        title="Mittlerer Fehler des paarweisen Tag-Abstands",
        xlabel="Basislaenge des Tag-Paars in der Referenz",
        ylabel="mittlerer Fehler |d_pair| [mm]",
        basename=OUTPUT_BASENAME + "_paarweise",
    )
    return "\n".join(lines), written


def main() -> None:
    """Fuehrt Einlesen, grafischen und numerischen Vergleich vollstaendig aus."""
    apply_font_sizes()

    # Nur konfigurierte Karten mit existierender Datei verwenden
    active: List[dict] = []
    for cfg in MAPS:
        path = cfg.get("path", "")
        if not path:
            continue
        if not os.path.isfile(path):
            print(f"Hinweis: Datei '{path}' nicht gefunden, Karte "
                  f"'{cfg.get('label', path)}' wird uebersprungen.")
            continue
        active.append(cfg)

    if not active:
        sys.exit("Keine gueltige Karte gefunden. Bitte Pfade in MAPS pruefen.")
    if len(active) > 4:
        print("Hinweis: mehr als vier Karten konfiguriert, es werden nur die "
              "ersten vier verwendet.")
        active = active[:4]

    if BAR_METRIC not in POS_KEYS:
        sys.exit(f"BAR_METRIC '{BAR_METRIC}' ist unbekannt. "
                 f"Erlaubt: {POS_KEYS}")
    if PER_TAG_POS_METRIC not in POS_KEYS:
        sys.exit(f"PER_TAG_POS_METRIC '{PER_TAG_POS_METRIC}' ist unbekannt. "
                 f"Erlaubt: {POS_KEYS}")
    if PER_TAG_ROT_METRIC not in ROT_KEYS:
        sys.exit(f"PER_TAG_ROT_METRIC '{PER_TAG_ROT_METRIC}' ist unbekannt. "
                 f"Erlaubt: {ROT_KEYS}")

    print(f"Lade {len(active)} Karte(n) ...")
    loaded = [load_map(cfg["path"]) for cfg in active]
    maps_xyz = [entry[0] for entry in loaded]
    maps_rot = [entry[1] for entry in loaded]
    maps_sizes = [entry[2] for entry in loaded]
    for cfg, positions in zip(active, maps_xyz):
        print(f"  '{cfg['label']}': {len(positions)} Tags aus '{cfg['path']}'")

    limits = compute_axis_limits(maps_xyz, AXIS_PADDING)

    written: List[str] = []
    written += plot_overlay(maps_xyz, active, limits)
    written += plot_subplots(maps_xyz, active, limits)
    written += plot_overlay_with_rings(maps_xyz, active, limits)

    # Referenzindex auf die aktive Liste abbilden (robust bei uebersprungenen Karten)
    ref_index = REFERENCE_INDEX if REFERENCE_INDEX < len(active) else 0

    report_path = os.path.join(OUTPUT_DIR, OUTPUT_BASENAME + "_report.txt")

    if len(active) >= 2:
        written += plot_per_tag_distance(maps_xyz, maps_rot, active, ref_index)
        written += plot_distance_to_tag(maps_xyz, maps_rot, active, ref_index)

        blocks: List[str] = []

        text, csv_path = compare_numeric(maps_xyz, maps_rot, active, ref_index)
        blocks.append(text)
        written.append(csv_path)

        text, csv_path = compare_single_tags(maps_xyz, maps_rot, maps_sizes,
                                             active, ref_index, SINGLE_TAG_IDS)
        blocks.append(text)
        written.append(csv_path)

        text, paths = compare_by_size(maps_xyz, maps_rot, maps_sizes, active,
                                      ref_index)
        blocks.append(text)
        written += paths

        text, paths = compare_by_ring(maps_xyz, maps_rot, active, ref_index)
        blocks.append(text)
        written += paths

        text, paths = compare_pairwise(maps_xyz, active, ref_index)
        blocks.append(text)
        written += paths

        report_text = "\n\n\n".join(blocks)
        with open(report_path, "w", encoding="utf-8") as handle:
            handle.write(report_text + "\n")
        written.append(report_path)

        print("\n" + report_text + "\n")
    else:
        print("\nNur eine Karte aktiv, numerischer Vergleich entfaellt.")

    print("Geschriebene Dateien:")
    for path in written:
        print(f"  {path}")


if __name__ == "__main__":
    main()
