#!/usr/bin/env python3
"""Erzeugt eine TagSLAM-Ground-Truth-Map aus einer Marker-Konfiguration.

Liest eine YAML-Datei im Format von ``marker_config_1.yaml`` (Feldgeometrie und
Markerliste mit Position und Groesse je Tag) und schreibt eine Karte im Format
von ``beispiel_map.yaml``. Die Karte kann in TagSLAM als bekannte, statische
Referenz (Ground Truth) verwendet werden.

Annahmen
--------
Ebene Lage
    Alle Tags liegen flach in einer Ebene (z = 0) mit der Vorderseite nach oben.
Ursprung
    Der Tag mit ``origin_id`` (Default 0) definiert den Ursprung des
    Map-Koordinatensystems. Seine Position wird von allen Tags subtrahiert,
    sodass er bei (0, 0, 0) liegt.
Achsen
    Die Config-Achsen werden direkt uebernommen: Map-x = Config-x, Map-y =
    Config-y. Damit gilt fuer Tag i: ``pos = (x_i - x0, y_i - y0, 0)``.
Orientierung
    Alle Tags sind identisch ausgerichtet, daher relativ zum Ursprung keine
    Verdrehung. Die Rotation wird als Rodrigues-Vektor (Achse * Winkel, in rad)
    ausgegeben und ist fuer jedes Tag (0, 0, 0). Bei Rotation (0, 0, 0) zeigt
    die Tag-z-Achse (Flaechennormale) in dieselbe Richtung wie beim Ursprung,
    also nach oben.
Rauschen
    Positions- und Rotationsrauschen werden je Tag fest auf ``tag_noise``
    (Default 1e-6) gesetzt, sodass die Karte als starre Ground Truth wirkt.

Aufruf
------
    python3 ground_truth_map_generierung.py [konfig.yaml] [map.yaml]

Ohne Argumente werden ``marker_config_1.yaml`` gelesen und ``ground_truth_map.yaml``
geschrieben.
"""

from __future__ import annotations

import sys
from typing import Any

import yaml


def _fmt(value: float) -> str:
    """Formatiert eine Zahl mit acht Nachkommastellen (Map-Stil).

    Parameters
    ----------
    value : float
        Zu formatierender Wert.

    Returns
    -------
    str
        Zahl als String mit acht Nachkommastellen; ``-0.00000000`` wird auf
        ``0.00000000`` normalisiert.
    """
    s = f"{value:.8f}"
    if s == "-0.00000000":
        s = "0.00000000"
    return s


def _pose_block(indent: int, pos: tuple[float, float, float],
                rot: tuple[float, float, float], pos_noise: float,
                rot_noise: float) -> list[str]:
    """Baut den ``pose``-Block (Position, Rotation, Rauschen) als Zeilenliste.

    Parameters
    ----------
    indent : int
        Einrueckung des Schluessels ``pose`` in Leerzeichen.
    pos : tuple[float, float, float]
        Position (x, y, z) in Metern.
    rot : tuple[float, float, float]
        Rotation als Rodrigues-Vektor (x, y, z) in Radiant.
    pos_noise : float
        Positionsrauschen (identisch fuer x, y, z).
    rot_noise : float
        Rotationsrauschen (identisch fuer x, y, z).

    Returns
    -------
    list[str]
        Formatierte YAML-Zeilen des ``pose``-Blocks.
    """
    p = " " * indent
    p2 = " " * (indent + 2)
    p4 = " " * (indent + 4)
    lines = [f"{p}pose:"]
    for key, (vx, vy, vz) in (
        ("position", pos),
        ("rotation", rot),
        ("position_noise", (pos_noise, pos_noise, pos_noise)),
        ("rotation_noise", (rot_noise, rot_noise, rot_noise)),
    ):
        lines.append(f"{p2}{key}:")
        lines.append(f"{p4}x: {_fmt(vx)}")
        lines.append(f"{p4}y: {_fmt(vy)}")
        lines.append(f"{p4}z: {_fmt(vz)}")
    return lines


def build_ground_truth_map(config: dict[str, Any], origin_id: int = 0,
                           default_tag_size: float = 0.170,
                           tag_noise: float = 1e-6,
                           body_noise: float = 1.0) -> str:
    """Erzeugt den YAML-Text der Ground-Truth-Map aus der Config.

    Parameters
    ----------
    config : dict[str, Any]
        Eingelesene Marker-Konfiguration (mit Schluessel ``markers``; jeder
        Eintrag ``id``, ``x``, ``y``, ``size``).
    origin_id : int
        ID des Ursprungs-Tags. Dessen Position wird zu (0, 0, 0).
    default_tag_size : float
        Wert fuer ``default_tag_size`` des ``tag_map``-Body.
    tag_noise : float
        Positions- und Rotationsrauschen je Tag (starre Ground Truth).
    body_noise : float
        Positions- und Rotationsrauschen der Body-Pose.

    Returns
    -------
    str
        Vollstaendiger YAML-Text im Format von ``beispiel_map.yaml``.

    Raises
    ------
    KeyError
        Wenn ``markers`` fehlt.
    ValueError
        Wenn ``origin_id`` nicht in der Markerliste vorkommt.
    """
    markers = config.get("markers")
    if not markers:
        raise KeyError("Config: Abschnitt 'markers' fehlt oder ist leer.")

    by_id = {int(m["id"]): m for m in markers}
    if origin_id not in by_id:
        raise ValueError(f"origin_id {origin_id} nicht in der Markerliste.")
    x0 = float(by_id[origin_id]["x"])
    y0 = float(by_id[origin_id]["y"])

    lines: list[str] = ["bodies:"]

    # --- Body: tag_map (statische Ground Truth) --------------------------
    lines.append(" - tag_map:")
    lines.append("     type: simple")
    lines.append("     is_static: true")
    lines.append(f"     default_tag_size: {_fmt(default_tag_size)}")
    lines += _pose_block(5, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                         body_noise, body_noise)
    lines.append("     tags:")

    for tag_id in sorted(by_id):
        m = by_id[tag_id]
        px = float(m["x"]) - x0
        py = float(m["y"]) - y0
        size = float(m["size"])
        lines.append(f"       - id: {tag_id}")
        lines.append(f"         size: {_fmt(size)}")
        lines += _pose_block(9, (px, py, 0.0), (0.0, 0.0, 0.0),
                             tag_noise, tag_noise)

    lines.append("       ")

    # --- Body: camera (dynamisch, wie im Beispiel) -----------------------
    lines.append(" - camera:")
    lines.append("     type: simple")
    lines.append("     is_static: false")
    lines.append("     default_tag_size: 0.00000000")
    lines.append("     tags:")

    return "\n".join(lines) + "\n"


def main() -> None:
    """Liest Config- und Ausgabepfad aus argv und schreibt die Map."""
    config_path = sys.argv[1] if len(sys.argv) > 1 else "marker_config_1.yaml"
    map_path = sys.argv[2] if len(sys.argv) > 2 else "ground_truth_map.yaml"

    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}

    text = build_ground_truth_map(config)
    with open(map_path, "w", encoding="utf-8") as f:
        f.write(text)

    n = len(config.get("markers", []))
    print(f"{n} Tags gelesen aus {config_path}")
    print(f"Ground-Truth-Map geschrieben: {map_path}")


if __name__ == "__main__":
    main()