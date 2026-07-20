#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ulog_auswertung.py
==================
Auswertungsskript für PX4-ULog-Dateien eines Testflugs mit Vision-gestützter
GNSS-freier Lokalisierung (External Vision / EKF2_EV_CTRL).

Schwerpunkte:
  * EKF-Zustände (Position, Geschwindigkeit, Lage, Sensor-Biases)
  * Vision-Vorgabe (External Vision) vs. EKF-Schätzung vs. Offboard-Sollwert
  * Tatsächlich geflogene Bahn (2D / 3D / Höhe)
  * Fusions-Gesundheit (Innovationen, Test-Ratios, Fused/Rejected, Aussetzer)
  * EKF-Unsicherheit (Kovarianz) und Vertrauen in neue Messungen
  * Yaw-Fusion, Höhenquellen-Vergleich, Vergleich der EKF-Instanzen
  * Flugmodi, Arming, Land-Detector

In JEDEM Diagramm werden Arming-Zeitpunkt (grün) und Landung (rot) markiert,
die Vision-Aussetzer orange hinterlegt (Konfigurationsblock AUSSETZER_*),
jedes Diagramm hat eine eigene Zeitachse und eine einheitliche Legende.
Die Abkürzung EV (External Vision) wird in allen Beschriftungen als Vision
ausgeschrieben, in den PX4-Bezeichnern (EKF2_EV_CTRL, cs_ev_pos,
estimator_aid_src_ev_pos) bleibt sie unverändert.

Hinweis zur Kovarianz-Indizierung:
    PX4 v1.16 loggt in estimator_states 25 Zustände, aber nur eine
    24-dimensionale Fehlerzustands-Kovarianz (Lagefehler 3D statt
    Quaternion 4D). Die Kovarianz-Blöcke sind daher gegenüber dem
    Zustandsvektor um einen Index verschoben. Das Skript erkennt die
    Indizierung automatisch anhand der Feldanzahl.

Aufruf:
    # (a) ganzer Flug:
    python3 ulog_auswertung.py <pfad/zur/datei.ulg>

    # (b) nur ein Zeitfenster (Sekunden seit Logstart), Plots werden skaliert:
    python3 ulog_auswertung.py <pfad/zur/datei.ulg> --tmin 240 --tmax 266

    # weitere Optionen:
    python3 ulog_auswertung.py <datei.ulg> --out ordner/ --instance 0 --dropout-schwelle 1.5

Die Aussetzer-Schwelle wird standardmäßig automatisch aus dem
Log-Intervall der Vision-Topics abgeleitet (2.5 x Median-Intervall), da die
estimator_aid_src-Topics vom Logger nur mit reduzierter Rate (~2 Hz)
aufgezeichnet werden. Mit --dropout-schwelle lässt sie sich fest vorgeben.

Welche Plots erzeugt werden, steuerst du im Konfigurationsblock PLOTS_AKTIV
direkt unter den Imports (True = an, False = aus).

Abhängigkeiten:
    pip install pyulog numpy matplotlib
"""

import argparse
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")  # Headless, schreibt nur Dateien
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

try:
    from pyulog import ULog
except ImportError:
    sys.exit("Fehler: pyulog nicht installiert. -> pip install pyulog")


# ===========================================================================
# KONFIGURATION: Plots aktivieren (True) oder deaktivieren (False)
# ===========================================================================
PLOTS_AKTIV = {
    "01_position_vision_ekf_sollwert": True,
    "02_bahn_2d_3d": True,
    "03_geschwindigkeit": True,
    "04_lage": True,
    "05_test_ratios": True,
    "06_ev_innovationen": True,
    "07_ekf_biases": True,
    "08_fusion_status": True,
    "09_flugmodi": True,
    "10_aussetzer_resets": True,
    "11_kovarianz": True,
    "12_vertrauen": True,
    "13_yaw_detail": True,
    "14_hoehenquellen": True,
    "15_ekf_instanzen": True,
    "16_stuetzquellen": True,
}

# Einheitliche Darstellung
LEGENDE_LOC = "upper right"     # einheitliche Legendenposition pro Subplot
EREIGNIS_FARBEN = {"Armed": "green", "Landung": "red"}

# ---------------------------------------------------------------------------
# Vision-Aussetzer in jedem Zeitdiagramm hinterlegen
# ---------------------------------------------------------------------------
AUSSETZER_UEBERALL = True          # False = nur dort, wo der Plot es selbst macht
AUSSETZER_QUELLE = "aid_src"       # "aid_src" (wie bisher) oder "flags" (cs_ev_pos)
AUSSETZER_AUSNAHMEN = {"16_stuetzquellen"}   # Plots ohne automatische Schattierung
AUSSETZER_FARBE = "orange"
AUSSETZER_ALPHA = 0.25
AUSSETZER_LABEL = "Vision-Aussetzer"

# ---------------------------------------------------------------------------
# Schriftgrößen für ALLE Plots (hier zentral ändern)
# ---------------------------------------------------------------------------
SCHRIFTGROESSEN = {
    "figur_titel": 18,          # fig.suptitle
    "subplot_titel": 11,        # ax.set_title
    "achsenbeschriftung": 15,   # ax.set_xlabel / ax.set_ylabel, z. B. "Zeit [s]"
    "ticks": 10,                # Zahlen und Kategorienamen an den Achsen
    "legende": 8,               # Legendeneinträge
    "annotation": 9,            # Textmarken im Plot, z. B. Prozentwerte
    "fussnote": 7,              # Hinweis auf das Zeitfenster
}

# Abweichende Schriftgrößen für einzelne Plots (Name wie in PLOT_FUNCS).
# Nicht genannte Schlüssel werden aus SCHRIFTGROESSEN übernommen.
SCHRIFT_JE_PLOT = {
    "16_stuetzquellen": {
        "achsenbeschriftung": 14,
        "ticks": 14,
        "annotation": 12,
        "subplot_titel": 14,
    },
}
# ===========================================================================


NAV_STATE_NAMES = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
    4: "AUTO_LOITER", 5: "AUTO_RTL", 10: "ACRO", 12: "DESCEND",
    13: "TERMINATION", 14: "OFFBOARD", 15: "STAB", 17: "AUTO_TAKEOFF",
    18: "AUTO_LAND", 19: "AUTO_FOLLOW", 20: "AUTO_PRECLAND", 21: "ORBIT",
}


# ---------------------------------------------------------------------------
# Schriftgrößen-Steuerung
# ---------------------------------------------------------------------------
_SCHRIFT_AKTUELL = dict(SCHRIFTGROESSEN)


def sg(schluessel):
    """Gibt die aktuell gueltige Schriftgroesse zu einem Schluessel zurueck.

    Args:
        schluessel: Schluessel aus SCHRIFTGROESSEN, etwa "subplot_titel".

    Returns:
        Schriftgroesse in Punkt.
    """
    return _SCHRIFT_AKTUELL.get(schluessel, SCHRIFTGROESSEN.get(schluessel, 10))


def setze_schriftgroessen(plot_name=None):
    """Aktiviert die Schriftgroessen fuer den naechsten Plot.

    Kombiniert SCHRIFTGROESSEN mit den plotspezifischen Werten aus
    SCHRIFT_JE_PLOT und uebertraegt sie in die matplotlib-rcParams, sodass
    auch nicht explizit gesetzte Elemente (Ticks, Achsenbeschriftungen)
    mitskalieren.

    Args:
        plot_name: Name des Plots wie in PLOT_FUNCS oder None fuer die
            globalen Vorgaben.

    Returns:
        Dict mit den aktiven Schriftgroessen.
    """
    global _SCHRIFT_AKTUELL
    groessen = dict(SCHRIFTGROESSEN)
    if plot_name:
        groessen.update(SCHRIFT_JE_PLOT.get(plot_name, {}))
    _SCHRIFT_AKTUELL = groessen
    plt.rcParams.update({
        "font.size": groessen["ticks"],
        "axes.titlesize": groessen["subplot_titel"],
        "axes.labelsize": groessen["achsenbeschriftung"],
        "xtick.labelsize": groessen["ticks"],
        "ytick.labelsize": groessen["ticks"],
        "legend.fontsize": groessen["legende"],
        "figure.titlesize": groessen["figur_titel"],
    })
    return groessen


# ---------------------------------------------------------------------------
# Zentrale Vision-Aussetzer fuer alle Zeitdiagramme
# ---------------------------------------------------------------------------
_AUSSETZER_AKTUELL = []


def ermittle_aussetzer(ulog, inst, schwelle, fenster):
    """Ermittelt die Vision-Aussetzer gemaess der Einstellung AUSSETZER_QUELLE.

    "aid_src" wertet estimator_aid_src_ev_pos aus (identisch zur bisherigen
    Schattierung), "flags" nutzt cs_ev_pos aus estimator_status_flags, das mit
    hoeherer Rate geloggt wird und auch kurze Unterbrechungen aufloest.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        schwelle: Aussetzer-Schwelle in Sekunden (nur fuer "aid_src").
        fenster: (t0, t1).

    Returns:
        Liste von (t_start, t_ende, dauer) in Sekunden.
    """
    if AUSSETZER_QUELLE == "flags":
        return finde_ev_aussetzer_flags(ulog, inst, fenster)
    return finde_ev_aussetzer(ulog, inst, schwelle, fenster)


def setze_aussetzer(aussetzer):
    """Hinterlegt die Aussetzerliste, die _finalisiere_zeit automatisch zeichnet.

    Args:
        aussetzer: Liste von (t_start, t_ende, dauer) oder leere Liste.

    Returns:
        Die gesetzte Liste.
    """
    global _AUSSETZER_AKTUELL
    _AUSSETZER_AKTUELL = list(aussetzer or [])
    return _AUSSETZER_AKTUELL


# ---------------------------------------------------------------------------
# Basis-Hilfsfunktionen
# ---------------------------------------------------------------------------
def lade_ulog(pfad):
    """Lädt eine ULog-Datei und gibt das ULog-Objekt zurück.

    Args:
        pfad: Pfad zur .ulg-Datei.

    Returns:
        ULog-Objekt.
    """
    if not os.path.isfile(pfad):
        sys.exit(f"Fehler: Datei nicht gefunden: {pfad}")
    return ULog(pfad)


def hole(ulog, name, mid=0):
    """Gibt den Datensatz eines Topics (mit Multi-Instanz-ID) zurück oder None.

    Args:
        ulog: ULog-Objekt.
        name: Topic-Name.
        mid: Multi-Instanz-ID (Standard 0).

    Returns:
        pyulog-Datenobjekt oder None.
    """
    for d in ulog.data_list:
        if d.name == name and d.multi_id == mid:
            return d
    return None


def zeit(ulog, datensatz):
    """Rechnet Zeitstempel in Sekunden seit Logstart um (uint64-sicher).

    Args:
        ulog: ULog-Objekt.
        datensatz: pyulog-Datenobjekt.

    Returns:
        numpy-Array mit Zeit in Sekunden.
    """
    t0 = np.int64(ulog.start_timestamp)
    return (datensatz.data["timestamp"].astype(np.int64) - t0) / 1e6


def quat_zu_euler(w, x, y, z):
    """Wandelt Quaternionen (w, x, y, z) in Euler-Winkel (roll, pitch, yaw) [rad].

    Args:
        w, x, y, z: Quaternion-Komponenten (skalar-zuerst).

    Returns:
        Tupel (roll, pitch, yaw) in Radiant.
    """
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def kovarianz_indizes(st):
    """Ermittelt die Kovarianz-Blockindizes passend zur EKF-Version.

    PX4 v1.16 (25 Zustände, 24 Kovarianzen, Fehlerzustands-Formulierung):
        0-2 Lagefehler, 3-5 Geschw. NED, 6-8 Pos NED,
        9-11 Gyro-Bias, 12-14 Accel-Bias, 15-20 Mag, 21-22 Wind, 23 Terrain.
    Ältere Versionen (24 Zustände, 24 Kovarianzen, Quaternion im Zustand):
        0-3 Quaternion, 4-6 Geschw. NED, 7-9 Pos NED,
        10-12 Gyro-Bias, 13-15 Accel-Bias, 16-21 Mag, 22-23 Wind.

    Args:
        st: pyulog-Datenobjekt von estimator_states.

    Returns:
        Dict mit Startindizes {"vel", "pos", "gyro_bias", "accel_bias"}.
    """
    n_cov = sum(1 for k in st.data if k.startswith("covariances["))
    n_states = int(st.data["n_states"][0]) if "n_states" in st.data \
        else sum(1 for k in st.data if k.startswith("states["))
    if n_states > n_cov:
        # Fehlerzustands-Kovarianz (v1.16): Lagefehler 3D -> Blöcke um 1 verschoben
        return {"vel": 3, "pos": 6, "gyro_bias": 9, "accel_bias": 12}
    return {"vel": 4, "pos": 7, "gyro_bias": 10, "accel_bias": 13}


def voller_zeitraum(ulog):
    """Ermittelt den gesamten Zeitraum des Logs (auf Basis vehicle_local_position).

    Args:
        ulog: ULog-Objekt.

    Returns:
        Tupel (t_start, t_ende) in Sekunden.
    """
    vlp = hole(ulog, "vehicle_local_position")
    t = zeit(ulog, vlp)
    return float(t[0]), float(t[-1])


def airborne_fenster(ulog):
    """Ermittelt das Zeitfenster, in dem die Drohne in der Luft war.

    Args:
        ulog: ULog-Objekt.

    Returns:
        Tupel (t_start, t_ende) in Sekunden oder (None, None).
    """
    ld = hole(ulog, "vehicle_land_detected")
    if ld is None:
        return None, None
    t = zeit(ulog, ld)
    luft = t[ld.data["landed"] == 0]
    if len(luft) == 0:
        return None, None
    return float(luft[0]), float(luft[-1])


def ereigniszeiten(ulog):
    """Ermittelt die Zeitpunkte von Arming und Landung.

    Args:
        ulog: ULog-Objekt.

    Returns:
        Dict {"Armed": t oder None, "Landung": t oder None} in Sekunden.
    """
    out = {"Armed": None, "Landung": None}
    vs = hole(ulog, "vehicle_status")
    if vs is not None:
        t = zeit(ulog, vs)
        m = (vs.data["arming_state"] == 2) & (t >= 0)
        if m.any():
            out["Armed"] = float(t[m][0])
    ld = hole(ulog, "vehicle_land_detected")
    if ld is not None:
        t = zeit(ulog, ld)
        landed = ld.data["landed"].astype(int)
        tr = np.where(np.diff(landed) == 1)[0] + 1  # Übergang in der Luft -> gelandet
        if len(tr):
            out["Landung"] = float(t[tr[-1]])
    return out


def auto_dropout_schwelle(ulog, inst, faktor=2.5, minimum=1.0):
    """Leitet die Aussetzer-Schwelle aus dem Log-Intervall der Vision-Topics ab.

    Die estimator_aid_src-Topics werden vom Logger mit reduzierter Rate
    aufgezeichnet (typisch ~2 Hz). Damit Logger-Subsampling nicht als
    Aussetzer gewertet wird, wird die Schwelle als Vielfaches des
    Median-Log-Intervalls gesetzt.

    Args:
        ulog: ULog-Objekt.
        inst: Primäre EKF-Instanz-ID.
        faktor: Vielfaches des Median-Intervalls (Standard 2.5).
        minimum: Untergrenze der Schwelle in Sekunden.

    Returns:
        Schwelle in Sekunden.
    """
    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    if ev is None:
        return minimum
    t = zeit(ulog, ev)
    if len(t) < 3:
        return minimum
    med = float(np.median(np.diff(t)))
    return max(minimum, faktor * med)


def finde_ev_aussetzer(ulog, inst, schwelle_s, fenster=None):
    """Findet Aussetzer (Lücken) in der External-Vision-Fusion.

    Args:
        ulog: ULog-Objekt.
        inst: Primäre EKF-Instanz-ID.
        schwelle_s: Mindestlücke in Sekunden ab der ein Aussetzer zählt.
        fenster: Optionales (t0, t1); nur Aussetzer mit Überlapp werden zurückgegeben.

    Returns:
        Liste von (t_start, t_ende, dauer) in Sekunden.
    """
    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    if ev is None:
        return []
    t = zeit(ulog, ev)
    tf = t[ev.data["fused"].astype(bool)]
    if len(tf) < 2:
        return []
    luecken = np.diff(tf)
    idx = np.where(luecken > schwelle_s)[0]
    res = [(float(tf[i]), float(tf[i + 1]), float(luecken[i])) for i in idx]
    if fenster is not None:
        res = [a for a in res if a[1] >= fenster[0] and a[0] <= fenster[1]]
    return res


def finde_ev_aussetzer_flags(ulog, inst, fenster=None):
    """Findet Unterbrechungen der EV-Positionsstuetzung anhand von cs_ev_pos.

    Gegenueber finde_ev_aussetzer wird nicht estimator_aid_src_ev_pos
    ausgewertet, das der Logger nur mit rund 2 Hz aufzeichnet, sondern das
    Statusflag cs_ev_pos aus estimator_status_flags. Dieses liegt mit
    hoeherer Rate vor und loest kurze Unterbrechungen auf.

    Args:
        ulog: ULog-Objekt.
        inst: Primaere EKF-Instanz-ID.
        fenster: Optionales (t0, t1); nur Aussetzer mit Ueberlapp werden zurueckgegeben.

    Returns:
        Liste von (t_start, t_ende, dauer) in Sekunden.
    """
    sf = hole(ulog, "estimator_status_flags", inst)
    if sf is None or "cs_ev_pos" not in sf.data:
        return []
    t = zeit(ulog, sf)
    v = sf.data["cs_ev_pos"].astype(int)
    wechsel = np.where(np.diff(v) != 0)[0] + 1
    res = []
    for k, i in enumerate(wechsel):
        if v[i] == 0:
            ende = float(t[wechsel[k + 1]]) if k + 1 < len(wechsel) else float(t[-1])
            res.append((float(t[i]), ende, ende - float(t[i])))
    if fenster is not None:
        res = [a for a in res if a[1] >= fenster[0] and a[0] <= fenster[1]]
    return res


def flag_anteil(ulog, inst, feld, fenster):
    """Berechnet den zeitgewichteten Anteil, in dem ein Statusflag gesetzt ist.

    Eine ungewichtete Mittelung ueber die Logeintraege verzerrt das Ergebnis,
    da estimator_status_flags nicht aequidistant aufgezeichnet wird.

    Args:
        ulog: ULog-Objekt.
        inst: Primaere EKF-Instanz-ID.
        feld: Name des Flags, etwa "cs_ev_pos".
        fenster: (t0, t1) ausgewerteter Zeitraum.

    Returns:
        Anteil in Prozent oder NaN, falls das Flag fehlt.
    """
    sf = hole(ulog, "estimator_status_flags", inst)
    if sf is None or feld not in sf.data:
        return float("nan")
    t = zeit(ulog, sf)
    m = maske(t, fenster)
    tt, vv = t[m], sf.data[feld][m].astype(float)
    if len(tt) < 2:
        return float("nan")
    d = np.diff(tt)
    return 100.0 * float(np.sum(d * vv[:-1]) / np.sum(d))


def finde_resets(ulog, inst, fenster=None):
    """Ermittelt die Zeitpunkte der EKF-Resets.

    Hinweis: Das Feld reset_count_pod_d ist ein Tippfehler in der
    PX4-Message-Definition selbst und heißt dort tatsächlich so.

    Args:
        ulog: ULog-Objekt.
        inst: Primäre EKF-Instanz-ID.
        fenster: Optionales (t0, t1) zum Filtern.

    Returns:
        Dict {Bezeichnung: numpy-Array mit Reset-Zeitpunkten [s]}.
    """
    es = hole(ulog, "estimator_status", inst)
    if es is None:
        return {}
    t = zeit(ulog, es)
    felder = {"Pos NE": "reset_count_pos_ne", "Pos D": "reset_count_pod_d",
              "Vel NE": "reset_count_vel_ne", "Vel D": "reset_count_vel_d",
              "Quat": "reset_count_quat"}
    out = {}
    for name, feld in felder.items():
        if feld in es.data:
            c = es.data[feld].astype(np.int64)
            rt = t[np.where(np.diff(c) > 0)[0] + 1]
            if fenster is not None:
                rt = rt[(rt >= fenster[0]) & (rt <= fenster[1])]
            out[name] = rt
    return out


def maske(t, fenster):
    """Boolesche Maske der Zeitpunkte innerhalb des Fensters.

    Args:
        t: Zeit-Array in Sekunden.
        fenster: (t0, t1).

    Returns:
        Boolesches numpy-Array.
    """
    return (t >= fenster[0]) & (t <= fenster[1])


def wickle_grad(winkel_deg):
    """Wickelt Winkel in Grad auf den Bereich (-180, 180].

    Args:
        winkel_deg: Winkel-Array in Grad.

    Returns:
        Gewickeltes numpy-Array in Grad.
    """
    return (np.asarray(winkel_deg) + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------
# Darstellungs-Helfer (Marker, Legende, Zeitachse)
# ---------------------------------------------------------------------------
def _markiere_zeit(ax, ereignisse, fenster):
    """Zeichnet vertikale Marker für Arming/Landung in eine Zeitachse.

    Args:
        ax: matplotlib-Achse.
        ereignisse: Dict {Name: Zeit}.
        fenster: (t0, t1) zur Sichtbarkeitsprüfung.
    """
    for name, t in ereignisse.items():
        if t is None or not (fenster[0] <= t <= fenster[1]):
            continue
        ln = ax.axvline(t, color=EREIGNIS_FARBEN.get(name, "gray"),
                        ls=":", lw=1.6, alpha=0.9, label=name)
        ln.set_gid("_marker")  # von der y-Autoskalierung ausschließen


def _markiere_ort(ax, ulog, ereignisse, fenster, dim3=False):
    """Markiert die Arming-/Landungs-Position in einem räumlichen Plot.

    Args:
        ax: matplotlib-Achse (2D oder 3D).
        ulog: ULog-Objekt.
        ereignisse: Dict {Name: Zeit}.
        fenster: (t0, t1) zur Sichtbarkeitsprüfung.
        dim3: True für 3D-Achsen.
    """
    vlp = hole(ulog, "vehicle_local_position")
    t = zeit(ulog, vlp)
    for name, te in ereignisse.items():
        if te is None or not (fenster[0] <= te <= fenster[1]):
            continue
        i = int(np.argmin(np.abs(t - te)))
        x, y, z = vlp.data["x"][i], vlp.data["y"][i], vlp.data["z"][i]
        c = EREIGNIS_FARBEN.get(name, "gray")
        if dim3:
            ax.scatter([x], [y], [-z], c=c, s=70, depthshade=False, label=name)
        else:
            ax.plot([y], [x], marker="o", color=c, ms=10, ls="", label=name)


def _legende(ax, loc=LEGENDE_LOC):
    """Setzt eine einheitliche Legende, falls beschriftete Elemente vorhanden sind.

    Args:
        ax: matplotlib-Achse.
        loc: Legendenposition.
    """
    h, l = ax.get_legend_handles_labels()
    if h:
        ax.legend(h, l, loc=loc, fontsize=sg("legende"), framealpha=0.9,
                  ncol=1 if len(h) <= 4 else 2)


def _autoscale_y(ax, fenster):
    """Skaliert die y-Achse auf die im Zeitfenster sichtbaren Datenlinien.

    Marker-Linien (gid '_marker') werden ignoriert.

    Args:
        ax: matplotlib-Achse.
        fenster: (t0, t1).
    """
    ymin, ymax = np.inf, -np.inf
    for ln in ax.get_lines():
        if ln.get_gid() == "_marker":
            continue
        x = np.asarray(ln.get_xdata(), dtype=float)
        y = np.asarray(ln.get_ydata(), dtype=float)
        if x.size == 0:
            continue
        m = (x >= fenster[0]) & (x <= fenster[1]) & np.isfinite(y)
        if m.any():
            ymin = min(ymin, float(np.min(y[m])))
            ymax = max(ymax, float(np.max(y[m])))
    if np.isfinite(ymin) and np.isfinite(ymax) and ymax > ymin:
        pad = 0.06 * (ymax - ymin)
        ax.set_ylim(ymin - pad, ymax + pad)


def _finalisiere_zeit(ax, fenster, ereignisse, autoscale_y=True):
    """Vereinheitlicht eine Zeitachsen-Achse: Aussetzer, Marker, x-Limits, Legende.

    Die Vision-Aussetzer werden aus der zentral hinterlegten Liste
    (setze_aussetzer) uebernommen, sofern die Achse nicht bereits von der
    Plotfunktion selbst schattiert wurde.

    Args:
        ax: matplotlib-Achse.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        autoscale_y: Ob die y-Achse auf das Fenster skaliert werden soll.
    """
    if (AUSSETZER_UEBERALL and _AUSSETZER_AKTUELL
            and not getattr(ax, "_aussetzer_schattiert", False)):
        sichtbar = [a for a in _AUSSETZER_AKTUELL
                    if a[1] >= fenster[0] and a[0] <= fenster[1]]
        _schattiere_aussetzer(ax, sichtbar)
    _markiere_zeit(ax, ereignisse, fenster)
    ax.set_xlim(*fenster)
    if autoscale_y:
        _autoscale_y(ax, fenster)
    ax.set_xlabel("Zeit [s]")                 # Zeitachse unter jedem Diagramm
    ax.tick_params(labelbottom=True)
    ax.grid(True, alpha=0.3)
    _legende(ax)


def _schattiere_aussetzer(ax, aussetzer, label=True):
    """Schattiert Vision-Aussetzer als orange Bereiche.

    Die Achse wird markiert, damit _finalisiere_zeit sie nicht ein zweites
    Mal hinterlegt.

    Args:
        ax: matplotlib-Achse.
        aussetzer: Liste von (t_start, t_ende, dauer).
        label: Ob ein Legendeneintrag gesetzt wird.
    """
    for i, (ta, te, _) in enumerate(aussetzer):
        ax.axvspan(ta, te, color=AUSSETZER_FARBE, alpha=AUSSETZER_ALPHA,
                   label=AUSSETZER_LABEL if (label and i == 0) else None)
    ax._aussetzer_schattiert = True


# ---------------------------------------------------------------------------
# Plot-Funktionen  (einheitliche Signatur: ulog, inst, fenster, ereignisse, schwelle)
# ---------------------------------------------------------------------------
# Belegung der Stuetzquellen
# ---------------------------------------------------------------------------
STUETZQUELLEN = [
    ("cs_ev_pos", "Vision Position"),
    ("cs_ev_hgt", "Vision Höhe"),
    ("cs_ev_yaw", "Vision Gierwinkel"),
    ("cs_baro_hgt", "Barometer Höhe"),
    ("cs_inertial_dead_reckoning", "Dead Reckoning"),
]


def plot_stuetzquellen(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die zeitliche Belegung der Stuetzquellen des EKF als Balken.

    Anders als uebereinandergelegte Nulleinslinien bleibt die Darstellung auch
    dann lesbar, wenn mehrere Quellen gleichzeitig aktiv sind. Ergaenzt ist die
    Koppelnavigation, die anzeigt, wann der EKF ohne Stuetzmessung rechnet.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle, hier ungenutzt.

    Returns:
        matplotlib-Figure.
    """
    sf = hole(ulog, "estimator_status_flags", inst)
    if sf is None:
        raise ValueError("estimator_status_flags nicht im Log vorhanden")
    t = zeit(ulog, sf)
    vorhanden = [(f, n) for f, n in STUETZQUELLEN if f in sf.data]

    fig, ax = plt.subplots(figsize=(13, 0.75 * len(vorhanden) + 2.2),
                           constrained_layout=True)
    for reihe, (feld, name) in enumerate(vorhanden):
        v = sf.data[feld].astype(int)
        anteil = flag_anteil(ulog, inst, feld, fenster)
        farbe = "C3" if feld == "cs_inertial_dead_reckoning" else "C0"
        wechsel = np.concatenate(([0], np.where(np.diff(v) != 0)[0] + 1, [len(v) - 1]))
        for k in range(len(wechsel) - 1):
            i, j = wechsel[k], wechsel[k + 1]
            if v[i] == 1:
                ax.barh(reihe, t[j] - t[i], left=t[i], height=0.6,
                        color=farbe, edgecolor="none")
        ax.text(1.005, reihe, f"{anteil:.1f} %", transform=ax.get_yaxis_transform(),
                va="center", fontsize=sg("annotation"))

    ax.set_yticks(range(len(vorhanden)))
    ax.set_yticklabels([n for _, n in vorhanden])
    ax.set_ylim(-0.7, len(vorhanden) - 0.3)
    ax.invert_yaxis()
    ax.set_ylabel("")
    _finalisiere_zeit(ax, fenster, ereignisse, autoscale_y=False)
    ax.set_title("Belegung der Bezugsqellen des EKFs ",
                 fontsize=sg("subplot_titel"), fontweight="bold")
    return fig


# ---------------------------------------------------------------------------
# LaTeX-Export der Kennzahlen
# ---------------------------------------------------------------------------
def kennzahlen(ulog, inst, fenster):
    """Ermittelt die Kennzahlen der Vision-Fusion fuer einen Zeitraum.

    Args:
        ulog: ULog-Objekt.
        inst: Primaere EKF-Instanz-ID.
        fenster: (t0, t1) ausgewerteter Zeitraum.

    Returns:
        Dict mit den Kennzahlen; fehlende Groessen sind NaN.
    """
    nan = float("nan")
    k = {"ev_pos": nan, "ev_yaw": nan, "dead_reckoning": nan,
         "aussetzer_n": nan, "aussetzer_max": nan,
         "innov_rms_n": nan, "innov_rms_e": nan, "tr_max": nan,
         "sigma_pos": nan, "rms_x": nan, "rms_y": nan, "rms_z": nan}

    k["ev_pos"] = flag_anteil(ulog, inst, "cs_ev_pos", fenster)
    k["ev_yaw"] = flag_anteil(ulog, inst, "cs_ev_yaw", fenster)
    k["dead_reckoning"] = flag_anteil(ulog, inst, "cs_inertial_dead_reckoning", fenster)

    aus = finde_ev_aussetzer_flags(ulog, inst, fenster)
    k["aussetzer_n"] = len(aus)
    k["aussetzer_max"] = max((a[2] for a in aus), default=0.0)

    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    if ev is not None:
        m = maske(zeit(ulog, ev), fenster) & ev.data["fused"].astype(bool)
        if m.any():
            k["innov_rms_n"] = float(np.sqrt(np.nanmean(ev.data["innovation[0]"][m] ** 2)))
            k["innov_rms_e"] = float(np.sqrt(np.nanmean(ev.data["innovation[1]"][m] ** 2)))
            tr = np.maximum(ev.data["test_ratio[0]"][m], ev.data["test_ratio[1]"][m])
            k["tr_max"] = float(np.nanmax(tr))

    st = hole(ulog, "estimator_states", inst)
    if st is not None:
        m = maske(zeit(ulog, st), fenster)
        if m.any():
            idx = kovarianz_indizes(st)
            k["sigma_pos"] = float(np.nanmean(
                np.sqrt(np.abs(st.data[f"covariances[{idx['pos']}]"]))[m]))

    vlp = hole(ulog, "vehicle_local_position")
    ts = hole(ulog, "trajectory_setpoint")
    if vlp is not None and ts is not None:
        t = zeit(ulog, vlp)
        m = maske(t, fenster)
        t_sp = zeit(ulog, ts)
        for name, achse, key in [("rms_x", "x", "position[0]"),
                                 ("rms_y", "y", "position[1]"),
                                 ("rms_z", "z", "position[2]")]:
            sp = np.interp(t[m], t_sp, ts.data[key])
            est = vlp.data[achse][m]
            ok = np.isfinite(sp) & np.isfinite(est)
            if ok.any():
                k[name] = float(np.sqrt(np.nanmean((sp[ok] - est[ok]) ** 2)))
    return k


def parse_phasen(text):
    """Zerlegt die Phasenangabe der Kommandozeile.

    Args:
        text: String der Form "Name:t0:t1,Name:t0:t1" oder None.

    Returns:
        Liste von (name, t0, t1); leer, falls text None ist.
    """
    if not text:
        return []
    out = []
    for teil in text.split(","):
        name, t0, t1 = teil.rsplit(":", 2)
        out.append((name.strip(), float(t0), float(t1)))
    return out


def kennzahlen_latex(ulog, inst, phasen):
    """Erzeugt eine LaTeX-Tabelle der Fusionskennzahlen je Flugphase.

    Args:
        ulog: ULog-Objekt.
        inst: Primaere EKF-Instanz-ID.
        phasen: Liste von (name, t0, t1).

    Returns:
        String mit dem LaTeX-Quelltext der Tabelle.
    """
    def z(v, n=2):
        """Formatiert eine Zahl mit Dezimalkomma fuer LaTeX."""
        if v != v:
            return "--"
        return "$" + f"{v:.{n}f}".replace(".", "{,}") + "$"

    werte = [(name, kennzahlen(ulog, inst, (t0, t1))) for name, t0, t1 in phasen]
    spalten = "l" + "c" * len(werte)
    kopf = " & ".join("\\textbf{" + n + "}" for n, _ in werte)
    L = ["\\begin{table}[htbp]",
         "\\centering",
         "\\begin{tabular}{" + spalten + "}",
         "\\toprule",
         "\\textbf{Kennzahl} & " + kopf + " \\\\",
         "\\midrule"]

    zeilen = [
        ("Vision-Positionsstützung aktiv in \\%", "ev_pos", 1),
        ("Vision-Gierstützung aktiv in \\%", "ev_yaw", 1),
        ("Dead Reckoning in \\%", "dead_reckoning", 1),
        ("Unterbrechungen der Stützung", "aussetzer_n", 0),
        ("längste Unterbrechung in s", "aussetzer_max", 2),
        ("Innovation Nord (quadr. Mittel) in m", "innov_rms_n", 3),
        ("Innovation Ost (quadr. Mittel) in m", "innov_rms_e", 3),
        ("größtes Test-Verhältnis", "tr_max", 3),
        ("$\\sigma$ Position Nord in m", "sigma_pos", 3),
        ("Regelabweichung $x$ (quadr. Mittel) in m", "rms_x", 3),
        ("Regelabweichung $y$ (quadr. Mittel) in m", "rms_y", 3),
        ("Regelabweichung $z$ (quadr. Mittel) in m", "rms_z", 3),
    ]
    for text, key, n in zeilen:
        L.append(text + " & "
                 + " & ".join(z(k[key], n) for _, k in werte) + " \\\\")

    L += ["\\bottomrule",
          "\\end{tabular}",
          "\\caption{Kennzahlen der Vision-Fusion je Flugphase}",
          "\\label{tab:ekf_fusion_kennzahlen}",
          "\\end{table}"]
    return "\n".join(L)


# ---------------------------------------------------------------------------
def plot_position_vision_ekf_sollwert(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet Position (N/E/D): Vision-Beobachtung vs. EKF vs. Sollwert.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt) Aussetzer-Schwelle.

    Returns:
        matplotlib-Figure.
    """
    vlp = hole(ulog, "vehicle_local_position")
    ts = hole(ulog, "trajectory_setpoint")
    ev_pos = hole(ulog, "estimator_aid_src_ev_pos", inst)
    ev_hgt = hole(ulog, "estimator_aid_src_ev_hgt", inst)

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)
    labels = ["x [m]", "y [m]", "z [m] (NED)"]
    t_est = zeit(ulog, vlp)
    est = [vlp.data["x"], vlp.data["y"], vlp.data["z"]]
    t_sp = zeit(ulog, ts) if ts is not None else None

    for i, ax in enumerate(axes):
        ax.plot(t_est, est[i], color="C0", lw=1.4, label="EKF-Schätzung")
        if ts is not None:
            ax.plot(t_sp, ts.data[f"position[{i}]"], color="C3", lw=1.0,
                    ls="--", label="Sollwert")
        if i < 2 and ev_pos is not None:
            ax.plot(zeit(ulog, ev_pos), ev_pos.data[f"observation[{i}]"],
                    color="C2", lw=0.0, marker=".", ms=2.5, alpha=0.6,
                    label="Vision-Beobachtung")
        if i == 2 and ev_hgt is not None:
            ax.plot(zeit(ulog, ev_hgt), ev_hgt.data["observation"],
                    color="C2", lw=0.0, marker=".", ms=2.5, alpha=0.6,
                    label="Vision-Beobachtung")
        ax.set_ylabel(labels[i])
        _finalisiere_zeit(ax, fenster, ereignisse)
    fig.suptitle("Position: Vision-Vorgabe vs. EKF-Schätzung vs. Sollwert",
                 fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_bahn_2d_3d(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die geflogene Bahn: Draufsicht (XY), Höhenverlauf und 3D.

    Layout: oben links XY, oben rechts Höhe(t), unten 3D über volle Breite
    (verhindert die Achsenbeschriftungs-Überschneidungen des alten Layouts).

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    vlp = hole(ulog, "vehicle_local_position")
    ts = hole(ulog, "trajectory_setpoint")
    t = zeit(ulog, vlp)
    m = maske(t, fenster)
    x, y, z = vlp.data["x"][m], vlp.data["y"][m], vlp.data["z"][m]

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.15],
                          hspace=0.30, wspace=0.22,
                          left=0.08, right=0.95, top=0.92, bottom=0.08)

    # Draufsicht XY
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(y, x, color="C0", lw=1.2, label="EKF-Bahn")
    if ts is not None:
        ms = maske(zeit(ulog, ts), fenster)
        ax1.plot(ts.data["position[1]"][ms], ts.data["position[0]"][ms],
                 color="C3", lw=0.9, ls="--", label="Sollwert")
    _markiere_ort(ax1, ulog, ereignisse, fenster)
    ax1.set_xlabel("Ost  y [m]")
    ax1.set_ylabel("Nord  x [m]")
    ax1.set_title("Draufsicht (XY)")
    ax1.axis("equal")
    ax1.grid(True, alpha=0.3)
    _legende(ax1, "best")

    # Höhenverlauf (Zeitachse)
    ax3 = fig.add_subplot(gs[0, 1])
    ax3.plot(t, -vlp.data["z"], color="C0", lw=1.3, label="EKF-Höhe")
    if ts is not None:
        ax3.plot(zeit(ulog, ts), -ts.data["position[2]"], color="C3",
                 lw=0.9, ls="--", label="Soll-Höhe")
    ax3.set_ylabel("Höhe -z [m]")
    ax3.set_title("Höhenverlauf")
    _finalisiere_zeit(ax3, fenster, ereignisse)

    # 3D über volle Breite
    ax2 = fig.add_subplot(gs[1, :], projection="3d")
    ax2.plot(x, y, -z, color="C0", lw=1.0, label="EKF-Bahn")
    _markiere_ort(ax2, ulog, ereignisse, fenster, dim3=True)
    ax2.set_xlabel("x [m]", labelpad=10)
    ax2.set_ylabel("y [m]", labelpad=10)
    ax2.zaxis.set_rotate_label(False)
    ax2.set_zlabel("z [m]", labelpad=10, rotation=90)
    ax2.tick_params(labelsize=8, pad=2)
    ax2.set_title("Flugbahn")
    _legende(ax2, "upper left")

    fig.suptitle("Geflogene Bahn", fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_geschwindigkeit(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die Geschwindigkeit (vx, vy, vz): EKF vs. Sollwert.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    vlp = hole(ulog, "vehicle_local_position")
    ts = hole(ulog, "trajectory_setpoint")
    t = zeit(ulog, vlp)
    v = [vlp.data["vx"], vlp.data["vy"], vlp.data["vz"]]
    labels = ["vx (Nord) [m/s]", "vy (Ost) [m/s]", "vz (Unten, NED) [m/s]"]

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)
    for i, ax in enumerate(axes):
        ax.plot(t, v[i], color="C0", lw=1.2, label="EKF-Schätzung")
        if ts is not None:
            ax.plot(zeit(ulog, ts), ts.data[f"velocity[{i}]"], color="C3",
                    lw=0.9, ls="--", label="Sollwert")
        ax.set_ylabel(labels[i])
        _finalisiere_zeit(ax, fenster, ereignisse)
    fig.suptitle("Geschwindigkeit: EKF vs. Sollwert", fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_lage(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die Lage (Roll, Pitch, Yaw): EKF vs. Sollwert.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    att = hole(ulog, "vehicle_attitude")
    asp = hole(ulog, "vehicle_attitude_setpoint")
    t = zeit(ulog, att)
    r, p, yw = quat_zu_euler(att.data["q[0]"], att.data["q[1]"],
                             att.data["q[2]"], att.data["q[3]"])
    est = [np.degrees(r), np.degrees(p), np.degrees(yw)]
    sp = None
    if asp is not None:
        rs, ps, yws = quat_zu_euler(asp.data["q_d[0]"], asp.data["q_d[1]"],
                                    asp.data["q_d[2]"], asp.data["q_d[3]"])
        sp = [np.degrees(rs), np.degrees(ps), np.degrees(yws)]
        t_sp = zeit(ulog, asp)
    labels = ["Roll [°]", "Pitch [°]", "Yaw [°]"]

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)
    for i, ax in enumerate(axes):
        ax.plot(t, est[i], color="C0", lw=1.2, label="EKF-Schätzung")
        if sp is not None:
            ax.plot(t_sp, sp[i], color="C3", lw=0.9, ls="--", label="Sollwert")
        ax.set_ylabel(labels[i])
        _finalisiere_zeit(ax, fenster, ereignisse)
    fig.suptitle("Lage: EKF vs. Sollwert", fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_test_ratios(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die Innovation-Test-Ratios der EKF-Aiding-Quellen.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    tr = hole(ulog, "estimator_innovation_test_ratios", inst)
    t = zeit(ulog, tr)
    fig, ax = plt.subplots(figsize=(13, 6), constrained_layout=True)
    for feld, name in [("ev_hpos[0]", "Vision Pos x"), ("ev_hpos[1]", "Vision Pos y"),
                       ("ev_vpos", "Vision Pos z"), ("baro_vpos", "Baro Höhe"),
                       ("heading", "Heading")]:
        if feld in tr.data:
            ax.plot(t, tr.data[feld], lw=1.0, label=name)
    ax.axhline(1.0, color="k", ls="--", lw=1.2, label="Verwerfungsschwelle 1.0")
    ax.set_ylabel("Test-Ratio [-]")
    ax.set_title("Innovation-Test-Ratios (Messung wird ab 1.0 verworfen)",
                 fontweight="bold")
    _finalisiere_zeit(ax, fenster, ereignisse)
    return fig


def plot_ev_innovationen(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet Vision-Innovationen mit ±1σ-Grenzen.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    ev_pos = hole(ulog, "estimator_aid_src_ev_pos", inst)
    ev_hgt = hole(ulog, "estimator_aid_src_ev_hgt", inst)
    ev_yaw = hole(ulog, "estimator_aid_src_ev_yaw", inst)
    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True,
                             constrained_layout=True)

    def _innov(ax, t, innov, var, titel):
        """Zeichnet eine Innovation mit ±1σ-Hüllkurve und skaliert manuell."""
        sig = np.sqrt(np.abs(var))
        ax.fill_between(t, -sig, sig, color="C0", alpha=0.2,
                        label="±1σ (Innovationsvarianz S)")
        ax.plot(t, innov, color="C1", lw=0.8, label="Innovation")
        ax.set_ylabel(titel)
        m = maske(t, fenster)
        if m.any():
            lim = 1.2 * np.nanmax(np.abs(np.concatenate([sig[m], innov[m]])))
            if np.isfinite(lim) and lim > 0:
                ax.set_ylim(-lim, lim)
        _finalisiere_zeit(ax, fenster, ereignisse, autoscale_y=False)

    if ev_pos is not None:
        t = zeit(ulog, ev_pos)
        _innov(axes[0], t, ev_pos.data["innovation[0]"],
               ev_pos.data["innovation_variance[0]"], "Vision Pos x [m]")
        _innov(axes[1], t, ev_pos.data["innovation[1]"],
               ev_pos.data["innovation_variance[1]"], "Vision Pos y [m]")
    if ev_hgt is not None:
        t = zeit(ulog, ev_hgt)
        _innov(axes[2], t, ev_hgt.data["innovation"],
               ev_hgt.data["innovation_variance"], "Vision Höhe [m]")
    if ev_yaw is not None:
        t = zeit(ulog, ev_yaw)
        _innov(axes[3], t, ev_yaw.data["innovation"],
               ev_yaw.data["innovation_variance"], "Vision Gierwinkel [rad]")
    fig.suptitle("External-Vision-Innovationen mit Konsistenzgrenzen",
                 fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_biases(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die geschätzten Gyro- und Accel-Biases.

    Die Zustands-Indizes (states[10-12] Gyro, states[13-15] Accel) gelten
    für den 25-elementigen Zustandsvektor von PX4 v1.16 und wurden gegen
    das Topic estimator_sensor_bias verifiziert.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    st = hole(ulog, "estimator_states", inst)
    t = zeit(ulog, st)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9), sharex=True,
                                   constrained_layout=True)
    for i, a in enumerate(["x", "y", "z"]):
        ax1.plot(t, np.degrees(st.data[f"states[{10 + i}]"]), lw=1.0,
                 label=f"Gyro-Bias {a}")
        ax2.plot(t, st.data[f"states[{13 + i}]"], lw=1.0, label=f"Accel-Bias {a}")
    ax1.set_ylabel("Gyro-Bias [°/s]")
    ax2.set_ylabel("Accel-Bias [m/s²]")
    _finalisiere_zeit(ax1, fenster, ereignisse)
    _finalisiere_zeit(ax2, fenster, ereignisse)
    fig.suptitle("EKF Sensor-Bias-Schätzungen", fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_fusion_status(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet Vision-Fusionsstatus, Beobachtungsrauschen und aktive Aiding-Quellen.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle (für Schattierung).

    Returns:
        matplotlib-Figure.
    """
    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    flags = hole(ulog, "estimator_status_flags", inst)
    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle, fenster)
    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)
    t = zeit(ulog, ev)

    axes[0].plot(t, ev.data["fused"], color="C2", lw=0.8, label="fused")
    axes[0].plot(t, ev.data["innovation_rejected"], color="C3", lw=0.8,
                 label="rejected")
    axes[0].set_ylabel("Status [0/1]")
    axes[0].set_ylim(-0.1, 1.1)
    _finalisiere_zeit(axes[0], fenster, ereignisse, autoscale_y=False)

    axes[1].plot(t, np.sqrt(ev.data["observation_variance[0]"]), lw=0.9,
                 label="σ Beobachtung x (R)")
    axes[1].plot(t, np.sqrt(ev.data["observation_variance[1]"]), lw=0.9,
                 label="σ Beobachtung y (R)")
    _schattiere_aussetzer(axes[1], aussetzer)
    axes[1].set_ylabel("Vision σ Beobachtung [m]")
    _finalisiere_zeit(axes[1], fenster, ereignisse)

    if flags is not None:
        tf = zeit(ulog, flags)
        for feld, name in [("cs_ev_pos", "EV Pos"), ("cs_ev_yaw", "EV Yaw"),
                           ("cs_ev_hgt", "EV Höhe"), ("cs_baro_hgt", "Baro Höhe"),
                           ("cs_in_air", "in der Luft")]:
            if feld in flags.data:
                axes[2].plot(tf, flags.data[feld], lw=0.9, label=name)
        axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_ylabel("aktiv [0/1]")
    _finalisiere_zeit(axes[2], fenster, ereignisse, autoscale_y=False)
    fig.suptitle("EKF Fusions-Status (External Vision)", fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_flugmodi(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet Nav-State, Arming-State und Land-Detector.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    vs = hole(ulog, "vehicle_status")
    ld = hole(ulog, "vehicle_land_detected")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8), sharex=True,
                                   constrained_layout=True)
    t = zeit(ulog, vs)
    ax1.step(t, vs.data["nav_state"], where="post", color="C0", label="Flugmodus")
    werte = sorted(set(vs.data["nav_state"]))
    ax1.set_yticks(werte)
    ax1.set_yticklabels([NAV_STATE_NAMES.get(int(v), str(v)) for v in werte])
    ax1.set_ylabel("Flugmodus")
    _finalisiere_zeit(ax1, fenster, ereignisse, autoscale_y=False)

    ax2.step(t, vs.data["arming_state"], where="post", color="C1",
             label="Arming-State (1=disarmed, 2=armed)")
    if ld is not None:
        ax2.step(zeit(ulog, ld), ld.data["landed"], where="post",
                 color="C2", label="landed (Land-Detector)")
    ax2.set_yticks([0, 1, 2])
    ax2.set_ylabel("Arming / landed")
    _finalisiere_zeit(ax2, fenster, ereignisse, autoscale_y=False)
    fig.suptitle("Flugmodi und Arming", fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_aussetzer_resets(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet Höhe, Vision-Updates, Vision-Aussetzer und EKF-Resets.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle in Sekunden.

    Returns:
        matplotlib-Figure.
    """
    vlp = hole(ulog, "vehicle_local_position")
    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    tv = zeit(ulog, vlp)
    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle, fenster)
    resets = finde_resets(ulog, inst, fenster)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9), constrained_layout=True)

    def _zeichne(ax, tmin, tmax, titel):
        """Zeichnet Höhe, Vision-Updates, Aussetzer und Resets in ein Zeitfenster."""
        ax.plot(tv, -vlp.data["z"], "C0-", lw=1.5, label="Höhe -z [m]")
        if ev is not None:
            te = zeit(ulog, ev)
            tf = te[ev.data["fused"].astype(bool)]
            ax.plot(tf, np.full(len(tf), -0.15), "g|", ms=10, label="Vision fusioniert")
        _schattiere_aussetzer(ax, [a for a in aussetzer
                                   if a[1] >= tmin and a[0] <= tmax])
        erst = True
        for r in resets.get("Pos NE", []):
            if tmin <= r <= tmax:
                ax.axvline(r, color="magenta", lw=1.3,
                           label="Pos-Reset (NE)" if erst else None).set_gid("_marker")
                erst = False
        ax.set_ylabel("Höhe -z [m]")
        ax.set_title(titel)
        _finalisiere_zeit(ax, (tmin, tmax), ereignisse)

    _zeichne(ax1, fenster[0], fenster[1], "Übersicht")
    rne = resets.get("Pos NE", np.array([]))
    if len(rne) > 1:
        zmin = max(fenster[0], rne.min() - 5)
        zmax = min(fenster[1], rne.max() + 5)
    else:
        zmin, zmax = fenster
    _zeichne(ax2, zmin, zmax, f"Zoom Reset-Bereich ({zmin:.0f}-{zmax:.0f} s)")
    fig.suptitle("Vision-Aussetzer und EKF-Resets", fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_kovarianz(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die EKF-Unsicherheit (1σ) aus der Kovarianz-Diagonale.

    Die Blockindizes werden über kovarianz_indizes() versionsabhängig
    bestimmt (v1.16: Fehlerzustands-Kovarianz, Position bei 6-8).

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle (für Schattierung).

    Returns:
        matplotlib-Figure.
    """
    st = hole(ulog, "estimator_states", inst)
    vlp = hole(ulog, "vehicle_local_position")
    t = zeit(ulog, st)
    idx = kovarianz_indizes(st)
    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle, fenster)

    def sig(i):
        """1σ aus Kovarianz-Index i."""
        return np.sqrt(np.abs(st.data[f"covariances[{i}]"]))

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)

    p0 = idx["pos"]
    for i, lab in zip([p0, p0 + 1, p0 + 2], ["Nord", "Ost", "Unten"]):
        axes[0].plot(t, sig(i), lw=1.1, label=f"σ Pos {lab}")
    if vlp is not None:
        tv = zeit(ulog, vlp)
        axes[0].plot(tv, vlp.data["eph"], "k:", lw=1.0, alpha=0.7,
                     label="eph (horizontal, EKF)")
        axes[0].plot(tv, vlp.data["epv"], "k--", lw=1.0, alpha=0.7,
                     label="epv (vertikal, EKF)")
    _schattiere_aussetzer(axes[0], aussetzer)
    axes[0].set_ylabel("Position 1σ [m]")
    # robuste Obergrenze (Ausreißer nach dem Aufsetzen ausblenden), fensterbezogen
    m = maske(t, fenster)
    werte = np.concatenate([sig(p0)[m], sig(p0 + 1)[m], sig(p0 + 2)[m]]) \
        if m.any() else sig(p0)
    og = np.nanpercentile(werte, 99) * 1.4
    if np.isfinite(og) and og > 0:
        axes[0].set_ylim(0, max(og, 0.1))
    _finalisiere_zeit(axes[0], fenster, ereignisse, autoscale_y=False)

    v0 = idx["vel"]
    for i, lab in zip([v0, v0 + 1, v0 + 2], ["Nord", "Ost", "Unten"]):
        axes[1].plot(t, sig(i), lw=1.1, label=f"σ Vel {lab}")
    _schattiere_aussetzer(axes[1], aussetzer, label=False)
    axes[1].set_ylabel("Geschw. 1σ [m/s]")
    _finalisiere_zeit(axes[1], fenster, ereignisse)

    g0, a0 = idx["gyro_bias"], idx["accel_bias"]
    for i, lab in zip([g0, g0 + 1, g0 + 2], ["x", "y", "z"]):
        axes[2].plot(t, np.degrees(sig(i)), lw=1.0, label=f"Gyro {lab} [°/s]")
    for i, lab in zip([a0, a0 + 1, a0 + 2], ["x", "y", "z"]):
        axes[2].plot(t, sig(i), lw=1.0, ls="--", label=f"Accel {lab} [m/s²]")
    axes[2].set_ylabel("Bias 1σ")
    _finalisiere_zeit(axes[2], fenster, ereignisse)
    fig.suptitle("EKF-Unsicherheit (Kovarianz-Diagonale, 1σ)",
                 fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_vertrauen(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet das skalare Kalman-Gain der Vision-Messungen.

    K = 1 - R/S mit R = observation_variance (Messrauschen) und
    S = H·P·Hᵀ + R = innovation_variance. Fuer die direkte Positionsmessung
    entspricht das K = P/(P + R).

    K ist ein Verhaeltnis und kein Guetemass. Ein hoher Wert bedeutet nicht,
    dass die Vision gut ist, sondern nur, dass die Praediktionsunsicherheit P
    gegenueber der Messunsicherheit R gross ist. Waehrend eines Aussetzers
    waechst P, weil der EKF ohne Stuetzung koppelnavigiert, deshalb steigt K
    dort an. Ein kleiner Wert bedeutet umgekehrt nicht, dass die Vision
    ignoriert wird, denn K gilt pro Update und die Messungen wirken ueber
    viele Updates hinweg.

    Die absolute Aussage zur Filterguete liefert der untere Teilplot: der
    Abstand zwischen sigma(S) und sigma(R) entspricht der eigenen
    Positionsunsicherheit sigma(P) = sqrt(S - R).

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle (für Schattierung).

    Returns:
        matplotlib-Figure.
    """
    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    evh = hole(ulog, "estimator_aid_src_ev_hgt", inst)
    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle, fenster)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9), sharex=True,
                                   constrained_layout=True)
    t = zeit(ulog, ev)
    for k, lab in [(0, "Vision Pos x"), (1, "Vision Pos y")]:
        K = np.clip(1.0 - ev.data[f"observation_variance[{k}]"]
                    / ev.data[f"innovation_variance[{k}]"], 0.0, 1.0)
        ax1.plot(t, K, lw=1.0, label=lab)
    if evh is not None:
        Kz = np.clip(1.0 - evh.data["observation_variance"]
                     / evh.data["innovation_variance"], 0.0, 1.0)
        ax1.plot(zeit(ulog, evh), Kz, lw=1.0, label="Vision Höhe")
    _schattiere_aussetzer(ax1, aussetzer)
    ax1.set_ylabel("Kalman-Gain K")
    ax1.set_ylim(-0.05, 1.05)
    ax1.set_title("",
                  fontsize=sg("subplot_titel"))
    _finalisiere_zeit(ax1, fenster, ereignisse, autoscale_y=False)

    ax2.plot(t, np.sqrt(ev.data["observation_variance[0]"]), lw=1.1,
             label="σ Messung (R)")
    ax2.plot(t, np.sqrt(ev.data["innovation_variance[0]"]), lw=1.1,
             label="σ Innovation (S = H·P·Hᵀ + R)")
    _schattiere_aussetzer(ax2, aussetzer, label=False)
    ax2.set_ylabel("1σ [m] (x-Kanal)")
    ax2.set_title("Absolute Unsicherheiten: der Abstand der Kurven entspricht der "
                  "EKF-Positionsunsicherheit σ(P)",
                  fontsize=sg("subplot_titel"))
    _finalisiere_zeit(ax2, fenster, ereignisse)
    fig.suptitle("Kalman-Gain der Vision-Positionsstützung",
                 fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_yaw_detail(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die Yaw-Fusion im Detail: EKF-Yaw vs. Vision-Gierwinkel-Beobachtung.

    Oben: Yaw-Schätzung des EKF und Vision-Gierwinkel-Beobachtung in Grad.
    Mitte: Yaw-Differenz (EKF minus Vision, gewickelt auf ±180°).
    Unten: Heading-Test-Ratio und Vision-Gierwinkel-Test-Ratio mit Verwerfungsschwelle.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    att = hole(ulog, "vehicle_attitude")
    ev_yaw = hole(ulog, "estimator_aid_src_ev_yaw", inst)
    tr = hole(ulog, "estimator_innovation_test_ratios", inst)

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)

    t_att = zeit(ulog, att)
    _, _, yw = quat_zu_euler(att.data["q[0]"], att.data["q[1]"],
                             att.data["q[2]"], att.data["q[3]"])
    yaw_ekf = np.degrees(yw)
    axes[0].plot(t_att, yaw_ekf, color="C0", lw=1.2, label="EKF-Yaw")
    if ev_yaw is not None:
        t_ev = zeit(ulog, ev_yaw)
        yaw_ev = np.degrees(ev_yaw.data["observation"])
        axes[0].plot(t_ev, yaw_ev, color="C2", lw=0.0, marker=".", ms=3.5,
                     alpha=0.7, label="Vision-Gierwinkel-Beobachtung")
    axes[0].set_ylabel("Yaw [°]")
    _finalisiere_zeit(axes[0], fenster, ereignisse)

    if ev_yaw is not None:
        yaw_ekf_i = np.interp(t_ev, t_att, np.unwrap(np.radians(yaw_ekf)))
        diff = wickle_grad(np.degrees(yaw_ekf_i) - yaw_ev)
        axes[1].plot(t_ev, diff, color="C3", lw=1.0, marker=".", ms=3,
                     label="Yaw-Differenz (EKF − Vision)")
        axes[1].axhline(0.0, color="k", lw=0.8, alpha=0.5)
    axes[1].set_ylabel("Differenz [°]")
    _finalisiere_zeit(axes[1], fenster, ereignisse)

    if tr is not None and "heading" in tr.data:
        axes[2].plot(zeit(ulog, tr), tr.data["heading"], lw=1.0, color="C0",
                     label="Test-Ratio Heading")
    if ev_yaw is not None:
        axes[2].plot(zeit(ulog, ev_yaw), ev_yaw.data["test_ratio"], lw=1.0,
                     color="C1", label="Test-Ratio Vision-Gierwinkel")
    axes[2].axhline(1.0, color="k", ls="--", lw=1.2,
                    label="Verwerfungsschwelle 1.0")
    axes[2].set_ylabel("Test-Ratio [-]")
    _finalisiere_zeit(axes[2], fenster, ereignisse)
    fig.suptitle("Yaw-Fusion: EKF vs. External Vision",
                 fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_hoehenquellen(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet den Vergleich der Höhenquellen: Vision, Baro und EKF.

    Oben: EKF-Höhe (-z), Vision-Höhen-Beobachtung (-observation) und Baro-Höhe.
    Die Baro-Höhe (AMSL) wird versatzbereinigt dargestellt, indem ihr
    Startwert im Fenster auf die EKF-Starthöhe gelegt wird; Drift bleibt
    dadurch sichtbar.
    Mitte: Differenzen EKF minus Vision und EKF minus Baro.
    Unten: aktive Höhenquellen-Flags des EKF.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle (für Schattierung).

    Returns:
        matplotlib-Figure.
    """
    vlp = hole(ulog, "vehicle_local_position")
    ev_hgt = hole(ulog, "estimator_aid_src_ev_hgt", inst)
    ad = hole(ulog, "vehicle_air_data")
    flags = hole(ulog, "estimator_status_flags", inst)
    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle, fenster)

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)

    t_ekf = zeit(ulog, vlp)
    h_ekf = -vlp.data["z"]
    axes[0].plot(t_ekf, h_ekf, color="C0", lw=1.3, label="EKF-Höhe (-z)")

    t_ev = h_ev = None
    if ev_hgt is not None:
        t_ev = zeit(ulog, ev_hgt)
        h_ev = -ev_hgt.data["observation"]
        axes[0].plot(t_ev, h_ev, color="C2", lw=0.0, marker=".", ms=3,
                     alpha=0.7, label="Vision-Höhen-Beobachtung")

    t_baro = h_baro = None
    if ad is not None:
        t_baro = zeit(ulog, ad)
        mb = maske(t_baro, fenster)
        me = maske(t_ekf, fenster)
        if mb.any() and me.any():
            versatz = ad.data["baro_alt_meter"][mb][0] - h_ekf[me][0]
        else:
            versatz = ad.data["baro_alt_meter"][0] - h_ekf[0]
        h_baro = ad.data["baro_alt_meter"] - versatz
        axes[0].plot(t_baro, h_baro, color="C1", lw=1.0, alpha=0.8,
                     label="Baro-Höhe (versatzbereinigt)")
    _schattiere_aussetzer(axes[0], aussetzer)
    axes[0].set_ylabel("Höhe [m]")
    _finalisiere_zeit(axes[0], fenster, ereignisse)

    if h_ev is not None:
        d_ev = np.interp(t_ev, t_ekf, h_ekf) - h_ev
        axes[1].plot(t_ev, d_ev, color="C2", lw=1.0, marker=".", ms=3,
                     label="EKF − Vision [m]")
    if h_baro is not None:
        d_baro = np.interp(t_baro, t_ekf, h_ekf) - h_baro
        axes[1].plot(t_baro, d_baro, color="C1", lw=1.0,
                     label="EKF − Baro [m]")
    axes[1].axhline(0.0, color="k", lw=0.8, alpha=0.5)
    axes[1].set_ylabel("Höhen-Differenz [m]")
    _finalisiere_zeit(axes[1], fenster, ereignisse)

    if flags is not None:
        tf = zeit(ulog, flags)
        for feld, name in [("cs_ev_hgt", "EV Höhe aktiv"),
                           ("cs_baro_hgt", "Baro Höhe aktiv"),
                           ("cs_rng_hgt", "Range Höhe aktiv")]:
            if feld in flags.data:
                axes[2].plot(tf, flags.data[feld], lw=0.9, label=name)
        axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_ylabel("aktiv [0/1]")
    _finalisiere_zeit(axes[2], fenster, ereignisse, autoscale_y=False)
    fig.suptitle("Höhenquellen-Vergleich: Vision vs. Baro vs. EKF",
                 fontsize=sg("figur_titel"), fontweight="bold")
    return fig


def plot_ekf_instanzen(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet den Vergleich der EKF-Instanzen und die Selector-Auswahl.

    Oben drei Achsen: Position x, y und Höhe -z aller verfügbaren
    EKF-Instanzen (estimator_local_position). Unten: die vom Selector
    gewählte primäre Instanz über der Zeit.

    Args:
        ulog: ULog-Objekt.
        inst: Primäre EKF-Instanz (wird in der Legende markiert).
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    fig, axes = plt.subplots(4, 1, figsize=(13, 13), sharex=True,
                             constrained_layout=True,
                             gridspec_kw={"height_ratios": [1, 1, 1, 0.6]})

    instanzen = []
    for mid in range(4):
        d = hole(ulog, "estimator_local_position", mid)
        if d is not None:
            instanzen.append((mid, d))

    komponenten = [("x", "Nord x [m]", 1.0),
                   ("y", "Ost y [m]", 1.0),
                   ("z", "Höhe -z [m]", -1.0)]
    for ax, (feld, ylab, vz) in zip(axes[:3], komponenten):
        for mid, d in instanzen:
            zusatz = " (primär)" if mid == inst else ""
            ax.plot(zeit(ulog, d), vz * d.data[feld], lw=1.0,
                    label=f"Instanz {mid}{zusatz}")
        ax.set_ylabel(ylab)
        _finalisiere_zeit(ax, fenster, ereignisse)

    sel = hole(ulog, "estimator_selector_status")
    if sel is not None:
        ts = zeit(ulog, sel)
        axes[3].step(ts, sel.data["primary_instance"], where="post",
                     color="C0", label="primäre Instanz (Selector)")
        werte = sorted(set(sel.data["primary_instance"]))
        axes[3].set_yticks(werte)
    axes[3].set_ylabel("Instanz")
    _finalisiere_zeit(axes[3], fenster, ereignisse, autoscale_y=False)
    fig.suptitle("Vergleich der EKF-Instanzen", fontsize=sg("figur_titel"), fontweight="bold")
    return fig


# Registry: Name -> Funktion (Reihenfolge wie im Report)
PLOT_FUNCS = {
    "01_position_vision_ekf_sollwert": plot_position_vision_ekf_sollwert,
    "02_bahn_2d_3d": plot_bahn_2d_3d,
    "03_geschwindigkeit": plot_geschwindigkeit,
    "04_lage": plot_lage,
    "05_test_ratios": plot_test_ratios,
    "06_ev_innovationen": plot_ev_innovationen,
    "07_ekf_biases": plot_biases,
    "08_fusion_status": plot_fusion_status,
    "09_flugmodi": plot_flugmodi,
    "10_aussetzer_resets": plot_aussetzer_resets,
    "11_kovarianz": plot_kovarianz,
    "12_vertrauen": plot_vertrauen,
    "13_yaw_detail": plot_yaw_detail,
    "14_hoehenquellen": plot_hoehenquellen,
    "15_ekf_instanzen": plot_ekf_instanzen,
    "16_stuetzquellen": plot_stuetzquellen,
}


# ---------------------------------------------------------------------------
# Text-Zusammenfassung
# ---------------------------------------------------------------------------
def textzusammenfassung(ulog, inst, fenster, schwelle_s):
    """Erstellt eine textuelle Kennzahlen-Zusammenfassung für das Zeitfenster.

    Args:
        ulog: ULog-Objekt.
        inst: Primäre EKF-Instanz-ID.
        fenster: (t0, t1) ausgewerteter Zeitraum.
        schwelle_s: Schwelle für die Aussetzer-Erkennung in Sekunden.

    Returns:
        String mit der Zusammenfassung.
    """
    L = []
    def p(s=""):
        """Hängt eine Zeile an."""
        L.append(s)

    vlp = hole(ulog, "vehicle_local_position")
    t = zeit(ulog, vlp)
    m = maske(t, fenster)
    voll = voller_zeitraum(ulog)
    p("=" * 64)
    p("  FLUG-ZUSAMMENFASSUNG")
    p("=" * 64)
    p(f"Logdauer gesamt         : {voll[1] - voll[0]:.1f} s")
    p(f"Ausgewerteter Zeitraum  : {fenster[0]:.1f} .. {fenster[1]:.1f} s")
    ev_ereig = ereigniszeiten(ulog)
    p(f"Arming / Landung        : "
      f"{ev_ereig['Armed']:.1f} s / {ev_ereig['Landung']:.1f} s"
      if ev_ereig["Armed"] is not None and ev_ereig["Landung"] is not None
      else "Arming/Landung: n/a")

    x, y, z = vlp.data["x"][m], vlp.data["y"][m], vlp.data["z"][m]
    if len(x) > 1:
        strecke = np.nansum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2 + np.diff(z) ** 2))
        p(f"Max. Höhe (-z)          : {-np.nanmin(z):.2f} m")
        p(f"Bahn-Ausdehnung x/y/z   : {np.nanmax(x)-np.nanmin(x):.2f} / "
          f"{np.nanmax(y)-np.nanmin(y):.2f} / {np.nanmax(z)-np.nanmin(z):.2f} m")
        p(f"Zurückgelegte Strecke   : {strecke:.1f} m")

    vs = hole(ulog, "vehicle_status")
    nav = vs.data["nav_state"][maske(zeit(ulog, vs), fenster)]
    if len(nav):
        p("")
        p("Flugmodus-Anteile (im Zeitraum):")
        for v in sorted(set(nav)):
            p(f"   {NAV_STATE_NAMES.get(int(v), str(v)):16s}: {100*np.mean(nav==v):5.1f} %")

    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    me = None
    if ev is not None:
        me = maske(zeit(ulog, ev), fenster)
        if me.any():
            p("")
            p("External-Vision-Fusion (Position):")
            p(f"   fused-Anteil         : {100*np.mean(ev.data['fused'][me]):.1f} %")
            p(f"   rejected-Anteil      : {100*np.mean(ev.data['innovation_rejected'][me]):.2f} %")
            tr = np.maximum(ev.data["test_ratio[0]"][me], ev.data["test_ratio[1]"][me])
            p(f"   Test-Ratio mittel/max: {np.nanmean(tr):.4f} / {np.nanmax(tr):.4f}")

    p("")
    p("Belegung der Stützquellen (zeitgewichtet):")
    for feld, name in STUETZQUELLEN:
        a = flag_anteil(ulog, inst, feld, fenster)
        if a == a:
            p(f"   {name:22s}: {a:5.1f} %")

    aussetzer = finde_ev_aussetzer_flags(ulog, inst, fenster)
    p("")
    p("Unterbrechungen der Vision-Positionsstützung (aus cs_ev_pos):")
    if aussetzer:
        p(f"   Anzahl               : {len(aussetzer)} "
          f"(gesamt {sum(a[2] for a in aussetzer):.1f} s ohne Vision)")
        for ta, te, d in aussetzer:
            p(f"   {ta:7.1f} .. {te:7.1f} s   (Dauer {d:.2f} s)")
    else:
        p("   keine")

    resets = finde_resets(ulog, inst, fenster)
    p("")
    p("EKF-Resets (Zeitpunkte im Zeitraum):")
    for name in ["Pos NE", "Pos D", "Vel NE", "Vel D", "Quat"]:
        rt = resets.get(name, np.array([]))
        zeiten = ", ".join(f"{x:.1f}" for x in rt) if len(rt) else "-"
        p(f"   {name:7s}: {len(rt):2d}  @ [{zeiten}] s")

    st = hole(ulog, "estimator_states", inst)
    if st is not None:
        ms = maske(zeit(ulog, st), fenster)
        if ms.any():
            idx = kovarianz_indizes(st)
            sp = np.sqrt(np.abs(st.data[f"covariances[{idx['pos']}]"]))[ms]
            sv = np.sqrt(np.abs(st.data[f"covariances[{idx['vel']}]"]))[ms]
            p("")
            p("EKF-Unsicherheit (1σ):")
            p(f"   Position Nord        : mittel {np.nanmean(sp):.3f} m, max {np.nanmax(sp):.3f} m")
            p(f"   Geschwindigkeit Nord : mittel {np.nanmean(sv):.3f} m/s, max {np.nanmax(sv):.3f} m/s")
    if ev is not None and me is not None and me.any():
        K = np.clip(1.0 - ev.data["observation_variance[0]"][me]
                    / ev.data["innovation_variance[0]"][me], 0.0, 1.0)
        p(f"   Kalman-Gewicht Vision-Pos: median {np.nanmedian(K):.3f}, max {np.nanmax(K):.3f}")

    ev_yaw = hole(ulog, "estimator_aid_src_ev_yaw", inst)
    if ev_yaw is not None:
        my = maske(zeit(ulog, ev_yaw), fenster)
        if my.any():
            p("")
            p("External-Vision-Fusion (Yaw):")
            p(f"   fused-Anteil         : {100*np.mean(ev_yaw.data['fused'][my]):.1f} %")
            innov_deg = np.degrees(ev_yaw.data["innovation"][my])
            p(f"   Innovation mittel/max: {np.nanmean(np.abs(innov_deg)):.3f}° / "
              f"{np.nanmax(np.abs(innov_deg)):.3f}°")

    ts = hole(ulog, "trajectory_setpoint")
    if ts is not None and m.any():
        p("")
        p("Tracking-Fehler (Sollwert - EKF), RMS im Zeitraum:")
        t_sp = zeit(ulog, ts)
        for i, (ax, key) in enumerate(zip(["x", "y", "z"],
                                          ["position[0]", "position[1]", "position[2]"])):
            sp_i = np.interp(t[m], t_sp, ts.data[key])
            est_i = [x, y, z][i]
            ok = np.isfinite(sp_i) & np.isfinite(est_i)
            rms = np.sqrt(np.nanmean((sp_i[ok] - est_i[ok]) ** 2))
            p(f"   {ax}: {rms:.3f} m")
    p("=" * 64)
    return "\n".join(L)


# ---------------------------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------------------------
def main():
    """Liest Argumente, erzeugt aktivierte Plots, PDF-Report und Textzusammenfassung."""
    parser = argparse.ArgumentParser(
        description="PX4-ULog-Auswertung für Vision-gestützten EKF-Flug.")
    parser.add_argument("ulog", help="Pfad zur .ulg-Datei")
    parser.add_argument("--out", default=None,
                        help="Ausgabeordner (Standard: <logname>_auswertung/)")
    parser.add_argument("--instance", type=int, default=None,
                        help="EKF-Instanz (Standard: primäre aus selector_status)")
    parser.add_argument("--tmin", type=float, default=None,
                        help="Startzeit des Auswertungsfensters [s] (Standard: Logstart)")
    parser.add_argument("--tmax", type=float, default=None,
                        help="Endzeit des Auswertungsfensters [s] (Standard: Logende)")
    parser.add_argument("--phasen", default=None,
                        help="Phasen fuer die LaTeX-Kennzahlentabelle, Format "
                             "\"Name:t0:t1,Name:t0:t1\" (Standard: keine Tabelle)")
    parser.add_argument("--dropout-schwelle", type=float, default=None,
                        help="Lücke in s ab der ein Vision-Aussetzer zählt "
                             "(Standard: automatisch, 2.5 x Median-Log-Intervall)")
    args = parser.parse_args()

    ulog = lade_ulog(args.ulog)

    if args.instance is not None:
        inst = args.instance
    else:
        sel = hole(ulog, "estimator_selector_status")
        inst = int(sel.data["primary_instance"][-1]) if sel is not None else 0

    # Aussetzer-Schwelle: fest vorgegeben oder automatisch aus Log-Intervall
    if args.dropout_schwelle is not None:
        schwelle = args.dropout_schwelle
        schwelle_info = "fest vorgegeben"
    else:
        schwelle = auto_dropout_schwelle(ulog, inst)
        schwelle_info = "automatisch (2.5 x Median-Log-Intervall der Vision-Topics)"

    # Zeitfenster auflösen: (a) ganzer Flug oder (b) --tmin/--tmax
    voll = voller_zeitraum(ulog)
    t0 = args.tmin if args.tmin is not None else voll[0]
    t1 = args.tmax if args.tmax is not None else voll[1]
    fenster = (t0, t1)
    gefenstert = (args.tmin is not None) or (args.tmax is not None)
    ereignisse = ereigniszeiten(ulog)

    if args.out is None:
        basis = os.path.splitext(os.path.basename(args.ulog))[0]
        args.out = f"{basis}_auswertung"
    os.makedirs(args.out, exist_ok=True)

    print(f"Auswertung läuft ... (EKF-Instanz {inst})")
    print(f"Zeitraum: {t0:.1f} .. {t1:.1f} s"
          + ("  [Fenster aktiv]" if gefenstert else "  [ganzer Flug]"))
    print(f"Aussetzer-Schwelle: {schwelle:.2f} s ({schwelle_info})")
    print(f"Ausgabeordner: {os.path.abspath(args.out)}\n")

    aktive = [(n, f) for n, f in PLOT_FUNCS.items() if PLOTS_AKTIV.get(n, True)]
    aussetzer_alle = ermittle_aussetzer(ulog, inst, schwelle, fenster)
    print(f"Vision-Aussetzer im Zeitraum: {len(aussetzer_alle)} "
          f"(Quelle: {AUSSETZER_QUELLE})\n")
    pdf_pfad = os.path.join(args.out, "report.pdf")
    with PdfPages(pdf_pfad) as pdf:
        for name, fn in aktive:
            try:
                setze_schriftgroessen(name)
                setze_aussetzer([] if name in AUSSETZER_AUSNAHMEN else aussetzer_alle)
                fig = fn(ulog, inst, fenster, ereignisse, schwelle)
                if gefenstert:
                    fig.text(0.99, 0.005, f"Zeitfenster {t0:.1f}-{t1:.1f} s",
                             ha="right", va="bottom", fontsize=sg("fussnote"), alpha=0.5)
                fig.savefig(os.path.join(args.out, name + ".png"), dpi=130)
                pdf.savefig(fig)
                plt.close(fig)
                print(f"  [ok] {name}.png")
            except Exception as e:
                print(f"  [übersprungen] {name}: {e}")

    phasen = parse_phasen(args.phasen)
    if phasen:
        tex = kennzahlen_latex(ulog, inst, phasen)
        tex_pfad = os.path.join(args.out, "kennzahlen.tex")
        with open(tex_pfad, "w", encoding="utf-8") as f:
            f.write(tex + "\n")
        print(f"\n  [ok] kennzahlen.tex ({len(phasen)} Phasen)")

    txt = textzusammenfassung(ulog, inst, fenster, schwelle)
    print("\n" + txt)
    with open(os.path.join(args.out, "zusammenfassung.txt"), "w",
              encoding="utf-8") as f:
        f.write(txt + "\n")
    print(f"\nFertig. PDF-Report: {pdf_pfad}")


if __name__ == "__main__":
    main()
