# Auswertung der Karten (`Maps`)

Dieser Ordner enthält die Auswertung der mit TagSLAM erzeugten AprilTag-Karten.
Das Skript vergleicht bis zu vier Karten im TagSLAM-YAML-Format gegen eine Referenzkarte (Ground Truth) und erzeugt daraus Grafiken, CSV-Dateien und einen Textreport.

## Ordnerstruktur

```
Maps
├── auswertung_karten.py     # Auswertungsskript
├── Beispiel_Auswertung      # Beispiel-Output einer vollständigen Auswertung
└── Karten                   # Eingangsdaten
    ├── ground_truth_config3.yaml   # Referenzkarte (Ground Truth)
    ├── positionen_lauf1.yaml       # aufgenommene Karte, Versuch 1
    ├── positionen_lauf2.yaml       # aufgenommene Karte, Versuch 2
    └── positionen_lauf3.yaml       # aufgenommene Karte, Versuch 3
```

- **`auswertung_karten.py`** – Skript zum Auswerten der Karten.
- **`Karten`** – die drei aufgenommenen Beispielkarten sowie die Ground Truth.
- **`Beispiel_Auswertung`** – ein Beispiel-Output der Auswertung (alle erzeugten Grafiken, CSV-Dateien und der Report).

## Was das Skript macht

Verglichen werden ausschließlich die Tags des `tag_map`-Bodys. Das Skript liefert unter anderem:

- eine überlagerte Draufsicht (x-y) aller Karten sowie Einzel-Teilplots,
- eine Overlay-Grafik mit Entfernungsringen um den Ursprung,
- Balkendiagramme der mittleren Abweichung je Markergröße und je Entfernungsring,
- je ein Diagramm der Positions- und Orientierungsabweichung pro Tag, sortiert nach Abstand zum Ursprung,
- die euklidische Distanz `d_xyz` über dem Abstand zum Bezugstag (Balken- und Punktdiagramm mit optionaler Ausgleichsgerade),
- einen numerischen Vergleich gleicher Tag-IDs gegen die Referenz (Konsole, CSV, Textreport),
- eine paarweise Abstandsprüfung aller Tag-Paare inklusive geschätztem Maßstabsfaktor.

Positionskennzahlen werden in Millimetern, Orientierungskennzahlen in Grad ausgegeben. Markergröße und Abstand zum Ursprung werden immer aus der Referenzkarte übernommen, damit die Gruppenzuordnung für alle Karten identisch ist.

## Installation

Voraussetzung ist Python 3. Empfohlen wird eine virtuelle Umgebung:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Benötigte Pakete installieren:

```bash
pip install numpy pyyaml matplotlib
```

## Konfiguration

Das Skript kennt **keine Kommandozeilenargumente**. Alle Einstellungen stehen im Block `KONFIGURATION` am Anfang von `auswertung_karten.py`.

Vor dem ersten Lauf sind mindestens die Kartenpfade anzupassen. Im Beispiel:

```python
MAPS = [
    {"path": "Karten/ground_truth_config3.yaml", "color": "black",      "label": "Ground Truth"},
    {"path": "Karten/positionen_lauf1.yaml",     "color": "tab:blue",   "label": "Kartierung Versuch 1"},
    {"path": "Karten/positionen_lauf2.yaml",     "color": "tab:orange", "label": "Kartierung Versuch 2"},
    {"path": "Karten/positionen_lauf3.yaml",     "color": "tab:green",  "label": "Kartierung Versuch 3"},
]

REFERENCE_INDEX = 0      # Index der Referenzkarte in MAPS (0 = erste Karte)
OUTPUT_DIR      = "."    # Zielordner für alle Ausgabedateien
OUTPUT_BASENAME = "karten_vergleich"   # Präfix aller Ausgabedateien
```

Weitere Optionen im selben Block betreffen unter anderem die Entfernungsringe (`RING_EDGES`), die Klassen der paarweisen Abstände (`PAIR_EDGES`), die dargestellten Kennzahlen (`BAR_METRIC`, `PER_TAG_POS_METRIC`, `PER_TAG_ROT_METRIC`), das Ausgabeformat (`SAVE_PDF`, `SAVE_PNG`, `PNG_DPI`) sowie die Schriftgrößen (`FONT_SIZES`).

## Ausführen

Nach dem Anpassen der Pfade genügt:

```bash
python3 auswertung_karten.py
```

Das Skript benötigt keine grafische Oberfläche (matplotlib läuft im `Agg`-Backend) und schreibt ausschließlich Dateien. Nicht gefundene Karten werden übersprungen; ist nur eine Karte aktiv, entfällt der numerische Vergleich.

## Ausgabedateien

Alle Dateien werden mit dem Präfix `OUTPUT_BASENAME` in `OUTPUT_DIR` abgelegt, jeweils als PDF und PNG (Grafiken) bzw. CSV und TXT (Daten). Ein vollständiges Beispiel liegt im Ordner `Beispiel_Auswertung`, unter anderem:

- `karten_vergleich_ueberlagert.*`, `karten_vergleich_teilplots.*`, `karten_vergleich_ringe.*`
- `karten_vergleich_groessenklassen.*`, `karten_vergleich_entfernungsringe.*`, `karten_vergleich_paarweise.*`
- `karten_vergleich_je_tag_position.*`, `karten_vergleich_je_tag_orientierung.*`
- `karten_vergleich_dxyz_ueber_tag0_balken.*`, `karten_vergleich_dxyz_ueber_tag0_punkte.*`
- `karten_vergleich_abweichungen.csv`, `karten_vergleich_einzeltags.csv`, `karten_vergleich_report.txt`
