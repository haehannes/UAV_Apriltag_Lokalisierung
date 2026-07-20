# AprilTag UAV Lokalisierung – Karten und Ground Truth

Dieses Verzeichnis enthält die Werkzeuge zur Erzeugung der AprilTag-Karten für die markerbasierte Indoor-Lokalisierung von UAVs sowie die zugehörige Ground Truth.

## Verzeichnisstruktur

```
~/Apriltag_UAV_Lokalisierung/AprilTags$ tree -L 2
.
├── Final_map
│   ├── apriltag_map.pdf
│   ├── ground_truth_final.yaml
│   └── marker_config_final.yaml
├── gen_ground_truth_map.py
├── gen_Karte_druck.py
├── marker_config.yaml
├── readme.md
└── Tag_36h11
    └── tag36h11.pdf
```

| Element | Beschreibung |
| --- | --- |
| `Final_map/` | Die final am OIC aufgeklebte Karte mit zugehöriger Ground Truth und Config (siehe unten). |
| `gen_ground_truth_map.py` | Skript zum Erzeugen der Ground Truth für die Positionierung der Drohne. |
| `gen_Karte_druck.py` | Skript zum Erzeugen einer PDF-Datei zum Drucken der Karte. |
| `marker_config.yaml` | Konfigurationsdatei zum Festlegen der AprilTags und ihrer Koordinaten (siehe unten). |
| `Tag_36h11/` | PDF-Datei (`tag36h11.pdf`) mit allen Tags der Familie tag36h11. |

## Final_map

Enthält die aktuell am OIC verwendete Karte.

| Datei | Beschreibung |
| --- | --- |
| `apriltag_map.pdf` | PDF der aufgeklebten Karte am OIC. |
| `ground_truth_final.yaml` | Ground Truth zur zugehörigen Karte. |
| `marker_config_final.yaml` | Config-Datei zum Erzeugen dieser Karte. |

## Skripte

### `gen_Karte_druck.py`

Erzeugt aus einer Konfigurationsdatei die PDF-Datei zum Drucken der Karte. Sinnvoll für das Plotten einer neuen Karte oder einer Erweiterung.

```bash
python3 gen_Karte_druck.py [konfig.yaml]
```

### `gen_ground_truth_map.py`

Erzeugt die Ground Truth für die Positionierung der Drohne. Benötigt zusätzlich die Karte (`map.yaml`).

```bash
python3 gen_ground_truth_map.py [konfig.yaml] [map.yaml]
```

## Konfiguration (`marker_config.yaml`)

In der YAML-Datei werden Feldabmessungen, Marker-Standardwerte, Ausgabeoptionen und die einzelnen Marker definiert:

```yaml
field:
  length: 4.0
  lane_width: 1.0
  n_lanes: 4
defaults:
  family: tag36h11
  rotate_180: true
output:
  path: apriltag_lanes.pdf
  px_per_module: 100
  quiet_modules: 1.0
  label: true
  registration_marks: true
markers:
  - {id:   0, x: 2.000, y: 2.100, size: 0.167}
```

### Parameter

- **`field`** – Abmessungen des Felds: Länge (`length`), Bahnbreite (`lane_width`) und Anzahl der Bahnen (`n_lanes`).
- **`defaults`** – Standardwerte für alle Marker: Tag-Familie (`family`) und optionale 180°-Drehung (`rotate_180`).
- **`output`** – Einstellungen der PDF-Ausgabe: Dateipfad (`path`), Auflösung pro Modul (`px_per_module`), Breite der Quiet Zone (`quiet_modules`), Beschriftung (`label`) und Passermarken (`registration_marks`).
- **`markers`** – Liste der einzelnen Marker mit ID, Position (`x`, `y`) und Kantenlänge (`size`) in Metern.
