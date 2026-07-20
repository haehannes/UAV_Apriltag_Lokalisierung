# Auswertung der Log-Dateien (`Ulog`)

Dieser Ordner enthält die Auswertung der PX4-ULog-Dateien eines Testflugs mit visionsgestützter, GNSS-freier Lokalisierung (External Vision, `EKF2_EV_CTRL`).
Das Skript liest eine `.ulg`-Datei ein und erzeugt daraus eine Reihe von Diagrammen, einen zusammengefassten PDF-Report und eine Textzusammenfassung.

## Ordnerstruktur

```
Ulog
├── ulog_auswertung.py       # Auswertungsskript
├── Log_file                 # Eingangsdaten
│   └── Beispiel_Log.ulg     # Beispiel-Log-Datei eines Testflugs
└── Beispiel_Auswertung      # Beispiel-Output einer vollständigen Auswertung
    ├── 01_position_vision_ekf_sollwert.png
    ├── ...
    ├── 16_stuetzquellen.png
    ├── report.pdf
    └── zusammenfassung.txt
```

- **`ulog_auswertung.py`** – Skript zum Auswerten der Log-Dateien.
- **`Log_file`** – enthält eine Beispiel-Log-Datei (`.ulg`).
- **`Beispiel_Auswertung`** – ein Beispiel-Output der Auswertung (alle Diagramme, `report.pdf` und `zusammenfassung.txt`).

## Was das Skript macht

Ausgewertet werden unter anderem:

- EKF-Zustände (Position, Geschwindigkeit, Lage, Sensor-Biases),
- Vision-Vorgabe (External Vision) gegenüber EKF-Schätzung und Offboard-Sollwert,
- die tatsächlich geflogene Bahn (2D, 3D, Höhe),
- die Fusions-Gesundheit (Innovationen, Test-Ratios, Fused/Rejected, Aussetzer),
- die EKF-Unsicherheit (Kovarianz) und der Kalman-Gain,
- Yaw-Fusion, Höhenquellen-Vergleich und Vergleich der EKF-Instanzen,
- Flugmodi, Arming und Land-Detector.

In jedem Zeitdiagramm sind Arming (grün) und Landung (rot) markiert sowie die Vision-Aussetzer orange hinterlegt. Welche Diagramme erzeugt werden, steuert der Block `PLOTS_AKTIV` direkt unter den Imports (`True` = an, `False` = aus).

## Installation

Voraussetzung ist Python 3. Empfohlen wird eine virtuelle Umgebung:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Benötigte Pakete installieren:

```bash
pip install pyulog numpy matplotlib
```

Fehlt `pyulog`, bricht das Skript mit einem entsprechenden Hinweis ab.

## Ausführen

Das Skript erwartet die Log-Datei als Argument.

Ganzer Flug:

```bash
python3 ulog_auswertung.py Log_file/Beispiel_Log.ulg
```

Nur ein Zeitfenster (in Sekunden seit Logstart); die Plots werden auf dieses Fenster skaliert:

```bash
python3 ulog_auswertung.py Log_file/Beispiel_Log.ulg --tmin 240 --tmax 266
```

Weitere Optionen:

```bash
python3 ulog_auswertung.py Log_file/Beispiel_Log.ulg --out ordner/ --instance 0 --dropout-schwelle 1.5
```

| Option | Bedeutung |
| --- | --- |
| `--tmin` / `--tmax` | Zeitfenster in Sekunden seit Logstart |
| `--out` | Zielordner für die Ausgabedateien |
| `--instance` | auszuwertende EKF-Instanz (Standard 0) |
| `--dropout-schwelle` | feste Schwelle für die Vision-Aussetzer |

Die Aussetzer-Schwelle wird standardmäßig automatisch aus dem Log-Intervall der Vision-Topics abgeleitet (2,5-faches Median-Intervall), da diese Topics vom Logger nur mit reduzierter Rate (etwa 2 Hz) aufgezeichnet werden. Mit `--dropout-schwelle` lässt sie sich fest vorgeben.

Das Skript benötigt keine grafische Oberfläche (matplotlib läuft im `Agg`-Backend) und schreibt ausschließlich Dateien.

## Ausgabedateien

Erzeugt werden die im Block `PLOTS_AKTIV` aktivierten Diagramme (`01_...png` bis `16_...png`), ein zusammengefasster `report.pdf` sowie eine Textzusammenfassung `zusammenfassung.txt`. Ein vollständiges Beispiel liegt im Ordner `Beispiel_Auswertung`.
