# Workspace `ws_jazzy` – Kartierung

In diesem Workspace wird die Kartierung mit **TagSLAM** durchgeführt.
Er funktioniert nur unter **ROS2 Jazzy**.

> **Hinweis:** TagSLAM arbeitet durchgehend im **Rodrigues-Format** (Achse-Winkel, Rotationsvektor). Alle Orientierungen in den Einstellungen und in den Ergebnisdateien sind entsprechend als Rodrigues-Vektoren zu lesen.

## Verzeichnisstruktur

```
ws_jazzy
├── Einstellungen
│   ├── camera_1920x1080_c922.yaml
│   ├── camera_poses.yaml
│   ├── cameras_rect.yaml
│   ├── tagslam_Lauf1.yaml
│   ├── tagslam_Lauf2.yaml
│   └── tagslam_Lauf3.yaml
├── launch
│   ├── launch_map
│   ├── offline_sync_and_detect.launch.py
│   ├── record_tagslam_input.launch.py
│   └── View_map_drone_vision_publisher.py
├── readme.md
└── src
    ├── apriltag_map_tf
    ├── flex_sync
    └── tagslam
```

| Verzeichnis | Beschreibung |
| --- | --- |
| `Einstellungen/` | Alle Einstellungen für TagSLAM. |
| `launch/` | Launch-Dateien des dreistufigen Kartierungsablaufs sowie die Kartenoptimierung in `launch_map/`. |
| `src/` | Der eigentlich notwendige Code (u. a. das TagSLAM-Paket). |

## Bauen und Sourcen

Ist der Workspace noch nicht gebaut (kein `install`-Ordner vorhanden) oder wurden Pakete geändert, muss zunächst gebaut werden:

```bash
colcon build --symlink-install
```

Anschließend – und in jedem neuen Terminal – den Workspace sourcen:

```bash
source install/setup.bash
```

## `Einstellungen/`

Hier liegen die Einstellungen für das Paket **TagSLAM**. Die Bedeutung der Parameter ist in der offiziellen Dokumentation beschrieben:
<https://berndpfrommer.github.io/tagslam_web/>

| Datei | Beschreibung |
| --- | --- |
| `camera_1920x1080_c922.yaml` | Kamera-Intrinsics des Rohbilds (C922, 1920 × 1080), verwendet bei der Bag-Aufnahme. |
| `cameras_rect.yaml` | Kamera-Intrinsics für das bereits entzerrte Bild, verwendet bei Detektion und Optimierung. |
| `camera_poses.yaml` | Legt fest, wie frei sich die Kamera bewegen darf/kann (siehe unten). |
| `tagslam_Lauf1.yaml` | TagSLAM-Config (grober Lauf), verwendet bei der Detektion. |
| `tagslam_Lauf2.yaml` | TagSLAM-Config (genauerer Lauf); Tags stärker auf eine Ebene und in der Bewegung fixiert. |
| `tagslam_Lauf3.yaml` | TagSLAM-Config für die eigentliche Kartenoptimierung (`tagslam_from_bag`). |

### `camera_poses.yaml`

Über die Kovarianzmatrix `R` wird festgelegt, wie frei sich die Kamera bewegen darf. Große Werte bedeuten eine sehr freie Bewegung:

```yaml
cam0:
  body: camera
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
    R: [1.0e6, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0e6, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0e6, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0e6, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0e6, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0e6]
# --> Sehr frei
```

## Ablauf der Kartierung (`launch/`)

Die Kartierung erfolgt in **drei Stufen**. Die Bag- und Config-Pfade sind in den Launch-Dateien hinterlegt und müssen gegebenenfalls angepasst werden.

```
launch
├── launch_map
│   ├── Output_Beispiel        # Beispiel-Output nach erfolgreichem Durchlauf
│   └── tagslam_from_bag.launch.py
├── offline_sync_and_detect.launch.py
├── record_tagslam_input.launch.py
└── View_map_drone_vision_publisher.py
```

### Stufe 1 – Bag aufnehmen (`record_tagslam_input.launch.py`)

Aufzeichnen eines Bags der Karte mit **einer Kamera**.
Das Launch-File startet die Kamera (gscam, C922, 1920 × 1080 bei 15 FPS, CPU-JPEG-Decoding – **nicht auf dem Jetson**), entzerrt das Bild über `rectify_node` (`/camera/image_raw → /camera/image_rect`) und nimmt nach kurzer Vorlaufzeit folgende Topics auf:

- `/camera/image_rect`
- `/camera/camera_info`
- `/tf_static`

Start:

```bash
ros2 launch launch/record_tagslam_input.launch.py
```

Während der Aufnahme die **Kamera möglichst senkrecht über die Karte** bewegen und dabei möglichst viele Tags gleichzeitig im Sichtfeld halten. Immer wieder an bereits abgefahrene Stellen zurückkehren, damit Loop-Closure-Constraints entstehen.

### Stufe 2 – Tags detektieren (`offline_sync_and_detect.launch.py`)

Detektion der Tags auf dem zuvor aufgenommenen Bag (Pfad des letzten Bags verwenden).
Das Launch-File startet den `sync_and_detect_node`, spielt den Roh-Bag langsam (`--rate 0.3`, mit `--clock` für simulierte Zeit) ab und nimmt einen neuen **kombinierten Bag** auf, der alles für `tagslam_from_bag` enthält:

- `/camera/image_rect`
- `/camera/camera_info`
- `/tagslam/tag_detections`
- `/tf_static`
- `/clock` – wird ebenfalls aufgenommen, weil offline mit simulierter Zeit gearbeitet wird.

Start:

```bash
ros2 launch launch/offline_sync_and_detect.launch.py
```

Dieser Schritt kann durchaus einige Minuten laufen.

### Stufe 3 – Kartieren (`launch_map/tagslam_from_bag.launch.py`)

Die eigentliche Kartenoptimierung aus dem kombinierten Bag.
Verwendet werden `cameras_rect.yaml`, `camera_poses.yaml` und `tagslam_Lauf3.yaml`. Die Ergebnisse werden anschließend automatisch im Verzeichnis `launch_map/` abgelegt.

Start:

```bash
ros2 launch launch/launch_map/tagslam_from_bag.launch.py
```

Je nach Einstellungen und Größe der Karte läuft dieser Schritt einige Stunden, im ungünstigsten Fall Tage.

Im Ordner `launch_map/Output_Beispiel/` liegt ein Beispiel-Output nach einem erfolgreichen Durchlauf.

> **`poses.yaml` ist die eigentliche Karte.**

## `src/`

Hier liegt der eigentlich notwendige Code.

| Paket | Beschreibung |
| --- | --- |
| `tagslam` | Das TagSLAM-Paket zur Kartierung. **Nicht mehr das Original** – für `tagslam_from_bag` angepasst. |
| `flex_sync` | Zeitliche Synchronisation der Nachrichten; von TagSLAM benötigt. |
| `apriltag_map_tf` | Veröffentlicht die Tag-Karte als statische TFs (u. a. zur Visualisierung). |

## Ablauf (Kurzfassung)

1. **Bag aufnehmen** – `ros2 launch launch/record_tagslam_input.launch.py` (Kamera senkrecht über die Karte bewegen).
2. **Tags detektieren** – `ros2 launch launch/offline_sync_and_detect.launch.py` (erzeugt kombinierten Bag mit Tag-Detektionen).
3. **Kartieren** – `ros2 launch launch/launch_map/tagslam_from_bag.launch.py` (erzeugt die Karte `poses.yaml` in `launch_map/`).

## Hinweise

- **Angepasstes TagSLAM:** Das enthaltene TagSLAM-Paket ist nicht mehr das Original, sondern wurde für `tagslam_from_bag` angepasst.
- **Pfade anpassen:** In allen Launch-Dateien müssen die Pfade an das eigene System angepasst werden. Die ROS-Bags sind nicht Teil des Repositories und werden erst bei der Aufnahme erzeugt.
- **Videogerät:** In der Aufnahme-Launch-Datei muss das richtige V4L2-Gerät (`/dev/videoX`) ausgewählt werden.
- **Kamera-Intrinsics:** Passend zur eingestellten Kameraauflösung sind die richtigen Intrinsics zu wählen (`camera_1920x1080_c922.yaml` für das Rohbild, `cameras_rect.yaml` für das entzerrte Bild).
- **RViz:** Zur Kontrolle und Visualisierung der Kartierung hat sich RViz als sehr nützliches Werkzeug erwiesen.
