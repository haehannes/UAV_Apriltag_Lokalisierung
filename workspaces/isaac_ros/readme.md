# Isaac ROS Workspace

Dieser Workspace enthält die ROS2-Pakete und Launch-Dateien für die visionsgestützte Lokalisierung auf dem Jetson.
Über Isaac ROS werden die Kamerabilder **hardwarebeschleunigt** entzerrt und die AprilTags detektiert.

## Voraussetzung: Isaac-ROS-Container

> **Wichtig:** Die Launch-Dateien lassen sich ausschließlich innerhalb des Isaac-ROS-Containers ausführen.

Der Container wird benötigt, um

- das Kamerabild zu entzerren (Rektifizierung) und
- die AprilTags hardwarebeschleunigt zu detektieren.

Auf dem Jetson öffnet der Befehl

```bash
start_isaac
```

den Workspace automatisch im Container. Alle folgenden Befehle werden innerhalb dieses Containers ausgeführt.

## Ordnerstruktur

```
.
├── launch
│   ├── dual_cam_.launch.py            # zwei Kameras (Kalibrierung)
│   └── single_cam_1280_720.launch.py  # eine Kamera, 1280 x 720 (Flug)
└── src
    ├── control_package                # Steuerung / Bedienkonsole
    ├── drone_camera_calibration       # Kamerakalibrierung (u. a. extrinsisch)
    ├── drone_vision_pose_publisher    # Positionsbestimmung und Ausgabe der Pose
    ├── isaac_ros_apriltag_interfaces  # Nachrichtentypen der Isaac-AprilTag-Erkennung
    ├── isaac_ros_common               # gemeinsame Isaac-ROS-Basis / Container-Infrastruktur
    └── px4_msgs                       # PX4-Nachrichtentypen (Anbindung an den Flugcontroller)
```

- **`launch`** – Startdateien für den Betrieb im Container.
- **`src`** – die ROS2-Pakete des Systems.

## Verarbeitungspipeline

Beide Launch-Dateien starten einen gemeinsamen, mehrsträngigen Container (`isaac_ros_container`, `component_container_mt`) und laden darin je Kamera zwei Composable Nodes:

- einen **`RectifyNode`** (`isaac_ros_image_proc`), der das Rohbild entzerrt und mit `1280 x 720` ausgibt,
- einen **`AprilTagNode`** (`isaac_ros_apriltag`), der die entzerrten Bilder auswertet.

Datenfluss je Kamera:

```
/camera_X/image_raw ─┐
                     ├─► RectifyNode ─► /camera_X/image_rect
/camera_X/camera_info┘                 /camera_X/camera_info_rect
                                              │
                                              ▼
                                        AprilTagNode ─► /camera_X/tag_detections
```

Die Rohbilder (`/camera_X/image_raw`, `/camera_X/camera_info`) liefert die jeweils zuständige Kamera aus `ws_humble` (gscam). Die Tag-Detektionen (`/camera_X/tag_detections`) verwenden anschließend die Positionsbestimmung bzw. der Kalibrier-Node.

Konfiguration der AprilTag-Erkennung:

| Launch-Datei | Kameras | Tag-Familie | Tag-Größe | max. Tags |
| --- | --- | --- | --- | --- |
| `single_cam_1280_720.launch.py` | `camera_1` | `tag36h11` | `0.170` m | 40 |
| `dual_cam_.launch.py` | `camera_1`, `camera_2` | `tag36h11` | `0.165` m | 40 |

> **Hinweis:** Die Tag-Größe (`size`) ist der einzige inhaltliche Unterschied der AprilTag-Nodes: `single_cam` nutzt `0.170` m (Kartenreferenz), `dual_cam` nutzt `0.165` m (Marker der Kalibrierbox).

## Starten

Alle Schritte innerhalb des Containers ausführen (siehe `start_isaac`).

1. Workspace gegebenenfalls bauen:

   Ist der Workspace noch nicht gebaut (kein `install`-Ordner vorhanden) oder wurden Pakete geändert, muss zunächst gebaut werden:

   ```bash
   colcon build --symlink-install
   ```

   Ist bereits ein aktueller Build vorhanden, kann dieser Schritt übersprungen werden.

2. Workspace sourcen:

   ```bash
   source install/setup.bash
   ```

3. Launch-Datei starten:

   Für den Flug (eine Bodenkamera):

   ```bash
   ros2 launch launch/single_cam_1280_720.launch.py
   ```

   Für die Kalibrierung (zwei Kameras):

   ```bash
   ros2 launch launch/dual_cam_.launch.py
   ```

## Was danach läuft

Nach dem Start laufen im Container `isaac_ros_container` folgende Composable Nodes:

- **`rectify_1`** (und bei `dual_cam` zusätzlich **`rectify_2`**) – entzerrt das Kamerabild,
- **`apriltag_1`** (und bei `dual_cam` zusätzlich **`apriltag_2`**) – detektiert die AprilTags hardwarebeschleunigt.

## Hinweise

- **Kamera für den Flug:** Für den Flug wird nur eine einzige Kamera genutzt, die mehr als 30 FPS liefert (`single_cam`). Die Kamerabilder liefert `ws_humble` (gscam).
- **Auflösung anpassen:** Bei abweichender Kameraauflösung müssen `output_width`/`output_height` der `RectifyNode`s sowie die AprilTag-`size` entsprechend angepasst werden.
- **RViz:** Zur Kontrolle der Entzerrung und der Tag-Detektionen hat sich RViz als sehr nützliches Werkzeug erwiesen.
