# Workspace `ws_jazzy` – Kartierung

In diesem Workspace wird die Kartierung mit TagSLAM durchgeführt. Er funktioniert nur unter **ROS2 Jazzy**.

## Verzeichnisstruktur

```
ws_jazzy$ ls
Einstellungen  launch  launch_map  src
```

| Verzeichnis | Beschreibung |
| --- | --- |
| `Einstellungen/` | Alle Einstellungen für TagSLAM. |
| `launch/` | Launch-Dateien zur Bag-Aufnahme und Tag-Detektion. |
| `launch_map/` | Start der SLAM-Kartierung; hier werden anschließend die Ergebnisse abgelegt. |
| `src/` | Der eigentliche notwendige Code. |

## `Einstellungen/`

```
Einstellungen$ ls
camera_1920x1080_c922.yaml  camera_poses.yaml  cameras.yaml
tagslam_Lauf1.yaml          tagslam_Lauf2.yaml
```

| Datei | Beschreibung |
| --- | --- |
| `camera_1920x1080_c922.yaml` | Kamera-Intrinsics. |
| `cameras.yaml` | Kamera-Intrinsics für das bereits entzerrte Bild. |
| `camera_poses.yaml` | Gibt an, wie frei sich die Kamera bewegen darf/kann (siehe unten). |
| `tagslam_Lauf1.yaml` | Config-Datei für TagSLAM (groberer Lauf). |
| `tagslam_Lauf2.yaml` | Config-Datei für TagSLAM (genauerer Lauf); Tags wurden stärker auf eine Ebene und in der Bewegung fixiert. |

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

## `launch/`

```
launch$ ls
offline_sync_and_detect.launch.py  record_tagslam_input.launch.py
```

### `record_tagslam_input.launch.py`

Aufnahme des ersten Bags. Die Dateien aus `Einstellungen/` sind notwendig; Pfade müssen ggf. angepasst werden.

Erst wieder aktivieren, wenn `/camera/image_raw` und `/camera/image_rect` sicher laufen. Aufgenommen werden folgende Topics:

- `/camera/image_rect`
- `/camera/camera_info`
- `/tf_static`

### `offline_sync_and_detect.launch.py`

Tags werden detektiert; der vorherige Bag ist notwendig. Pfade müssen ggf. angepasst werden, die Dateien aus `Einstellungen/` werden benötigt.

Es entsteht ein neuer kombinierter Bag, der anschließend alles für `tagslam_from_bag` enthält:

- `/camera/image_rect`
- `/camera/camera_info`
- `/tagslam/tag_detections`
- `/tf_static`
- `/clock` – wird ebenfalls aufgenommen, weil offline mit simulierter Zeit gearbeitet wird.

## `launch_map/`

```
launch_map$ ls
calibration.yaml   camera_poses.yaml  error_map.txt  poses.yaml
tag_corners.txt    tag_diagnostics.txt  time_diagnostics.txt
tagslam_from_bag.launch.py
```

### `tagslam_from_bag.launch.py`

Start der SLAM-Kartierung (kann je nach Größe der Karte einige Zeit laufen). In diesem Verzeichnis werden anschließend automatisch Dateien abgelegt.

- `camera_poses.yaml` – **wichtigste Output-Datei** (ist die Karte selbst).

## `src/`

Hier liegt der eigentlich notwendige Code.

## Ablauf (Kurzfassung)

1. **Bag aufnehmen** – `record_tagslam_input.launch.py` (nimmt Rohaufnahme auf).
2. **Tags detektieren** – `offline_sync_and_detect.launch.py` (erzeugt kombinierten Bag mit Tag-Detektionen).
3. **Kartieren** – `tagslam_from_bag.launch.py` (erzeugt die Karte `camera_poses.yaml` in `launch_map/`).
