# AprilTag UAV Lokalisierung

Dieses Repository bündelt die Werkzeuge, Konfigurationen und Workspaces zur markerbasierten Indoor-Lokalisierung eines UAVs mittels AprilTags.

## Verzeichnisstruktur

```
Apriltag_UAV_Lokalisierung$ ls
AprilTags  Auswertungen  Kamera_Kalibrierung  Parameter_PX4_6C  workspaces
```

| Verzeichnis | Beschreibung |
| --- | --- |
| `AprilTags/` | Alles zum Drucken einer physischen Karte und zur Ground Truth. |
| `Auswertungen/` | Auswertungen zu Karten und PX4-Log-Files. |
| `Kamera_Kalibrierung/` | Kalibrierdateien der verwendeten Logitech-Kameras. |
| `Parameter_PX4_6C/` | Die letzten flugfähigen Einstellungen des Flightcontrollers. |
| `workspaces/` | Die einzelnen ROS2-Workspaces (siehe unten). |

## AprilTags

```
AprilTags$ ls
Final_map  gen_ground_truth_map.py  gen_Karte_druck.py  marker_config.yaml  readme.md  Tag_36h11
```

| Inhalt | Beschreibung |
| --- | --- |
| `Tag_36h11/` | PDF mit allen Tags der Familie tag36h11. |
| `Final_map/` | Die final am OIC geklebte Karte. |
| `gen_Karte_druck.py` | Skript zum Generieren einer PDF zum Drucken einer Karte. |
| `gen_ground_truth_map.py` | Skript zum Generieren der Ground Truth. |
| `marker_config.yaml` | Config-File zum Einstellen der Koordinaten der AprilTags auf der Karte. |

## Auswertungen

Enthält Auswertungen zu Karten und Log-Files.

```
Auswertungen$ ls
Maps  Ulog
```

### Ulog

| Inhalt | Beschreibung |
| --- | --- |
| Auswerte-Skript | Skript zum Durchführen von Auswertungen der PX4-Log-Files. |
| Beispiel-Log | Ein Beispiel-Log-File von PX4. |
| Beispielauswertung | Eine fertige Beispielauswertung. |

### Maps

| Inhalt | Beschreibung |
| --- | --- |
| Beispielkarte | Eine Beispielkarte. |
| Karten | Vier Karten (von TagSLAM erzeugt sowie die Ground Truth). |
| Auswerte-Skript | Skript zum Erstellen der Auswertungen. |

## Kamera_Kalibrierung

Enthält die Kalibrierdateien der verwendeten Logitech-Kameras (C920 und C922) sowie das Schachbrett zum Ausdrucken.

## Parameter_PX4_6C

Enthält die Einstellungen des PX4-Controllers für den Betrieb im Innenraum (`px4_6C_final.params`).

## Workspaces

```
workspaces$ ls
isaac_ros  ws_humble  ws_jazzy
```

| Workspace | Zweck | Hinweise |
| --- | --- | --- |
| `isaac_ros/` | Für den Container und Isaac ROS (Detektion). Beim Start des Containers ist dies das Ausgangsverzeichnis. | Nur auf dem Jetson (ROS2 Humble). |
| `ws_humble/` | Für Positionierung, extrinsische Kalibrierung und Steuerung. | Auf dem Jetson (ROS2 Humble). |
| `ws_jazzy/` | Für die Kartierung. | Nur unter ROS2 Jazzy. |

## Ablauf für einen Flug

### Voraussetzungen

Bevor ein Flug möglich ist, müssen die folgenden Schritte einmalig durchgeführt werden.

1. **Intrinsics bestimmen** – Die Intrinsics der Kameras bestimmen (in `ws_jazzy` beschrieben).
2. **Kartierung durchführen** – Eine Kartierung erstellen (unter ROS2 Jazzy, `ws_jazzy`).

Details finden sich in den README-Dateien der jeweiligen Unterordner.

### Flugbetrieb

Für den eigentlichen Flug sind zwei Schritte nötig.

**1. Extrinsische Kalibrierung**

Bestimmt die Transformationsmatrizen der Kameras in den UAV-Mittelpunkt. Dafür werden zwei Terminals benötigt.

| Terminal | Aufgabe | Workspace |
| --- | --- | --- |
| Terminal 1 | Extrinsische Kalibrierung (Launch) | `ws_humble` |
| Terminal 2 | Isaac ROS – Detektion (Launch) | `isaac_ros` |

**2. Positionierung mit Flug**

Benötigt zusätzlich eine SSH-Verbindung zum Jetson. Es werden drei Terminals geöffnet.

| Terminal | Aufgabe | Workspace |
| --- | --- | --- |
| Terminal 1 | Positionierung (Launch) | `ws_humble` |
| Terminal 2 | Isaac ROS – Detektion (Launch) | `isaac_ros` |
| Terminal 3 | Control | `ws_humble` |

> Es ist sinnvoll, mindestens den Control-Node über eine SSH-Verbindung auf dem Jetson zu starten. Andernfalls ist beim Flug keine Bildschirmausgabe vorhanden und es kann nicht vernünftig gestartet werden.

## Kamerastream prüfen

Bei **jedem Launch-File** muss überprüft werden, welcher Kamerastream verwendet wird. Dieser kann von PC zu PC unterschiedlich sein und ist jeweils in der `gscam_config` des Launch-Files einzustellen.

Beispiel einer `gscam_config`:

```python
'gscam_config': (
    'v4l2src device=/dev/video4 do-timestamp=true ! '
    'image/jpeg,width=1920,height=1080,framerate=15/1 ! '
    'queue leaky=downstream max-size-buffers=1 ! '
    'jpegdec ! '
    'queue leaky=downstream max-size-buffers=1 ! '
    'videoconvert ! '
    'video/x-raw,format=RGB ! '
    'queue leaky=downstream max-size-buffers=1'
),
```

### Verfügbare Streams nachsehen

Die angeschlossenen Kameras und ihre Gerätepfade (`/dev/videoX`) anzeigen:

```bash
v4l2-ctl --list-devices
```

Die unterstützten Formate, Auflösungen und Frameraten eines konkreten Geräts anzeigen (hier `/dev/video4`):

```bash
v4l2-ctl -d /dev/video4 --list-formats-ext
```

Den in der `gscam_config` angegebenen `device`-Pfad entsprechend anpassen.

> `v4l2-ctl` ist Teil des Pakets `v4l-utils` (`sudo apt install v4l-utils`).

## Hinweise

- Die Workspaces sind an unterschiedliche ROS2-Distributionen gebunden: `isaac_ros` und `ws_humble` laufen unter **Humble** (Jetson), `ws_jazzy` unter **Jazzy**.
- Details zur jeweiligen Nutzung finden sich in den README-Dateien der einzelnen Unterordner.
