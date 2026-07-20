# Workspace `ws_humble` – Positionierung, Kalibrierung und Steuerung

In diesem Workspace werden die extrinsische Kalibrierung, die Positionierung und die Steuerung durchgeführt. Er läuft unter **ROS2 Humble** (auch auf dem Jetson).

## Verzeichnisstruktur

```
ws_humble$ ls
Einstellungen  launch  src
```

| Verzeichnis | Beschreibung |
| --- | --- |
| `Einstellungen/` | Hier werden die Einstellungen zur Positionierung getroffen. |
| `launch/` | Launch-Dateien zum Starten von Kalibrierung, Positionierung und Control-Node. |
| `src/` | Hier ist der Quellcode abgelegt. |

## `Einstellungen/`

```
Einstellungen$ ls
Kamera_intrinsics  Maps
```

| Verzeichnis | Beschreibung |
| --- | --- |
| `Kamera_intrinsics/` | Hier werden die Intrinsics abgespeichert. |
| `Maps/` | Hier liegen die abgespeicherten Karten sowie die Kalibrierdaten. |

## `launch/`

Hier liegen die Launch-Dateien zum Starten der Kalibrierung, der Positionierung sowie des Control-Nodes (der Fernbedienung).

Die Fernbedienung öffnet ein Terminal, in dem alle Funktionen beschrieben sind.

## `src/`

Hier ist der Quellcode abgelegt.
