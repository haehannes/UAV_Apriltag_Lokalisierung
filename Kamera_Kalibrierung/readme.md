# Kamera-Kalibrierung

Dieser Ordner enthält die Materialien und Ergebnisse zur intrinsischen Kalibrierung der Logitech-Kameras.

## Verzeichnisstruktur

```
$ ls
chessboard-to-print.pdf
'Din A3-Kalibrierung Logitech C922'
'Din A3-Kalibrierung Logitech C920'
```

| Element | Beschreibung |
| --- | --- |
| `chessboard-to-print.pdf` | Schachbrettmuster zur Kamera-Kalibrierung. |
| `Din A3-Kalibrierung Logitech C922` | Intrinsics für eine Logitech C922. |
| `Din A3-Kalibrierung Logitech C920` | Intrinsics für eine Logitech C920. **Vorsicht:** Es handelt sich um zwei verschiedene Kameras. |

## Benötigte Pakete

Die Kalibrierung setzt eine funktionierende ROS2-Installation (Humble) voraus. Zusätzlich werden folgende Pakete benötigt:

```bash
# ROS2-Pakete
sudo apt install ros-humble-gscam ros-humble-camera-calibration

# GStreamer-Plugins (für jpegdec / v4l2src)
sudo apt install \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev
```

> Passe `humble` an deine ROS2-Distribution an, falls du eine andere Version verwendest.

## Kalibrierung starten

### 1. Kamera-Stream über gscam bereitstellen (1080p)

```bash
ros2 run gscam gscam_node --ros-args \
  -p gscam_config:="v4l2src device=/dev/video4 do-timestamp=true ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=RGB" \
  -p frame_id:=camera_optical_frame \
  -p camera_name:=camera
```

> Prüfe vorher mit `ls /dev/video*` bzw. `v4l2-ctl --list-devices`, unter welchem Gerät die gewünschte Kamera liegt, und passe `device=/dev/video4` entsprechend an.

### 2. Kamera kalibrieren

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x6 \
  --square 0.032 \
  --no-service-check \
  --ros-args \
  --remap image:=/image_raw
```

### Parameter

- **`--size 9x6`** – Anzahl der inneren Ecken des Schachbrettmusters (Spalten × Zeilen).
- **`--square 0.032`** – Kantenlänge eines Schachbrettfelds in Metern.
- **`--remap image:=/image_raw`** – Bindet den von gscam veröffentlichten Bildtopic an den Kalibrator.

## Hinweise

- Das Schachbrett (`chessboard-to-print.pdf`) sollte plan auf einer festen Unterlage montiert werden, damit die Messung nicht durch Wellen im Papier verfälscht wird.
- Für einen vollständigen Kalibrierlauf das Muster in verschiedenen Abständen, Winkeln und Bildbereichen zeigen, bis im Kalibrator die Balken **X**, **Y**, **Size** und **Skew** ausreichend gefüllt sind.
- C920 und C922 getrennt kalibrieren und die Ergebnisse eindeutig zuordnen — die beiden Modelle unterscheiden sich.
