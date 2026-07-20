### Kalibrier-Launch-File -- Drone-Camera-Extrinsik (1280x720, C920)
#
# Startet beide Kameras (gscam) und die optischen TFs. Der
# Drone-Camera-Kalibrier-Node bringt die Live-Lokalisierung inzwischen
# selbst mit (keine separaten live_lokalisierung-Nodes mehr). Er abonniert
# direkt die AprilTag-Detektionen je Kamera, berechnet T_map_camera,
# veroeffentlicht diese als liveposition + TF und schreibt die gemittelte
# Extrinsik T_drone_camera_* als YAML.
#
# ---------------------------------------------------------------------------
# PFADE:
# Alle Pfade sind relativ zum Repository-Wurzelverzeichnis
# 'UAV_Apriltag_Lokalisierung' aufgebaut. Die Wurzel wird zur Laufzeit
# ausgehend von der Position dieser Launch-Datei ermittelt (siehe
# _find_repo_root). Dadurch ist das Skript unabhaengig vom absoluten
# Speicherort des Repos und vom aktuellen Arbeitsverzeichnis.
#
# Erwartete Struktur unterhalb der Repo-Wurzel:
#   UAV_Apriltag_Lokalisierung/
#     workspaces/ws_humble/Einstellungen/
#       Kamera_intrinsics/  (Kamera-Kalibrierungen)
#       Maps/Kalibrierung/  (Extrinsik-Kalibrierung: Karte + Ergebnis)
# ---------------------------------------------------------------------------

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _find_repo_root(marker='UAV_Apriltag_Lokalisierung'):
    """Ermittelt das Repository-Wurzelverzeichnis.

    Laeuft ausgehend von der Position dieser Launch-Datei die
    Verzeichnisebenen nach oben, bis ein Verzeichnis mit dem Namen
    ``marker`` gefunden wird. Dieses Verzeichnis dient als Basis fuer
    alle weiteren (repo-relativen) Pfade.

    Args:
        marker: Name des Repository-Wurzelverzeichnisses.

    Returns:
        Path: Absoluter Pfad zum Repository-Wurzelverzeichnis.

    Raises:
        RuntimeError: Wenn in der Elternkette kein Verzeichnis mit dem
            Namen ``marker`` gefunden wird.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        if parent.name == marker:
            return parent
    raise RuntimeError(
        f"Repository-Wurzel '{marker}' wurde ausgehend von "
        f"{here} nicht gefunden."
    )


def generate_launch_description():
    """Erzeugt die LaunchDescription fuer die Drone-Camera-Kalibrierung.

    Startet beide Kameras (gscam), die statischen optischen TFs sowie den
    Drone-Camera-Kalibrier-Node (mit integrierter Live-Lokalisierung).
    Alle Datei-Pfade (Kamera-Intrinsics, Kalibrier-Karte, Ergebnis-YAML)
    werden repo-relativ aus dem Repository-Wurzelverzeichnis aufgeloest.

    Returns:
        LaunchDescription: Die vollstaendige Startbeschreibung.
    """

    # ====================================================================
    # Repo-relative Basis-Pfade
    # ====================================================================
    repo_root = _find_repo_root()
    einstellungen = repo_root / 'workspaces' / 'ws_humble' / 'Einstellungen'
    kamera_dir = einstellungen / 'Kamera_intrinsics'
    kalibrierung_dir = einstellungen / 'Maps' / 'Kalibrierung'

    # Kamera-Intrinsics (camera_info-URLs)
    camera_1_info_url = (kamera_dir / 'C922' / 'logitech_c922_1280_720.yaml').as_uri()
    camera_2_info_url = (kamera_dir / 'C920' / 'front_1280_720_C920.yaml').as_uri()

    # Kalibrier-Karte (mehrfach referenziert) und Ergebnis-YAML
    map_file = str(kalibrierung_dir / 'map_Kalibrierung.yaml')
    result_file = str(kalibrierung_dir / 'map_Kalibrierung_result.yaml')

    return LaunchDescription([

        ############################################################
        # Kamera 1
        # Blickrichtung nach unten
        # Aufloesung: 1280x720 @ 30 FPS (MJPG, Hardware-Decode)
        ############################################################
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_camera_1',
            namespace='camera_1',
            output='screen',
            parameters=[{
                'gscam_config': (
                    'v4l2src device=/dev/video0 do-timestamp=true ! '
                    'image/jpeg,width=1280,height=720,framerate=60/1 ! '
                    'queue leaky=downstream max-size-buffers=1 ! '
                    'nvv4l2decoder mjpeg=1 ! '
                    'queue leaky=downstream max-size-buffers=1 ! '
                    'nvvidconv ! '
                    'video/x-raw,format=BGRx ! '
                    'videoconvert ! '
                    'video/x-raw,format=RGB,framerate=60/1'
                ),
                'camera_name': 'camera_1',
                'frame_id': 'camera_1_optical_frame',
                'camera_info_url': camera_1_info_url,
                'use_gst_timestamps': True,
                'sync_sink': False,
                'preroll': False,
                'image_encoding': 'rgb8',
                'use_sensor_data_qos': False,
            }],
            remappings=[
                ('camera/image_raw', 'image_raw'),
                ('camera/camera_info', 'camera_info'),
                ('camera/image_raw/compressed', 'image_raw/compressed'),
                ('camera/image_raw/compressedDepth', 'image_raw/compressedDepth'),
                ('camera/image_raw/theora', 'image_raw/theora'),
            ],
        ),

        ############################################################
        # Kamera 2
        # Blickrichtung nach vorne
        # Aufloesung: 1280x720 @ 30 FPS (MJPG, Hardware-Decode)
        ############################################################
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_camera_2',
            namespace='camera_2',
            output='screen',
            parameters=[{
                'gscam_config': (
                    'v4l2src device=/dev/video2 do-timestamp=true ! '
                    'image/jpeg,width=1280,height=720,framerate=30/1 ! '
                    'queue leaky=downstream max-size-buffers=1 ! '
                    'nvv4l2decoder mjpeg=1 ! '
                    'queue leaky=downstream max-size-buffers=1 ! '
                    'nvvidconv ! '
                    'video/x-raw,format=BGRx ! '
                    'videoconvert ! '
                    'video/x-raw,format=RGB,framerate=30/1'
                ),
                'camera_name': 'camera_2',
                'frame_id': 'camera_2_optical_frame',
                'camera_info_url': camera_2_info_url,
                'use_gst_timestamps': True,
                'sync_sink': False,
                'preroll': False,
                'image_encoding': 'rgb8',
                'use_sensor_data_qos': False,
            }],
            remappings=[
                ('camera/image_raw', 'image_raw'),
                ('camera/camera_info', 'camera_info'),
                ('camera/image_raw/compressed', 'image_raw/compressed'),
                ('camera/image_raw/compressedDepth', 'image_raw/compressedDepth'),
                ('camera/image_raw/theora', 'image_raw/theora'),
            ],
        ),

        ############################################################
        # TF Kamera 1: camera_1 -> camera_1_optical_frame
        ############################################################
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_1_to_optical_tf',
            arguments=[
                '0', '0', '0',
                '-0.5', '0.5', '-0.5', '0.5',
                'camera_1',
                'camera_1_optical_frame',
            ],
        ),

        ############################################################
        # TF Kamera 2: camera_2 -> camera_2_optical_frame
        ############################################################
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_2_to_optical_tf',
            arguments=[
                '0', '0', '0',
                '-0.5', '0.5', '-0.5', '0.5',
                'camera_2',
                'camera_2_optical_frame',
            ],
        ),

        ############################################################
        # Drone-Camera-Kalibrierung (mit integrierter Lokalisierung)
        #
        # Berechnet je Kamera aus /camera_X/tag_detections selbst
        # T_map_camera (Kernpfad: Isaac->apriltag_ros-Drehung,
        # gewichtete Fusion; ohne Outlierfilter, ohne EMA) und daraus:
        #   T_drone_camera_1 = inv(T_map_drone) * T_map_camera_1
        #   T_drone_camera_2 = inv(T_map_drone) * T_map_camera_2
        #
        # Zusaetzlich werden liveposition (PoseStamped) und TF
        # (tag_map -> camera_X_optical_frame) je Kamera veroeffentlicht.
        # Das Ergebnis wird gemittelt in YAML geschrieben.
        ############################################################
        Node(
            package='drone_camera_calibration',
            executable='drone_camera_calibration_node',
            name='drone_camera_calibration_node',
            output='screen',
            parameters=[{
                'map_file': map_file,
                'output_file': result_file,

                # Tag-ID aus map_Kalibrierung.yaml, die den Drohnenmittelpunkt beschreibt
                'drone_center_tag_id': 100,

                # Anzahl Messwerte pro Kamera
                'samples_required': 100,

                # True: YAML wird nach jedem Sample aktualisiert
                # False: YAML erst schreiben, wenn beide Kameras fertig sind
                'write_every_sample': True,

                # --- Lokalisierungs-Parameter (global) ---
                # Bezugsframe der Karte (Header der liveposition + TF-Parent)
                'parent_frame': 'tag_map',

                # Auch mit nur einem sichtbaren bekannten Tag publishen
                'min_tags_required': 1,

                # Mindestgewicht aus decision_margin (bei Isaac oft 0.0)
                'min_decision_margin_weight': 1.0,

                # --- Kamera 1 (Unten-Kamera) ---
                'camera_1_enabled': True,
                'camera_1_name': 'camera_1',
                'camera_1_detections_topic': '/camera_1/tag_detections',
                'camera_1_pose_topic': '/liveposition_camera_1',
                'camera_1_frame': 'camera_1_optical_frame',

                # --- Kamera 2 (Frontkamera) ---
                'camera_2_enabled': True,
                'camera_2_name': 'camera_2',
                'camera_2_detections_topic': '/camera_2/tag_detections',
                'camera_2_pose_topic': '/liveposition_camera_2',
                'camera_2_frame': 'camera_2_optical_frame',
            }],
        ),

    ])