### Launch File auf dem Jetson (host) -- uXRCE-DDS-Variante
#
# Schlankes Setup mit einem einzigen Vision-Pose-Node, der:
#   - Tag-Detections von der Kamera direkt liest
#   - Drohnen-Pose direkt berechnet (inkl. Kamera->Drohnenmitte)
#   - VehicleOdometry direkt an PX4 publiziert (uXRCE-DDS)
#
# Unterschiede zur MAVROS-Variante:
#   - KEIN mavros_node mehr.
#   - Stattdessen wird der Micro-XRCE-DDS-Agent mitgestartet (ExecuteProcess).
#   - Der Vision-Node ist vision_pose_uxrce_node (statt vision_pose_direct_node)
#     und publiziert auf /fmu/in/vehicle_visual_odometry statt auf
#     /mavros/vision_pose/pose_cov.
#
# Behalten:
#   - Beide gscam-Nodes (Kamera 2 weiterhin deaktiviert)
#   - Isaac-TF-Verkabelung (tag_map -> isaac -> camera_*_optical_frame)
#   - tag_map -> Karte (fuer Visualisierung in RViz)
#
# Transport: UDP ueber Ethernet (PX4-Parameter UXRCE_DDS_CFG = Ethernet).
# Der Agent lauscht auf UDP-Port 8888; PX4 verbindet sich aktiv zur in
# UXRCE_DDS_AG_IP konfigurierten Jetson-IP (192.168.0.10).
# Voraussetzung: Jetson-Ethernet hat eine IP im PX4-Subnetz (192.168.0.x)
# und PX4 ist per Ping erreichbar.
#
# ---------------------------------------------------------------------------
# PFADE:
# Alle Pfade sind relativ zum Repository-Wurzelverzeichnis
# 'Apriltag_UAV_Lokalisierung' aufgebaut. Die Wurzel wird zur Laufzeit
# ausgehend von der Position dieser Launch-Datei ermittelt (siehe
# _find_repo_root). Dadurch ist das Skript unabhaengig vom absoluten
# Speicherort des Repos und vom aktuellen Arbeitsverzeichnis.
#
# Erwartete Struktur unterhalb der Repo-Wurzel:
#   Apriltag_UAV_Lokalisierung/
#     workspaces/ws_humble/Einstellungen/
#       Kamera_intrinsics/  (Kamera-Kalibrierungen)
#       Maps/Aufgenommen/   (TagSLAM-Karten)
#       Maps/Kalibrierung/  (Extrinsik-Kalibrierung)
# ---------------------------------------------------------------------------

import yaml

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def _find_repo_root(marker='Apriltag_UAV_Lokalisierung'):
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
    """Erzeugt die LaunchDescription fuer die uXRCE-DDS-Live-Positionierung.

    Startet den Micro-XRCE-DDS-Agent, die Boden-Kamera (gscam), die
    statischen TFs fuer Isaac AprilTag sowie den vereinigten
    Vision-Pose-Node. Alle Datei-Pfade (Karte, Kalibrierung,
    Kamera-Intrinsics) werden repo-relativ aus dem
    Repository-Wurzelverzeichnis aufgeloest.

    Returns:
        LaunchDescription: Die vollstaendige Startbeschreibung.
    """

    # ====================================================================
    # Repo-relative Basis-Pfade
    # ====================================================================
    repo_root = _find_repo_root()
    einstellungen = repo_root / 'workspaces' / 'ws_humble' / 'Einstellungen'
    maps_dir = einstellungen / 'Maps'
    kamera_dir = einstellungen / 'Kamera_intrinsics'

    # Aktive Karte auswaehlen (Zeile ein-/auskommentieren zum Wechseln):
    map_file = str(maps_dir / 'Aufgenommen' / (
        'ground_truth.yaml'
        #'versuch_1.yaml'
        #'versuch_2.yaml'
        
    ))

    calibration_file = str(
        maps_dir / 'Kalibrierung' / 'map_Kalibrierung_result.yaml'
    )

    # Kamera 1 (Boden): camera_info-URL fuer 1280x720 Profil
    camera_info_url = (kamera_dir / 'C922' / 'logitech_c922_1280_720.yaml').as_uri()

    # ====================================================================
    # Micro-XRCE-DDS-Agent -- UDP ueber Ethernet
    # ====================================================================
    # PX4 (UXRCE_DDS_CFG = Ethernet) verbindet sich aktiv zur Agent-IP
    # (UXRCE_DDS_AG_IP = 192.168.0.10) auf dem UDP-Port unten. Der Agent
    # muss daher nur auf diesem Port lauschen.
    #
    # Pruefe nach dem Start mit:  ros2 topic list | grep /fmu
    uxrce_agent_port = '8888'

    return LaunchDescription([

        #############################################################
        # Micro-XRCE-DDS-Agent (ersetzt die MAVROS-Verbindung)
        # UDP ueber Ethernet.
        #############################################################
        ExecuteProcess(
            cmd=[
                'MicroXRCEAgent', 'udp4',
                '-p', uxrce_agent_port,
            ],
            output='screen',
        ),

        ############################################################
        # Kamera 1: Blickrichtung Boden
        # 1280x720 @ 60 FPS, hardware-beschleunigte MJPEG-Pipeline.
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
                'camera_info_url': camera_info_url,
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
        # Kamera 2: Blickrichtung nach vorne
        # --- DEAKTIVIERT ---
        ############################################################
        # Node(
        #     package='gscam',
        #     executable='gscam_node',
        #     name='gscam_camera_2',
        #     namespace='camera_2',
        #     output='screen',
        #     parameters=[{
        #         'gscam_config': (
        #             'v4l2src device=/dev/video2 do-timestamp=true ! '
        #             'image/jpeg,width=640,height=480,framerate=30/1 ! '
        #             'queue leaky=downstream max-size-buffers=1 ! '
        #             'nvv4l2decoder mjpeg=1 ! '
        #             'queue leaky=downstream max-size-buffers=1 ! '
        #             'nvvidconv ! '
        #             'video/x-raw,format=BGRx ! '
        #             'videoconvert ! '
        #             'video/x-raw,format=RGB,framerate=30/1'
        #         ),
        #         'camera_name': 'camera_2',
        #         'frame_id': 'camera_2_optical_frame',
        #         'camera_info_url':
        #             (kamera_dir / 'C920' / 'front_640_480_C920.yaml').as_uri(),
        #         'use_gst_timestamps': True,
        #         'sync_sink': False,
        #         'preroll': False,
        #         'image_encoding': 'rgb8',
        #         'use_sensor_data_qos': False,
        #     }],
        #     remappings=[
        #         ('camera/image_raw', 'image_raw'),
        #         ('camera/camera_info', 'camera_info'),
        #         ('camera/image_raw/compressed', 'image_raw/compressed'),
        #         ('camera/image_raw/compressedDepth', 'image_raw/compressedDepth'),
        #         ('camera/image_raw/theora', 'image_raw/theora'),
        #     ],
        # ),

        ############################################################
        # Statische TFs fuer Isaac AprilTag.
        # Isaac braucht die TF-Kette tag_map -> isaac -> camera_*_optical_frame.
        ############################################################
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tag_map_to_isaac_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'tag_map',
                'isaac',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='isaac_to_camera_1_optical_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'isaac',
                'camera_1_optical_frame',
            ],
        ),

        # --- DEAKTIVIERT (Kamera 2) ---
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='isaac_to_camera_2_optical_tf',
        #     arguments=[
        #         '0', '0', '0',
        #         '0', '0', '0', '1',
        #         'isaac',
        #         'camera_2_optical_frame',
        #     ],
        # ),

        ############################################################
        # Karte-Frame (optional, fuer RViz-Visualisierung).
        # tag_map -> Karte ist Identitaet, weil die Karte direkt
        # in tag_map-Koordinaten vorliegt.
        ############################################################
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tag_map_to_karte_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'tag_map',
                'Karte',
            ],
        ),

        ############################################################
        # Der EINE Vision-Pose-Node (uXRCE-DDS).
        #
        # Liest direkt:
        #   /camera_1/tag_detections  (Kamera Boden)
        #
        # Publiziert direkt:
        #   /fmu/in/vehicle_visual_odometry  (VehicleOdometry, NED/FRD)
        ############################################################
        Node(
            package='drone_vision_pose_publisher',
            executable='vision_pose_uxrce_node',
            name='vision_pose_uxrce_node',
            output='screen',
            parameters=[{
                # Karten und Kalibrierung
                'map_file': map_file,
                'calibration_file': calibration_file,

                # Kamera-1 (Boden)
                'cam1_topic': '/camera_1/tag_detections',
                'cam1_name': 'camera_1',
                'cam1_weight': 1.0,

                # Kamera-2 (vorne) -- DEAKTIVIERT
                'cam2_weight': 0.0,

                # ----- uXRCE-Ausgabe -----
                # VehicleOdometry an PX4. pose_frame=1 -> NED (passend zu
                # einer ENU-Karte nach der internen ENU->NED-Konvertierung).
                'publish_topic': '/fmu/in/vehicle_visual_odometry',
                'pose_frame': 1,
                'timesync_topic': '/fmu/out/timesync_status',
                'publish_rate_hz': 50.0,

                # map_frame ist bei VehicleOdometry ohne Bedeutung (kein
                # frame_id-String), bleibt nur fuer interne Logs.
                'map_frame': 'tag_map',

                # Wie alt darf eine Kamera-Schaetzung sein?
                'max_estimate_age_s': 0.3,

                # Tag-Mindestanzahl pro Kamera-Frame.
                'min_tags_required': 1,

                # RANSAC-Parameter
                'ransac_base_threshold_m': 0.06, #0.015,
                'ransac_reference_distance_m': 1.0, #0.3, #0.30, #2m hat nichts so toll funktioniert.
                'ransac_min_inliers_ratio': 0.70,#0.6, #0.7,
                'ransac_max_iterations': 20,
                'ransac_refinement_iterations': 2,
                'ransac_max_distance_factor': 4.0,#hat sich gelöst... so funktioniert es besser...#4.0, #50.0, #dieser Wert macht bei kleinen Konfigurationen etwas Probleme.

                # Kovarianz: VehicleOdometry traegt die Varianzfelder.
                'use_covariance': False,
                'covariance_pos_base_sigma_m': 0.012,
                'covariance_rot_base_sigma_rad': 0.02, #0.0122,
                'covariance_reference_distance_m': 1.0, #1.5,
                'covariance_reference_tag_size_m': 0.17,

                #EMA-Glättung
                'position_smoothing_alpha': 0.2,    #vorher 0.15
                'rotation_smoothing_alpha': 0.2,    #vorher 0.15
                # Debug: Tag-Karte als static TF publizieren (false = null CPU)
                'publish_tag_map_tf': False,
                'tag_frame_prefix': 'tag_',

                # Isaac-Referenzgroesse nur setzen, wenn isaac_ros_apriltag
                # eine andere size hat als die Karte:
                'isaac_reference_size': 0.170,
            }],
        ),

    ])

    #mit ekf2_baro = 0 springt er immerwieder hoch und runter. 




## Karten und Kalibrierung
#                'map_file': map_file,
#                'calibration_file': calibration_file,
#
#                # Kamera-1 (Boden)
#                'cam1_topic': '/camera_1/tag_detections',
#                'cam1_name': 'camera_1',
#                'cam1_weight': 1.0,
#
#                # Kamera-2 (vorne) -- DEAKTIVIERT
#                'cam2_weight': 0.0, #deaktiviert--> Karte ist in dem aktuellen Fall nur am Boden --> Frontkamera kann nichts sehen.
#
#                # ----- uXRCE-Ausgabe -----
#                # VehicleOdometry an PX4. pose_frame=1 -> NED (passend zu
#                # einer ENU-Karte nach der internen ENU->NED-Konvertierung).
#                'publish_topic': '/fmu/in/vehicle_visual_odometry',
#                'pose_frame': 1,
#                'timesync_topic': '/fmu/out/timesync_status',
#                'publish_rate_hz': 50.0, #50 hz dass konstant 50 hz kommen können, auch wenn kamera mal nicht mitkommt.
#
#                # map_frame ist bei VehicleOdometry ohne Bedeutung (kein
#                # frame_id-String), bleibt nur fuer interne Logs.
#                'map_frame': 'tag_map',
#
#                # Wie alt darf eine Kamera-Schaetzung sein?
#                'max_estimate_age_s': 0.03, #nicht älter als 33 hz
#
#                # Tag-Mindestanzahl pro Kamera-Frame.
#                'min_tags_required': 1, #beim start ist nur ein marker sichtbar
#    # RANSAC-Parameter
#                'ransac_base_threshold_m': 0.06, #hier eine erklärung, warum der Wert gut passt.
#                'ransac_reference_distance_m': 1.5, #dieser Wert hat iterativ am beseten funktioniert. bei einer angestrebten Flughöhe von 2m ist die kamera uf einer höhe von etwa 1,7m
#                'ransac_min_inliers_ratio': 0.70,#0Hier erklärung, warum wert gut passt --> machst du
#                'ransac_max_iterations': 20, #Hier erklärung, warum wert gut passt. --> machst du
#                'ransac_refinement_iterations': 2, #hier erklärung, warum wert gut passt. --> machst du
#                'ransac_max_distance_factor': 4.0,#Hier erklärung, warum wert gut passt. --> machst du
#
#                # Kovarianz: VehicleOdometry traegt die Varianzfelder.
#                'use_covariance': False,    #Kovarianz komlett weg lassen
#                'covariance_pos_base_sigma_m': 0.012,
#                'covariance_rot_base_sigma_rad': 0.02, 
#                'covariance_reference_distance_m': 1.0, 
#                'covariance_reference_tag_size_m': 0.17,
#
#                #EMA-Glättung
#                'position_smoothing_alpha': 0.3,    #iterativ, hat gut funktioniert.
#                'rotation_smoothing_alpha': 0.3,    #iterativ, hat gut funktioniert.
#                # Debug: Tag-Karte als static TF publizieren (false = null CPU)
#                'publish_tag_map_tf': False,
#                'tag_frame_prefix': 'tag_',
#
#                # Isaac-Referenzgroesse nur setzen, wenn isaac_ros_apriltag
#                # eine andere size hat als die Karte:
#                'isaac_reference_size': 0.170, #so ist es eben gesetzt
#    
#Die Kamera wird mit 1280,height=720,framerate=60/1 betrieben.