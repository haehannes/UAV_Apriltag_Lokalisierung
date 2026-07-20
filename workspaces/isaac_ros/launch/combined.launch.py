### Kombiniertes Launch-File -- laeuft KOMPLETT im Isaac-ROS-Container (Jetson)
#
# Enthaelt:
#   - Bildaufnahme            (gscam, Kamera 1 / Boden)
#   - Entzerrung              (Isaac RectifyNode, Composable)
#   - AprilTag-Detektion      (Isaac AprilTagNode, Composable)
#   - Positionsbestimmung     (vision_pose_uxrce_node)
#
# NICHT enthalten:
#   - Micro-XRCE-DDS-Agent -- laeuft auf dem HOST (kein Start hier).
#
# ---------------------------------------------------------------------------
# PFAD-AUFLOESUNG (WICHTIG):
#   Die Pfade werden RELATIV ZU DIESER LAUNCH-DATEI aufgeloest, nicht mehr
#   ueber den Repo-Namen. Grund: Im Container ist das Repo weg-gemountet
#   (z.B. nach /workspaces/isaac_ros-dev), der Ordnername
#   'Apriltag_UAV_Lokalisierung' taucht im Pfad nicht auf.
#
#   Erwartete Struktur (eine Ebene ueber dem 'launch/'-Ordner):
#     <basis>/
#       launch/combined.launch.py   <- diese Datei
#       Einstellungen/
#         Kamera_intrinsics/
#         Maps/Aufgenommen/
#         Maps/Kalibrierung/
#
#   Im Container also:
#     /workspaces/isaac_ros-dev/launch/combined.launch.py
#     /workspaces/isaac_ros-dev/Einstellungen/...
# ---------------------------------------------------------------------------

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _find_einstellungen():
    """Ermittelt den Pfad zum 'Einstellungen'-Verzeichnis.

    Die Aufloesung erfolgt relativ zum Speicherort dieser Launch-Datei:
    'Einstellungen' wird eine Verzeichnisebene oberhalb des 'launch/'-
    Ordners erwartet. Dadurch ist der Pfad unabhaengig davon, unter
    welchem (ggf. gemounteten) Namen das Repository im Container liegt.

    Returns:
        Path: Absoluter Pfad zum 'Einstellungen'-Verzeichnis.

    Raises:
        RuntimeError: Wenn das erwartete 'Einstellungen'-Verzeichnis
            nicht existiert.
    """
    here = Path(__file__).resolve()
    base = here.parent.parent
    einstellungen = base / 'Einstellungen'
    if not einstellungen.is_dir():
        raise RuntimeError(
            f"'Einstellungen'-Verzeichnis nicht gefunden unter {einstellungen} "
            f"(ausgehend von {here}). Erwartet wird die Struktur "
            f"<basis>/launch/ und <basis>/Einstellungen/."
        )
    return einstellungen


def generate_launch_description():
    """Erzeugt die kombinierte LaunchDescription fuer den Container-Betrieb.

    Startet Bildaufnahme (gscam), Entzerrung und AprilTag-Detektion (Isaac,
    als Composable Nodes) sowie den Vision-Pose-Node. Der Micro-XRCE-DDS-Agent
    wird bewusst NICHT gestartet (laeuft auf dem Host). Alle Datei-Pfade
    werden relativ zu dieser Launch-Datei aufgeloest.

    Returns:
        LaunchDescription: Die vollstaendige Startbeschreibung.
    """

    # ====================================================================
    # Basis-Pfade (relativ zu dieser Launch-Datei)
    # ====================================================================
    einstellungen = _find_einstellungen()
    maps_dir = einstellungen / 'Maps'
    kamera_dir = einstellungen / 'Kamera_intrinsics'

    # Aktive Karte auswaehlen (Zeile ein-/auskommentieren zum Wechseln):
    map_file = str(maps_dir / 'Aufgenommen' / (
        #'test_501_bis_509_a4.yaml'
        #'tagslam_lauf3_Zimmer_24_05.yaml'
        #'bearbeitet_Kleiner.yaml'
        #'OIC_0_bis_12_Lauf2.yaml'
        'OIC_gesamt_teilw_Lauf2.yaml'
        #'OIC_2_groessen_Lauf2.yaml'
    ))

    calibration_file = str(
        maps_dir / 'Kalibrierung' / 'map_Kalibrierung_result.yaml'
    )

    # Kamera 1 (Boden): camera_info-URL fuer 1280x720 Profil
    camera_info_url = (kamera_dir / 'C922' / 'logitech_c922_1280_720.yaml').as_uri()

    return LaunchDescription([

        ############################################################
        # OPTIONAL: v4l2-Kameraeinstellungen VOR gscam setzen.
        #
        # In der Flugumgebung einkommentieren, damit echte 60 FPS
        # reproduzierbar sind. Werte an das Licht vor Ort anpassen:
        #   - auto_exposure=1              -> manueller Belichtungsmodus
        #   - exposure_dynamic_framerate=0 -> Kamera darf FPS NICHT senken
        #   - exposure_time_absolute       -> Einheit 100us; fuer 60 FPS
        #                                     Belichtung < ~16 ms (Wert <= ~150)
        ############################################################
        # ExecuteProcess(
        #     cmd=[
        #         'bash', '-c',
        #         'v4l2-ctl -d /dev/video0 -c auto_exposure=1 && '
        #         'v4l2-ctl -d /dev/video0 -c exposure_dynamic_framerate=0 && '
        #         'v4l2-ctl -d /dev/video0 -c exposure_time_absolute=150'
        #     ],
        #     output='screen',
        # ),

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
        # Kamera 2: Blickrichtung nach vorne  --- DEAKTIVIERT ---
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
        # Isaac-Container: Entzerrung + AprilTag-Detektion.
        # Composable Nodes (intra-process/NITROS, GPU) -- CPU-schonend.
        # Kamera 2 (rectify_2 / apriltag_2) deaktiviert.
        ############################################################
        ComposableNodeContainer(
            name='isaac_ros_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[

                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                    name='rectify_1',
                    parameters=[{
                        'output_width': 1280,
                        'output_height': 720,
                    }],
                    remappings=[
                        ('image_raw',        '/camera_1/image_raw'),
                        ('camera_info',      '/camera_1/camera_info'),
                        ('image_rect',       '/camera_1/image_rect'),
                        ('camera_info_rect', '/camera_1/camera_info_rect'),
                    ],
                ),

                ComposableNode(
                    package='isaac_ros_apriltag',
                    plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                    name='apriltag_1',
                    parameters=[{
                        'family':   'tag36h11',
                        'size':     0.167,
                        'max_tags': 40,
                    }],
                    remappings=[
                        ('image',          '/camera_1/image_rect'),
                        ('camera_info',    '/camera_1/camera_info_rect'),
                        ('tag_detections', '/camera_1/tag_detections'),
                    ],
                ),

                # --- DEAKTIVIERT (Kamera 2) ---
                # ComposableNode(
                #     package='isaac_ros_image_proc',
                #     plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                #     name='rectify_2',
                #     parameters=[{
                #         'output_width': 1280,
                #         'output_height': 720,
                #     }],
                #     remappings=[
                #         ('image_raw',        '/camera_2/image_raw'),
                #         ('camera_info',      '/camera_2/camera_info'),
                #         ('image_rect',       '/camera_2/image_rect'),
                #         ('camera_info_rect', '/camera_2/camera_info_rect'),
                #     ],
                # ),
                # ComposableNode(
                #     package='isaac_ros_apriltag',
                #     plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                #     name='apriltag_2',
                #     parameters=[{
                #         'family':   'tag36h11',
                #         'size':     0.167,
                #         'max_tags': 40,
                #     }],
                #     remappings=[
                #         ('image',          '/camera_2/image_rect'),
                #         ('camera_info',    '/camera_2/camera_info_rect'),
                #         ('tag_detections', '/camera_2/tag_detections'),
                #     ],
                # ),

            ],
        ),

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
        # Vision-Pose-Node (uXRCE-DDS).
        # Originalwerte (60 FPS). Publiziert /fmu/in/vehicle_visual_odometry,
        # das der Agent auf dem HOST an PX4 weiterreicht.
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
                'publish_topic': '/fmu/in/vehicle_visual_odometry',
                'pose_frame': 1,
                'timesync_topic': '/fmu/out/timesync_status',
                'publish_rate_hz': 60.0,

                'map_frame': 'tag_map',

                'max_estimate_age_s': 0.3,

                'min_tags_required': 1,

                # RANSAC-Parameter
                'ransac_base_threshold_m': 0.015,
                'ransac_reference_distance_m': 0.30,
                'ransac_min_inliers_ratio': 0.7,
                'ransac_max_iterations': 20,
                'ransac_refinement_iterations': 2,
                'ransac_max_distance_factor': 50.0,

                # Kovarianz
                'use_covariance': True,
                'covariance_pos_base_sigma_m': 0.012,
                'covariance_rot_base_sigma_rad': 0.0122,
                'covariance_reference_distance_m': 1.5,
                'covariance_reference_tag_size_m': 0.17,

                # Debug: Tag-Karte als static TF publizieren (false = null CPU)
                'publish_tag_map_tf': False,
                'tag_frame_prefix': 'tag_',

                # Isaac-Referenzgroesse: muss der 'size' des AprilTagNode
                # entsprechen (0.167), falls die Karte einen anderen
                # default_tag_size hat. 0 = aus Karte.
                #'isaac_reference_size': 0.167,
            }],
        ),

    ])