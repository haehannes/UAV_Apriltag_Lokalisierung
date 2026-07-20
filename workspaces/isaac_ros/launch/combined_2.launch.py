### Kombiniertes Launch-File -- laeuft KOMPLETT im Isaac-ROS-Container (Jetson)
#
# Enthaelt:
#   - Bildaufnahme            (gscam, Kamera 1 / Boden, RGBA -> kein CPU-videoconvert)
#   - Formatkonvertierung     (Isaac ImageFormatConverter, GPU: rgba8 -> rgb8)
#   - Entzerrung              (Isaac RectifyNode, Composable)
#   - AprilTag-Detektion      (Isaac AprilTagNode, Composable)
#   - Positionsbestimmung     (vision_pose_uxrce_node)
#
# NICHT enthalten:
#   - Micro-XRCE-DDS-Agent -- laeuft auf dem HOST (kein Start hier).
#
# CPU-Weg: gscam gibt RGBA aus (nvvidconv/VIC, kein CPU-videoconvert). Da die
# RectifyNode kein RGBA akzeptiert, macht der Isaac-ImageFormatConverter die
# Umwandlung rgba8->rgb8 auf der GPU (statt CPU-videoconvert).
#
# ---------------------------------------------------------------------------
# PFAD-AUFLOESUNG:
#   Pfade relativ zu dieser Launch-Datei ('Einstellungen' eine Ebene ueber
#   dem 'launch/'-Ordner). Im Container: /workspaces/isaac_ros-dev/Einstellungen/
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

    Startet Bildaufnahme (gscam), GPU-Formatkonvertierung, Entzerrung und
    AprilTag-Detektion (Isaac, als Composable Nodes) sowie den Vision-Pose-
    Node. Der Micro-XRCE-DDS-Agent wird bewusst NICHT gestartet (laeuft auf
    dem Host). Alle Datei-Pfade werden relativ zu dieser Launch-Datei
    aufgeloest.

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
        # In der Flugumgebung einkommentieren fuer reproduzierbare 60 FPS.
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
        # 1280x720 @ 60 FPS. RGBA-Ausgabe -> KEIN CPU-videoconvert.
        # Publiziert /camera_1/image_raw (rgba8).
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
                    'video/x-raw,format=RGBA,framerate=60/1 ! '
                    'queue leaky=downstream max-size-buffers=1'
                ),
                'camera_name': 'camera_1',
                'frame_id': 'camera_1_optical_frame',
                'camera_info_url': camera_info_url,
                'use_gst_timestamps': True,
                'sync_sink': False,
                'preroll': False,
                'image_encoding': 'rgba8',
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
        # (unveraendert deaktiviert)

        ############################################################
        # Isaac-Container: Formatkonverter + Entzerrung + AprilTag.
        # Alle als Composable Nodes (intra-process/NITROS, GPU).
        #
        # Datenfluss Kamera 1:
        #   /camera_1/image_raw (rgba8)
        #     -> format_converter_1 -> /camera_1/image_rgb (rgb8)
        #     -> rectify_1          -> /camera_1/image_rect
        #     -> apriltag_1         -> /camera_1/tag_detections
        # camera_info laeuft unveraendert von gscam zu rectify_1.
        ############################################################
        ComposableNodeContainer(
            name='isaac_ros_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[

                # ---- GPU-Formatkonverter: rgba8 -> rgb8 ----
                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                    name='format_converter_1',
                    parameters=[{
                        'encoding_desired': 'rgb8',
                        'image_width': 1280,
                        'image_height': 720,
                    }],
                    remappings=[
                        ('image_raw', '/camera_1/image_raw'),   # Eingang (rgba8)
                        ('image',     '/camera_1/image_rgb'),   # Ausgang (rgb8)
                    ],
                ),

                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                    name='rectify_1',
                    parameters=[{
                        'output_width': 1280,
                        'output_height': 720,
                    }],
                    remappings=[
                        ('image_raw',        '/camera_1/image_rgb'),   # vom Konverter
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
                # (format_converter_2 / rectify_2 / apriltag_2 analog)

            ],
        ),

        ############################################################
        # Statische TFs fuer Isaac AprilTag.
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

        ############################################################
        # Karte-Frame (optional, fuer RViz-Visualisierung).
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
        # Vision-Pose-Node (uXRCE-DDS). Originalwerte (60 FPS).
        ############################################################
        Node(
            package='drone_vision_pose_publisher',
            executable='vision_pose_uxrce_node',
            name='vision_pose_uxrce_node',
            output='screen',
            parameters=[{
                'map_file': map_file,
                'calibration_file': calibration_file,

                'cam1_topic': '/camera_1/tag_detections',
                'cam1_name': 'camera_1',
                'cam1_weight': 1.0,

                'cam2_weight': 0.0,

                'publish_topic': '/fmu/in/vehicle_visual_odometry',
                'pose_frame': 1,
                'timesync_topic': '/fmu/out/timesync_status',
                'publish_rate_hz': 60.0,

                'map_frame': 'tag_map',

                'max_estimate_age_s': 0.3,

                'min_tags_required': 1,

                'ransac_base_threshold_m': 0.015,
                'ransac_reference_distance_m': 0.30,
                'ransac_min_inliers_ratio': 0.7,
                'ransac_max_iterations': 20,
                'ransac_refinement_iterations': 2,
                'ransac_max_distance_factor': 50.0,

                'use_covariance': True,
                'covariance_pos_base_sigma_m': 0.012,
                'covariance_rot_base_sigma_rad': 0.0122,
                'covariance_reference_distance_m': 1.5,
                'covariance_reference_tag_size_m': 0.17,

                'publish_tag_map_tf': False,
                'tag_frame_prefix': 'tag_',

                #'isaac_reference_size': 0.167,
            }],
        ),

    ])