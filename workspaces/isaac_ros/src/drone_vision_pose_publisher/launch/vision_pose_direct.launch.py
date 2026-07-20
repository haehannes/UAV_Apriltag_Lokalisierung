###Launch File auf meinem Jetson (host)
#
# Schlankes Setup mit einem einzigen Vision-Pose-Node, der:
#   - Tag-Detections von beiden Kameras direkt liest
#   - Drohnen-Pose direkt berechnet (inkl. Kamera->Drohnenmitte)
#   - PoseStamped direkt auf /mavros/vision_pose/pose publiziert
#
# Entfernt gegenueber der Vorversion:
#   - Zwei separate localization_tf_node Instanzen
#   - vision_pose_fusion_node
#   - Diverse TF-Indirektionen (live_pos, cam_down, cam_front, drone_from_cam_*)
#
# Behalten:
#   - mavros_node
#   - Beide gscam-Nodes
#   - Isaac-TF-Verkabelung (tag_map -> isaac -> camera_*_optical_frame),
#     weil Isaac AprilTag das fuer interne TF-Operationen braucht
#   - tag_map -> Karte (fuer Visualisierung in RViz)



####Datei ist nicht mehr gepflegt!!!


import yaml

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    map_file = (
        "/home/hannes/Masterarbeit/workspaces/tagslam_ws_humble/"
        "Einstellungen/maps/Aufgenommen/bearbeitet_Kleiner.yaml"
    )
    

    calibration_file = (
        "/home/hannes/Masterarbeit/workspaces/tagslam_ws_humble/"
        "Einstellungen/maps/Kalibrierung/map_Kalibrierung_result.yaml"
    )

    return LaunchDescription([

        #############################################################
        # MAVROS Verbindung zur echten Drohne
        #############################################################
        Node(
            package='mavros',
            executable='mavros_node',
            #name='mavros',                  # explizit, muss mit YAML übereinstimmen
            output='screen',
            parameters=[
                '/home/hannes/Masterarbeit/workspaces/tagslam_ws_humble/'
                'Einstellungen/Mavros/mavros_pluginlist.yaml',
                {
                    'fcu_url': 'serial:///dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00:2000000',
                    'gcs_url': '',
                    'target_system_id': 1,
                    'target_component_id': 1,
                    'fcu_protocol': 'v2.0',
                }
            ],
        ),

        ############################################################
        # Kamera 1: Blickrichtung Boden
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
                    'image/jpeg,width=640,height=480,framerate=30/1 ! '
                    'queue leaky=downstream max-size-buffers=1 ! '
                    'nvv4l2decoder mjpeg=1 ! '
                    'queue leaky=downstream max-size-buffers=1 ! '
                    'nvvidconv ! '
                    'video/x-raw,format=BGRx ! '
                    'videoconvert ! '
                    'video/x-raw,format=RGB,framerate=30/1'
                ),
                'camera_name': 'camera_1',
                'frame_id': 'camera_1_optical_frame',
                'camera_info_url':
                    'file:///home/hannes/Masterarbeit/workspaces/tagslam_ws_humble/'
                    'Einstellungen/Kamera/C920/down_640_480_C920.yaml',
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
                    'image/jpeg,width=640,height=480,framerate=30/1 ! '
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
                'camera_info_url':
                    'file:///home/hannes/Masterarbeit/workspaces/tagslam_ws_humble/'
                    'Einstellungen/Kamera/C920/front_640_480_C920.yaml',
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

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='isaac_to_camera_2_optical_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'isaac',
                'camera_2_optical_frame',
            ],
        ),

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
        # Der EINE Vision-Pose-Node.
        #
        # Liest direkt:
        #   /camera_1/tag_detections  (Kamera Boden)
        #   /camera_2/tag_detections  (Kamera vorne)
        #
        # Publiziert direkt:
        #   /mavros/vision_pose/pose
        ############################################################
        Node(
            package='drone_vision_pose_publisher',
            executable='vision_pose_direct_node',
            name='vision_pose_direct_node',
            output='screen',
            parameters=[{
                # Karten und Kalibrierung
                'map_file': map_file,
                'calibration_file': calibration_file,

                # Kamera-1 (Boden)
                'cam1_topic': '/camera_1/tag_detections',
                'cam1_name': 'camera_1',
                'cam1_weight': 1.0,

                # Kamera-2 (vorne)
                # Wenn Kamera 2 noch keine Tags in der Karte hat,
                # wird sie automatisch ignoriert (liefert einfach keine
                # gueltigen Schaetzungen). cam2_weight kann auf 1.0
                # bleiben, sobald die Kamera Tags sieht, traegt sie bei.
                'cam2_topic': '/camera_2/tag_detections',
                'cam2_name': 'camera_2',
                'cam2_weight': 0.0,

                # Output
                'publish_topic': '/mavros/vision_pose/pose',
                'map_frame': 'tag_map',
                'publish_rate_hz': 30.0,

                # Wie alt darf eine Kamera-Schaetzung sein, bevor sie
                # nicht mehr in die Fusion eingeht?
                'max_estimate_age_s': 0.3,

                # Tag-Mindestanzahl pro Kamera-Frame.
                # 2 verhindert die Yaw-Ambiguitaet bei nur einem Tag.
                # Falls Setup zu sparse: voruebergehend auf 1 setzen.
                'min_tags_required': 1,

                # Maximale Position-Abweichung einer Tag-Schaetzung
                # vom Median pro Frame (Outlier-Filter)
                'max_position_outlier_m': 0.08,

                # EMA-Glaettung
                # alpha=0.15 entspricht dem alten funktionierenden Setup.
                # alpha=1.0 wuerde komplett ungefiltert publizieren.
                'position_smoothing_alpha': 0.8,
                'rotation_smoothing_alpha': 0.8,
            }],
        ),

    ])