#Record_rosbag für Tagslam-Input
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def _find_repo_root():
    """Ermittelt die Wurzel des Repositories `UAV_Apriltag_Lokalisierung`.

    Läuft vom Speicherort dieser Datei aus die Verzeichnishierarchie nach oben,
    bis ein Verzeichnis mit dem Namen `UAV_Apriltag_Lokalisierung` gefunden wird,
    und gibt dessen absoluten Pfad zurück. Dadurch funktionieren die Pfade
    unabhängig davon, wo das Repository im Dateisystem abgelegt ist.

    Returns:
        str: Absoluter Pfad zur Repository-Wurzel.

    Raises:
        RuntimeError: Wenn keine passende Repository-Wurzel gefunden wird.
    """
    path = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.basename(path) == 'UAV_Apriltag_Lokalisierung':
            return path
        parent = os.path.dirname(path)
        if parent == path:
            raise RuntimeError(
                'Repository-Wurzel "UAV_Apriltag_Lokalisierung" nicht gefunden '
                '(ausgehend von: {})'.format(os.path.abspath(__file__))
            )
        path = parent


def generate_launch_description():

    repo_root = _find_repo_root()

    ############################################################
    # Pfade
    ############################################################
    settings_dir = os.path.join(repo_root, 'workspaces', 'ws_jazzy', 'Einstellungen')

    camera_info_url = 'file://' + os.path.join(settings_dir, 'camera_1920x1080_c922.yaml')

    output_bag = os.path.join(repo_root, 'workspaces', 'ws_jazzy', 'ros_bag', 'bag_in', 'final_map_OIC')

    return LaunchDescription([

        ############################################################
        # Alten Bag löschen
        ############################################################
        ExecuteProcess(
            cmd=[
                'rm', '-rf',
                output_bag
            ],
            output='screen'
        ),

        ############################################################
        # Kamera mit gscam starten
        #
        # C922 laut v4l2-ctl:
        #   /dev/video8
        #   MJPG
        #   1920x1080
        #   15 fps
        #
        # Diese Variante nutzt CPU-JPEG-Decoding. (nicht auf Jetson)
        # 
        # 
        #
        # Ausgabe:
        #   /camera/image_raw
        #   /camera/camera_info_rect
        ############################################################
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_camera',
            output='screen',
            parameters=[{
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

                'camera_name': 'camera',
                'camera_info_url': camera_info_url,

                # Muss zu deiner cameras.yaml passen.
                # Wenn dort rig_body: camera_optical_frame steht, ist das okay.
                # Wenn dort rig_body: camera steht, dann hier auf 'camera' ändern.
                'frame_id': 'camera',

                'use_gst_timestamps': True,
                'sync_sink': False,
                'preroll': False,
                'image_encoding': 'rgb8',
                'use_sensor_data_qos': False,
            }],
            remappings=[
                ('camera/image_raw', '/camera/image_raw'),
                ('camera/camera_info', '/camera/camera_info'),
            ]
        ),

        ############################################################
        # Echte Bildentzerrung
        #
        # Eingang:
        #   /camera/image_raw
        #   /camera/camera_info_rect
        #
        # Ausgang:
        #   /camera/image_rect
        ############################################################
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_node',
            namespace='camera',
            output='screen',
            remappings=[
                ('image', 'image_raw'),
                #('camera_info', 'camera_info_rect'),
                ('image_rect', 'image_rect'),
            ]
        ),
#
        #############################################################
        ## Bag aufnehmen
        ##
        ## Erst wieder aktivieren, wenn /camera/image_raw und
        ## /camera/image_rect sicher laufen.
        #############################################################
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'record',
                        '-o', output_bag,
    
                        '/camera/image_rect',
                        '/camera/camera_info',
                        '/tf_static',
                    ],
                    output='screen'
                )
            ]
        ),
    ])