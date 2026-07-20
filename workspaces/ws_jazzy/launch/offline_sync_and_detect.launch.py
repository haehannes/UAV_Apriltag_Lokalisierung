#Rosbag aufbereiten für Tagslam-Input
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

    raw_bag = os.path.join(repo_root, 'workspaces', 'ws_jazzy', 'ros_bag', 'bag_in', 'final_map_OIC')

    combined_bag = os.path.join(repo_root, 'workspaces', 'ws_jazzy', 'ros_bag', 'bag_in', 'final_map_OIC_comb')

    return LaunchDescription([

        ############################################################
        # Alten kombinierten Bag löschen
        ############################################################
        ExecuteProcess(
            cmd=[
                'rm', '-rf',
                combined_bag
            ],
            output='screen'
        ),

       
        Node(
            package='tagslam',
            executable='sync_and_detect_node',
            name='sync_and_detect_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'cameras': settings_dir + '/cameras_rect.yaml',
                'tagslam_config': settings_dir + '/tagslam_Lauf1.yaml',
            }]
        ),

        ############################################################
        # Neuen kombinierten Bag aufnehmen
        #
        # Dieser Bag enthält danach alles für tagslam_from_bag:
        #
        #   /camera/image_rect
        #   /camera/camera_info_rect
        #   /tagslam/tag_detections
        #   /tf_static
        #
        # /clock wird ebenfalls aufgenommen, weil offline mit
        # simulierter Zeit gearbeitet wird.
        ############################################################
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'record',
                        '-o', combined_bag,

                        '/camera/image_rect',
                        '/camera/camera_info',
                        '/tagslam/tag_detections',
                        '/tf_static',
                        '/clock',
                    ],
                    output='screen'
                )
            ]
        ),

        ############################################################
        # Entzerrten Input-Bag langsam abspielen
        #
        # --clock ist wichtig für use_sim_time.
        #
        # --rate 0.3 gibt der Tag-Erkennung mehr Zeit.
        # Wenn alles stabil läuft, kannst du später 0.5 oder 1.0 testen.
        ############################################################
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'play',
                        raw_bag,
                        '--clock',
                        '--rate', '0.3',
                    ],
                    output='screen'
                )
            ]
        ),
    ])