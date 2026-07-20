#Tagslam selbast 
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def _find_repo_root():
    """Ermittelt die Wurzel des Repositories `Apriltag_UAV_Lokalisierung`.

    Läuft vom Speicherort dieser Datei aus die Verzeichnishierarchie nach oben,
    bis ein Verzeichnis mit dem Namen `Apriltag_UAV_Lokalisierung` gefunden wird,
    und gibt dessen absoluten Pfad zurück. Dadurch funktionieren die Pfade
    unabhängig davon, wo das Repository im Dateisystem abgelegt ist.

    Returns:
        str: Absoluter Pfad zur Repository-Wurzel.

    Raises:
        RuntimeError: Wenn keine passende Repository-Wurzel gefunden wird.
    """
    path = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.basename(path) == 'Apriltag_UAV_Lokalisierung':
            return path
        parent = os.path.dirname(path)
        if parent == path:
            raise RuntimeError(
                'Repository-Wurzel "Apriltag_UAV_Lokalisierung" nicht gefunden '
                '(ausgehend von: {})'.format(os.path.abspath(__file__))
            )
        path = parent


def generate_launch_description():

    repo_root = _find_repo_root()

    ############################################################
    # Pfade
    ############################################################
    settings_dir = os.path.join(repo_root, 'workspaces', 'ws_jazzy', 'Einstellungen')

    input_bag = os.path.join(repo_root, 'workspaces', 'ws_jazzy', 'ros_bag', 'bag_in', 'final_map_OIC_comb')

    output_bag = os.path.join(repo_root, 'workspaces', 'ws_jazzy', 'ros_bag', 'bag_out', 'final_map_OIC_comb_tagslam')

    internal_out_bag = os.path.join(repo_root, 'workspaces', 'ws_jazzy', 'ros_bag', 'out.bag')

    return LaunchDescription([

        ############################################################
        # Alten Output-Bag löschen
        #
        # tagslam_from_bag schreibt bei dir zusätzlich intern nach:
        #   /home/hannes/tagslam_ws_bag/out.bag
        #
        # Deshalb wird dieser Ordner ebenfalls gelöscht.
        ############################################################
        ExecuteProcess(
            cmd=[
                'rm', '-rf',
                output_bag,
                internal_out_bag
            ],
            output='screen'
        ),

        ############################################################
        # TagSLAM-Kartierung aus kombiniertem Bag
        #
        # Input-Bag enthält:
        #   /camera/image_rect
        #   /camera/camera_info_rect
        #   /tagslam/tag_detections
        #   /tf_static
        ############################################################
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='tagslam',
                    executable='tagslam_from_bag',
                    name='tagslam_from_bag',
                    output='screen',
                    parameters=[{
                        'cameras': settings_dir + '/cameras_rect.yaml',
                        'camera_poses': settings_dir + '/camera_poses.yaml',
                        'tagslam_config': settings_dir + '/tagslam_Lauf3.yaml',
                        #'play_rate': 0.3,
                        'in_bag': input_bag,
                        # 'out_bag': output_bag,   # deaktiviert: kein Replay-Bag ausgeben (Karte bleibt unberührt)
                        #'max_number_of_frames': 1000000,
                    }]
                )
            ]
        ),
    ])