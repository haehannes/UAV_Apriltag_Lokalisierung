"""Launch-Datei: publiziert eine Karte als statische TF (und Marker) fuer RViz."""

from pathlib import Path

from launch import LaunchDescription
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
    """Erzeugt die LaunchDescription fuer den Karten-TF-Node.

    Loest den Karten-Pfad repo-relativ auf und startet den map_tf_node,
    der die Karte als statische TF und optionales MarkerArray publiziert.

    Returns:
        LaunchDescription: Die vollstaendige Startbeschreibung.
    """
    repo_root = _find_repo_root()
    einstellungen = repo_root / 'workspaces' / 'ws_jazzy' / 'Einstellungen'
    maps_dir = einstellungen / 'maps'

    # Aktive Karte auswaehlen (Zeile ein-/auskommentieren zum Wechseln):
    map_file = str(maps_dir / (
        #'OIC_0_bis_12_Lauf2.yaml'
        #'OIC_2_groessen_Lauf2.yaml'
        'ground_truth_config3.yaml'
    ))

    node = Node(
        package='apriltag_map_tf',
        executable='map_tf_node',
        name='map_tf_node',
        output='screen',
        parameters=[{
            'config_file': map_file,
            'origin_tag_id': -1,      # -1 = Karte unveraendert; sonst Tag-ID als Ursprung
            'publish_markers': True,
        }],
    )

    return LaunchDescription([node])