#!/usr/bin/env python3
"""ROS2-Node: publiziert eine Karte als statische TF (und optional als Marker).

Der Loader erkennt das Kartenformat automatisch:

* ``marker_config_*.yaml`` (Schluessel ``markers``): flache Druck-Konfiguration
  mit ``id``, ``x``, ``y``, ``size``. Alle Tags liegen in der Ebene (z = 0) mit
  Identitaets-Rotation.
* TagSLAM-Map (Schluessel ``bodies``, Format wie ``beispiel_map.yaml``): jeder Tag
  hat eine volle Pose (Position und Rodrigues-Rotation) relativ zum Body-Frame.

Fuer jeden Tag wird eine statische Transformation vom Karten-Frame (Default
``map``) zum Tag-Frame ``<tag_prefix><id>`` gesendet. Optional wird zusaetzlich
ein ``MarkerArray`` mit nach Groesse eingefaerbten Quadraten und ID-Text
veroeffentlicht, damit die Karte in RViz sichtbar wird.
"""

import math
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def rodrigues_to_quat(rx, ry, rz):
    """Wandelt einen Rodrigues-Rotationsvektor in ein Quaternion um.

    Args:
        rx, ry, rz: Komponenten des Rotationsvektors (Achse * Winkel, in rad).

    Returns:
        tuple[float, float, float, float]: Quaternion (x, y, z, w).
    """
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(theta / 2.0)
    return (rx / theta * s, ry / theta * s, rz / theta * s, math.cos(theta / 2.0))


def load_map(path):
    """Laedt eine Karte und erkennt das Format automatisch.

    Args:
        path: Pfad zur Kartendatei (Druck-Config oder TagSLAM-Map).

    Returns:
        list[dict]: Normierte Tags, je mit ``id``, ``size``, ``x``, ``y``, ``z``
            und ``quat`` (Quaternion als (x, y, z, w)).

    Raises:
        ValueError: Wenn weder ``markers`` noch ein Body mit ``tags`` vorliegt.
    """
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    # Format 1: flache Druck-Konfiguration
    if cfg.get("markers"):
        tags = []
        for m in cfg["markers"]:
            tags.append({
                "id": int(m["id"]),
                "size": float(m["size"]),
                "x": float(m["x"]),
                "y": float(m["y"]),
                "z": 0.0,
                "quat": (0.0, 0.0, 0.0, 1.0),
            })
        return tags

    # Format 2: TagSLAM-Map (bodies -> tag_map -> tags)
    for body in cfg.get("bodies", []):
        for spec in body.values():
            if isinstance(spec, dict) and spec.get("tags"):
                default_size = float(spec.get("default_tag_size", 0.16) or 0.16)
                tags = []
                for t in spec["tags"]:
                    pose = t.get("pose", {})
                    pos = pose.get("position", {})
                    rot = pose.get("rotation", {})
                    tags.append({
                        "id": int(t["id"]),
                        "size": float(t.get("size", default_size)),
                        "x": float(pos.get("x", 0.0)),
                        "y": float(pos.get("y", 0.0)),
                        "z": float(pos.get("z", 0.0)),
                        "quat": rodrigues_to_quat(
                            float(rot.get("x", 0.0)),
                            float(rot.get("y", 0.0)),
                            float(rot.get("z", 0.0)),
                        ),
                    })
                return tags

    raise ValueError(
        "Unbekanntes Kartenformat: weder 'markers' noch ein Body mit 'tags'."
    )


def color_for_size(size):
    """Waehlt eine RViz-Farbe abhaengig von der Tag-Kantenlaenge.

    Args:
        size: Kantenlaenge des Tags in Metern.

    Returns:
        tuple[float, float, float]: RGB-Werte im Bereich [0, 1].
    """
    if size < 0.2:
        return (0.10, 0.80, 0.30)
    if size < 0.5:
        return (1.00, 0.60, 0.00)
    return (0.90, 0.15, 0.15)


class MapTfNode(Node):
    """Node, der die Kartentags als statische TF und optionale Marker sendet."""

    def __init__(self):
        """Liest Parameter, sendet die statische TF und publiziert optional Marker."""
        super().__init__("map_tf_node")

        self.declare_parameter("config_file", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("tag_prefix", "tag_")
        self.declare_parameter("origin_tag_id", -1)   # -1 = Karte unveraendert
        self.declare_parameter("publish_markers", True)

        config_file = self.get_parameter("config_file").value
        self.map_frame = self.get_parameter("map_frame").value
        self.tag_prefix = self.get_parameter("tag_prefix").value
        origin_tag_id = int(self.get_parameter("origin_tag_id").value)
        publish_markers = bool(self.get_parameter("publish_markers").value)

        if not config_file or not os.path.isfile(config_file):
            self.get_logger().error(f"config_file nicht gefunden: '{config_file}'")
            raise SystemExit(1)

        try:
            tags = load_map(config_file)
        except (ValueError, KeyError) as exc:
            self.get_logger().error(f"Karte konnte nicht geladen werden: {exc}")
            raise SystemExit(1)
        if not tags:
            self.get_logger().error("Karte enthaelt keine Tags.")
            raise SystemExit(1)

        self._apply_origin(tags, origin_tag_id)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(self._build_transforms(tags))
        self.get_logger().info(
            f"{len(tags)} statische TF gesendet (Parent-Frame '{self.map_frame}')."
        )

        if publish_markers:
            qos = QoSProfile(depth=1)
            qos.history = QoSHistoryPolicy.KEEP_LAST
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.marker_pub = self.create_publisher(
                MarkerArray, "apriltag_map_markers", qos
            )
            self.marker_pub.publish(self._build_markers(tags))
            self.get_logger().info(
                "MarkerArray auf Topic 'apriltag_map_markers' veroeffentlicht."
            )

    def _apply_origin(self, tags, origin_tag_id):
        """Verschiebt alle Tags so, dass ``origin_tag_id`` im Ursprung liegt.

        Args:
            tags: Normierte Tag-Liste (wird in-place veraendert).
            origin_tag_id: Tag-ID als neuer Ursprung; -1 laesst die Karte unveraendert.

        Raises:
            SystemExit: Wenn ``origin_tag_id`` nicht in der Karte vorkommt.
        """
        if origin_tag_id < 0:
            return
        ref = next((t for t in tags if t["id"] == origin_tag_id), None)
        if ref is None:
            self.get_logger().error(
                f"origin_tag_id {origin_tag_id} nicht in der Karte vorhanden."
            )
            raise SystemExit(1)
        x0, y0, z0 = ref["x"], ref["y"], ref["z"]
        for t in tags:
            t["x"] -= x0
            t["y"] -= y0
            t["z"] -= z0

    def _build_transforms(self, tags):
        """Erzeugt fuer jeden Tag eine statische TransformStamped.

        Args:
            tags: Normierte Tag-Liste.

        Returns:
            list[TransformStamped]: Transformationen map -> tag_<id>.
        """
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for t in tags:
            qx, qy, qz, qw = t["quat"]
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = f"{self.tag_prefix}{t['id']}"
            tf.transform.translation.x = t["x"]
            tf.transform.translation.y = t["y"]
            tf.transform.translation.z = t["z"]
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            transforms.append(tf)
        return transforms

    def _build_markers(self, tags):
        """Erzeugt ein MarkerArray mit einem Quadrat und einem ID-Text je Tag.

        Args:
            tags: Normierte Tag-Liste.

        Returns:
            MarkerArray: Flache, nach Groesse eingefaerbte Quadrate plus ID-Text.
        """
        arr = MarkerArray()
        for t in tags:
            mid = t["id"]
            size = t["size"]
            qx, qy, qz, qw = t["quat"]
            r, g, b = color_for_size(size)

            square = Marker()
            square.header.frame_id = self.map_frame
            square.ns = "tags"
            square.id = mid
            square.type = Marker.CUBE
            square.action = Marker.ADD
            square.pose.position.x = t["x"]
            square.pose.position.y = t["y"]
            square.pose.position.z = t["z"]
            square.pose.orientation.x = qx
            square.pose.orientation.y = qy
            square.pose.orientation.z = qz
            square.pose.orientation.w = qw
            square.scale.x = size
            square.scale.y = size
            square.scale.z = 0.001
            square.color.r = r
            square.color.g = g
            square.color.b = b
            square.color.a = 0.9
            arr.markers.append(square)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.ns = "labels"
            text.id = mid
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = t["x"]
            text.pose.position.y = t["y"] - size / 2.0 - 0.05
            text.pose.position.z = t["z"]
            text.pose.orientation.w = 1.0
            text.scale.z = 0.08   # Texthoehe in Metern
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = str(mid)
            arr.markers.append(text)
        return arr


def main(args=None):
    """Startet den Node und haelt ihn am Leben, damit die statische TF latcht."""
    rclpy.init(args=args)
    node = MapTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()