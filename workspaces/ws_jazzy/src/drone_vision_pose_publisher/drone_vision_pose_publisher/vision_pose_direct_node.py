#!/usr/bin/env python3
# vision_pose_direct_node.py
#
# Schlanker Vision-Pose-Node fuer GPS-denied Indoor-Flug.
#
# Pipeline (so kurz wie moeglich):
#
#   /camera_1/tag_detections ─┐
#                             ├─► dieser Node ─► /mavros/vision_pose/pose
#   /camera_2/tag_detections ─┘
#
# Was der Node macht:
#   1) Laedt die TagSLAM-Karte einmalig beim Start.
#   2) Laedt die Kamera-zu-Drohnenmittelpunkt-Transformationen
#      einmalig beim Start (aus deiner Kalibrierungs-YAML).
#   3) Pro eingehender Tag-Detection:
#         - Berechnet die Pose der Drohnenmitte in der Karte.
#         - Speichert sie zwischen.
#   4) Per Timer (publish_rate_hz):
#         - Fusioniert die juengsten Schaetzungen beider Kameras.
#         - Publiziert direkt PoseStamped auf /mavros/vision_pose/pose.
#
# Was der Node bewusst NICHT macht:
#   - Keine TF-Veroeffentlichung (waere zusaetzlicher Overhead).
#   - Keine TF-Subscription (vermeidet Latenz durch tf-Buffer).
#   - Keine separaten Localization-Nodes pro Kamera.
#
# CPU-/Latenz-Optimierungen:
#   - Reine Python-Tuple-Arithmetik fuer Quaternionen statt numpy
#     (vermeidet numpy-Overhead bei wenigen Operationen pro Frame).
#   - Subscriber mit Queue-Size 1 (alte Detections sind wertlos).
#   - Timer-basiertes Publish, damit /mavros/vision_pose/pose eine
#     konstante Rate hat (PX4 EKF2 mag das).
#
# Outlier-Filter:
#   Statt eines achsenweisen Median-Filters wird ein adaptiver
#   RANSAC-Filter verwendet. Vorteile:
#     - Robust gegen bimodale Verteilungen (z.B. wenn 2 Tags eine
#       Pose A und 3 Tags eine Pose B vorschlagen - der Median
#       wuerde dann zwischen A und B mitteln; RANSAC waehlt das
#       groessere konsistente Cluster aus).
#     - Distanz-adaptive Inlier-Schwelle: bei groesserer Tag-
#       Entfernung wird die Schwelle automatisch groesser, weil
#       PnP-Rauschen mit d^2 skaliert. Wichtig fuer den Flug, wo
#       die Drohne in unterschiedlichen Hoehen ueber den Tags ist.
#     - Fallback fuer wenige Tags (1-2): kein RANSAC moeglich,
#       trotzdem sinnvolle Strategie.
#
# Output-Format:
#   Wahlweise PoseStamped (/mavros/vision_pose/pose) oder
#   PoseWithCovarianceStamped (/mavros/vision_pose/pose_cov).
#   Bei use_covariance=True wird pro Publish eine Kovarianzmatrix
#   geschaetzt aus:
#     - Anzahl Inlier-Tags (mehr Tags = sicherer, ~1/sqrt(n))
#     - Mittlere Tag-Distanz (naeher = sicherer, ~d^2)
#     - Tag-Groesse pro Tag (groessere Tags = sicherer, ~s_ref/s)
#   Pro-Tag-Varianzen werden via inverse-Varianz-Gewichtung kombiniert
#   (Maximum-Likelihood-Schaetzung): grosse, nahe Tags dominieren die
#   Gesamt-Schaetzung, kleine, weit entfernte Tags tragen weniger bei.
#   PX4 EKF2 kann damit die Vision-Pose pro Frame adaptiv gewichten.
#   Wichtig: PX4-Parameter EKF2_EV_NOISE_MD muss auf 1 gesetzt sein,
#   damit die Kovarianz aus der Message ueberhaupt verwendet wird.

import math
import os
import random
import yaml
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


# =========================================================================
# Quaternion-/Transform-Hilfsfunktionen (Python-Tuple-Stil, CPU-effizient)
# =========================================================================

def quat_normalize(q):
    """Normalisiert ein Quaternion (x, y, z, w) auf Einheitslaenge."""
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)


def quat_conjugate(q):
    """Quaternion-Konjugation."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(q1, q2):
    """Quaternion-Multiplikation im ROS-Format (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def rotate_vector(q, v):
    """Rotiert einen 3D-Vektor v mit einem Quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    qr = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return (qr[0], qr[1], qr[2])


def transform_inverse(t, q):
    """Inverse einer Transformation (Translation + Rotation)."""
    q_inv = quat_conjugate(q)
    t_inv = rotate_vector(q_inv, (-t[0], -t[1], -t[2]))
    return t_inv, q_inv


def transform_multiply(t1, q1, t2, q2):
    """
    Komposition zweier Transformationen:
        T_13 = T_12 * T_23
    """
    t2_rot = rotate_vector(q1, t2)
    t = (
        t1[0] + t2_rot[0],
        t1[1] + t2_rot[1],
        t1[2] + t2_rot[2],
    )
    q = quat_multiply(q1, q2)
    return t, quat_normalize(q)


def rot_matrix_to_quat(R):
    """
    Wandelt eine 3x3-Rotationsmatrix in ein Quaternion (x, y, z, w).
    Wird einmalig beim Laden der Kamera-Kalibrierung benoetigt.
    """
    tr = R[0][0] + R[1][1] + R[2][2]

    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s

    return quat_normalize((x, y, z, w))


def apply_apriltag_ros_rotation(q_isaac):
    """
    Isaac-AprilTag-Pose auf die Konvention bringen, mit der die TagSLAM-
    Karte erstellt wurde.

    Mathematisch: q_out = q_isaac * RotX(pi)
    RotX(pi) entspricht (1, 0, 0, 0) im (x, y, z, w)-Format.

    Diese Korrektur war Teil deines funktionierenden Setups und muss
    beibehalten werden, sonst stimmt die Pose nicht zur Karte.
    """
    x, y, z, w = q_isaac
    return quat_normalize((w, z, -y, -x))


def rpy_to_quat(roll, pitch, yaw):
    """
    Roll/Pitch/Yaw (Radiant) in Quaternion (x, y, z, w).
    Wird fuer TagSLAM-Karten ohne explizites w im Rotation-Eintrag genutzt.
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return quat_normalize((qx, qy, qz, qw))


def pose_to_transform(pose):
    """
    Liest pose: {position: ..., rotation: ...} aus YAML.
    Unterstuetzt Quaternion (mit w) und RPY (ohne w).
    """
    if pose is None:
        pose = {}

    p = pose.get("position", {})
    r = pose.get("rotation", {})

    t = (
        float(p.get("x", 0.0)),
        float(p.get("y", 0.0)),
        float(p.get("z", 0.0)),
    )

    if "w" in r:
        q = quat_normalize((
            float(r.get("x", 0.0)),
            float(r.get("y", 0.0)),
            float(r.get("z", 0.0)),
            float(r.get("w", 1.0)),
        ))
    else:
        q = rpy_to_quat(
            float(r.get("x", 0.0)),
            float(r.get("y", 0.0)),
            float(r.get("z", 0.0)),
        )

    return t, q


# =========================================================================
# Hauptknoten
# =========================================================================

class VisionPoseDirectNode(Node):
    """
    Vereinigter Vision-Pose-Node: nimmt Tag-Detections von beiden Kameras
    und publiziert direkt PoseStamped auf /mavros/vision_pose/pose.
    """

    def __init__(self):
        super().__init__("vision_pose_direct_node")

        # ------------------------------------------------------------
        # Parameter
        # ------------------------------------------------------------
        # Pflicht:
        self.declare_parameter("map_file", "")
        self.declare_parameter("calibration_file", "")

        # Topic-Namen
        self.declare_parameter("cam1_topic", "/camera_1/tag_detections")
        self.declare_parameter("cam1_name", "camera_1")
        self.declare_parameter("cam2_topic", "/camera_2/tag_detections")
        self.declare_parameter("cam2_name", "camera_2")
        self.declare_parameter("publish_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("map_frame", "tag_map")

        # Publish-Rate
        self.declare_parameter("publish_rate_hz", 30.0)

        # Maximales Alter einer Kamera-Schaetzung, bevor sie verworfen wird.
        # Wichtig: wenn nur eine Kamera valide ist, soll die andere nicht
        # uralt mit reinfusioniert werden.
        self.declare_parameter("max_estimate_age_s", 0.3)

        # Gewichtung der Kameras in der Fusion.
        self.declare_parameter("cam1_weight", 1.0)
        self.declare_parameter("cam2_weight", 1.0)

        # EMA-Filter (alpha=1.0 -> kein Filter, alpha=0.15 -> wie alter Node)
        self.declare_parameter("position_smoothing_alpha", 0.15)
        self.declare_parameter("rotation_smoothing_alpha", 0.15)

        # Tag-Auswahl
        # Mindestens so viele Tags pro Kamera-Frame muessen sichtbar sein,
        # damit der Frame zur Fusion verwendet wird.
        self.declare_parameter("min_tags_required", 2)

        # ------------------------------------------------------------
        # Outlier-Filter (RANSAC)
        # ------------------------------------------------------------
        # Inlier-Schwelle bei der Referenzdistanz (typisch 30 cm).
        # Sollte etwa 2-3x die PnP-Standardabweichung sein.
        # Bei 4 cm Tags @ 30 cm Distanz: ~5 mm Sigma -> 15 mm Schwelle.
        self.declare_parameter("ransac_base_threshold_m", 0.015)

        # Referenzdistanz, bei der ransac_base_threshold_m gilt.
        # Bei groesserer Distanz wird die Schwelle mit (d / ref_d)^2
        # hochskaliert (PnP-Rauschen ~ d^2).
        self.declare_parameter("ransac_reference_distance_m", 0.30)

        # Mindestanteil Inlier am Gesamt-Tag-Set, damit RANSAC ueber-
        # haupt einen gueltigen Konsens meldet. 0.6 = 60% der Tags
        # muessen konsistent sein.
        self.declare_parameter("ransac_min_inliers_ratio", 0.6)

        # Maximal-Anzahl RANSAC-Iterationen. Bei n<=max_iterations
        # wird stattdessen exhaustive RANSAC verwendet (jeder Tag
        # einmal als Hypothese - deterministisch).
        self.declare_parameter("ransac_max_iterations", 20)

        # Refinement-Iterationen nach RANSAC: mit den Inliern wird
        # der Schwerpunkt berechnet und nochmal gefiltert. Bringt
        # i.d.R. ~10-20% genauere Endschaetzung. 0 = aus.
        self.declare_parameter("ransac_refinement_iterations", 2)

        # Obergrenze fuer den adaptiven Distanzfaktor. Schuetzt vor
        # Runaway bei sehr grossen Distanzen. Bei Faktor 50 wuerde
        # die Schwelle z.B. von 15 mm auf 750 mm hochskalieren -
        # mehr ist selten sinnvoll.
        self.declare_parameter("ransac_max_distance_factor", 50.0)

        # ------------------------------------------------------------
        # Kovarianz-Schaetzung (fuer PoseWithCovarianceStamped)
        # ------------------------------------------------------------
        # Wenn True, wird PoseWithCovarianceStamped auf publish_topic
        # publiziert (typisch /mavros/vision_pose/pose_cov). Wenn
        # False, wird PoseStamped publiziert (typisch /mavros/
        # vision_pose/pose). Beim Wechsel muss publish_topic in der
        # Launch-Datei angepasst werden, weil MAVROS unterschiedliche
        # Topic-Namen fuer die beiden Message-Typen vorgibt.
        self.declare_parameter("use_covariance", True)

        # ------------------------------------------------------------
        # Basis-Sigma fuer die Pro-Tag-Varianz.
        # ------------------------------------------------------------
        # Diese Werte gelten unter Referenzbedingungen:
        #   - Tag-Groesse = covariance_reference_tag_size_m
        #   - Tag-Distanz = covariance_reference_distance_m
        #   - 1 Tag (keine Multi-Tag-Mittelung)
        #
        # AprilTag liefert bei einem 17 cm Tag aus 1.5 m Distanz
        # typisch ~10-15 mm Positions-Genauigkeit und ~0.5-1 Grad
        # Orientierungs-Genauigkeit. Wir setzen konservative
        # Default-Werte und referenzieren auf diese Bedingungen,
        # weil das der erwartete Flug-Use-Case ist.
        # ------------------------------------------------------------

        # Basis-Standardabweichung fuer Position in Metern.
        # 12 mm bei 17 cm Tag @ 1.5 m Distanz - leicht konservativ.
        # Wenn deine Drohne im Flug zappelt: Wert erhoehen.
        # Wenn sie traege ist: Wert verringern.
        self.declare_parameter("covariance_pos_base_sigma_m", 0.012)

        # Basis-Standardabweichung fuer Rotation in Radiant.
        # 0.7 Grad = 0.0122 rad bei 17 cm Tag @ 1.5 m Distanz.
        self.declare_parameter("covariance_rot_base_sigma_rad", 0.0122)

        # Referenzdistanz, bei der die Basis-Sigmas gelten.
        # Mittelpunkt deines erwarteten Flug-Hoehenbereichs (1.2-2.5 m).
        self.declare_parameter("covariance_reference_distance_m", 1.5)

        # Referenz-Tag-Groesse, bei der die Basis-Sigmas gelten.
        # Auf deine Haupt-Tag-Groesse setzen. Kleinere Tags bekommen
        # automatisch hoehere Sigma, groessere niedrigere.
        # Bei Multi-Size-Setups gilt: Referenzgroesse setzt das
        # Bezugssystem, nicht die "beste" Groesse.
        self.declare_parameter("covariance_reference_tag_size_m", 0.17)

        # Obergrenze fuer den adaptiven Distanzfaktor in der
        # Kovarianz-Schaetzung. Verhindert Sigma-Runaway bei
        # extremer Distanz. Faktor 50 entspricht 5x Referenzdistanz.
        self.declare_parameter("covariance_max_distance_factor", 50.0)

        # Obergrenze fuer den adaptiven Tag-Groessen-Faktor.
        # Schuetzt vor unrealistisch grossem Sigma bei sehr kleinen
        # Tags. Faktor 10 entspricht einem Tag, der 10x kleiner ist
        # als die Referenzgroesse (z.B. 1.7 cm bei 17 cm Referenz).
        self.declare_parameter("covariance_max_size_factor", 10.0)

        # ------------------------------------------------------------
        # Multi-Size-Setup
        # ------------------------------------------------------------
        # Isaac AprilTagNode kennt nur EINE 'size' fuer alle Tags.
        # Tags mit abweichender realer Groesse liefern dadurch eine
        # falsch skalierte Translation. Wir korrigieren das im Hot
        # Path mit Faktor   real_size / isaac_reference_size.
        #
        # 0.0 (Default) bedeutet: nimm 'default_tag_size' aus der Map.
        # Wenn ein Wert > 0 gesetzt ist, gilt der explizit als
        # Referenz - z.B. wenn Isaac mit 0.162 laeuft, die Map aber
        # default_tag_size 0.167 hat.
        # ------------------------------------------------------------
        self.declare_parameter("isaac_reference_size", 0.0)

        # ------------------------------------------------------------
        # Debug: Tag-Karte als statische TF publizieren.
        # Wenn False, wird kein Broadcaster angelegt -> null CPU-Last.
        # Static-TF wird einmalig auf /tf_static gelatched, danach
        # verursacht es ohnehin keinen Overhead mehr.
        # ------------------------------------------------------------
        self.declare_parameter("publish_tag_map_tf", False)
        self.declare_parameter("tag_frame_prefix", "tag_")

        # ------------------------------------------------------------
        # Parameter auslesen
        # ------------------------------------------------------------
        self.map_file = self.get_parameter("map_file").value
        self.calibration_file = self.get_parameter("calibration_file").value

        self.cam1_topic = self.get_parameter("cam1_topic").value
        self.cam1_name = self.get_parameter("cam1_name").value
        self.cam2_topic = self.get_parameter("cam2_topic").value
        self.cam2_name = self.get_parameter("cam2_name").value

        self.publish_topic = self.get_parameter("publish_topic").value
        self.map_frame = self.get_parameter("map_frame").value

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.max_estimate_age_s = float(
            self.get_parameter("max_estimate_age_s").value
        )

        self.cam1_weight = float(self.get_parameter("cam1_weight").value)
        self.cam2_weight = float(self.get_parameter("cam2_weight").value)

        self.position_alpha = float(
            self.get_parameter("position_smoothing_alpha").value
        )
        self.rotation_alpha = float(
            self.get_parameter("rotation_smoothing_alpha").value
        )

        self.min_tags_required = int(
            self.get_parameter("min_tags_required").value
        )

        # RANSAC-Parameter
        self.ransac_base_threshold_m = float(
            self.get_parameter("ransac_base_threshold_m").value
        )
        self.ransac_reference_distance_m = float(
            self.get_parameter("ransac_reference_distance_m").value
        )
        self.ransac_min_inliers_ratio = float(
            self.get_parameter("ransac_min_inliers_ratio").value
        )
        self.ransac_max_iterations = int(
            self.get_parameter("ransac_max_iterations").value
        )
        self.ransac_refinement_iterations = int(
            self.get_parameter("ransac_refinement_iterations").value
        )
        self.ransac_max_distance_factor = float(
            self.get_parameter("ransac_max_distance_factor").value
        )

        # Kovarianz-Parameter
        self.use_covariance = bool(
            self.get_parameter("use_covariance").value
        )
        self.cov_pos_base_sigma = float(
            self.get_parameter("covariance_pos_base_sigma_m").value
        )
        self.cov_rot_base_sigma = float(
            self.get_parameter("covariance_rot_base_sigma_rad").value
        )
        self.cov_reference_distance = float(
            self.get_parameter("covariance_reference_distance_m").value
        )
        self.cov_reference_tag_size = float(
            self.get_parameter("covariance_reference_tag_size_m").value
        )
        self.cov_max_distance_factor = float(
            self.get_parameter("covariance_max_distance_factor").value
        )
        self.cov_max_size_factor = float(
            self.get_parameter("covariance_max_size_factor").value
        )

        # ------------------------------------------------------------
        # Karten und Kalibrierungen laden
        # ------------------------------------------------------------
        self.map_tags = self.load_map(self.map_file)

        # ------------------------------------------------------------
        # Skalierungsfaktoren fuer Multi-Size-Tags vorberechnen.
        #
        # Hintergrund: Isaac AprilTagNode bekommt eine globale 'size'
        # mitgegeben und nutzt sie fuer die PnP-Pose-Schaetzung. Die
        # Translation skaliert linear mit dieser size. Wenn der echte
        # Tag eine andere Groesse hat, ist die Translation entsprechend
        # falsch -- die Rotation aber nicht.
        #
        # Wir berechnen pro Tag den Faktor real_size / isaac_reference.
        # Im Hot Path multiplizieren wir die Isaac-Translation damit.
        # ------------------------------------------------------------
        isaac_ref = float(
            self.get_parameter("isaac_reference_size").value
        )
        if isaac_ref <= 0.0:
            # Kein expliziter Wert -> nimm default_tag_size aus der Map.
            isaac_ref = self.map_default_tag_size
        if isaac_ref <= 0.0:
            raise ValueError(
                "Keine Isaac-Referenzgroesse gefunden. "
                "Entweder Parameter 'isaac_reference_size' setzen oder "
                "in der Map 'default_tag_size' definieren."
            )
        self.isaac_reference_size = isaac_ref

        # Dict {tag_id: scale}. Wir speichern bewusst NUR die Tags
        # mit abweichender Groesse - das spart Speicher und macht den
        # Hot Path noch einen Tick schneller (dict.get(...,1.0) gibt
        # fuer "Default-Tags" sofort 1.0 zurueck).
        self.tag_scale = {}
        for tag_id, real_size in self.map_tag_sizes.items():
            if real_size <= 0.0:
                # Tag ohne sinnvolle Groesse -> ueberspringen
                # (wuerde sonst Division durch 0 erzeugen oder
                # falsche Skalierung).
                continue
            s = real_size / self.isaac_reference_size
            # Floats vergleichen wir mit kleinem Epsilon, sonst
            # speichern wir Faktoren wie 1.0000000001 unnoetig.
            if abs(s - 1.0) > 1e-6:
                self.tag_scale[tag_id] = s

        # Transformationen Kamera -> Drohnenmittelpunkt aus YAML laden.
        # Die YAML enthaelt drone_center -> camera_X, wir brauchen die
        # Inverse: camera_X -> drone_center.
        self.t_cam_drone_1, self.q_cam_drone_1 = self.load_camera_to_drone(
            self.calibration_file, self.cam1_name
        )
        self.t_cam_drone_2, self.q_cam_drone_2 = self.load_camera_to_drone(
            self.calibration_file, self.cam2_name
        )

        # ------------------------------------------------------------
        # Letzte Schaetzungen pro Kamera (thread-safe gespeichert).
        # ------------------------------------------------------------
        # Jede Schaetzung: (timestamp_node, position, quaternion, weight)
        # timestamp_node ist self.get_clock().now() im Moment der Berechnung.
        # ------------------------------------------------------------
        self._lock = Lock()
        self._last_cam1 = None
        self._last_cam2 = None

        # EMA-Zustand
        self._filtered_t = None
        self._filtered_q = None

        # Deterministischer RNG-Seed fuer reproduzierbare RANSAC-Ergebnisse.
        # Falls die Anzahl Tags die ransac_max_iterations uebersteigt und
        # wir tatsaechlich Zufallssampling brauchen, soll das Verhalten
        # zwischen Programmlaeufen reproduzierbar sein (wichtig fuer
        # Debugging und Tests).
        self._ransac_rng = random.Random(42)

        # ------------------------------------------------------------
        # QoS: sensor_data fuer schnelle Lieferung mit Queue-Size 1
        # ------------------------------------------------------------
        # AprilTag-Detections wollen wir frisch, alte werfen wir weg.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ------------------------------------------------------------
        # Subscriber & Publisher
        # ------------------------------------------------------------
        self.sub_cam1 = self.create_subscription(
            AprilTagDetectionArray,
            self.cam1_topic,
            self.callback_cam1,
            qos,
        )
        self.sub_cam2 = self.create_subscription(
            AprilTagDetectionArray,
            self.cam2_topic,
            self.callback_cam2,
            qos,
        )

        # ------------------------------------------------------------
        # Publisher: Message-Typ je nach use_covariance.
        # ------------------------------------------------------------
        # PoseWithCovarianceStamped erlaubt PX4 EKF2, die Vision-Pose
        # pro Frame adaptiv zu gewichten. Erfordert EKF2_EV_NOISE_MD=1
        # auf PX4-Seite. Topic muss zum Message-Typ passen:
        #   PoseStamped              -> /mavros/vision_pose/pose
        #   PoseWithCovarianceStamped-> /mavros/vision_pose/pose_cov
        if self.use_covariance:
            self.pub = self.create_publisher(
                PoseWithCovarianceStamped,
                self.publish_topic,
                10,
            )
        else:
            self.pub = self.create_publisher(
                PoseStamped,
                self.publish_topic,
                10,
            )

        # ------------------------------------------------------------
        # Timer fuer konstantes Publishen
        # ------------------------------------------------------------
        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_timer,
        )

        # ------------------------------------------------------------
        # Konfig-Log
        # ------------------------------------------------------------
        self.get_logger().info(
            f"Vision Pose Direct Node gestartet"
        )
        self.get_logger().info(f"Map: {self.map_file} ({len(self.map_tags)} Tags)")
        self.get_logger().info(f"Kalibrierung: {self.calibration_file}")
        self.get_logger().info(f"Cam1 ({self.cam1_name}): {self.cam1_topic}")
        self.get_logger().info(f"Cam2 ({self.cam2_name}): {self.cam2_topic}")
        msg_type = "PoseWithCovarianceStamped" if self.use_covariance else "PoseStamped"
        self.get_logger().info(
            f"Publish: {self.publish_topic} @ {publish_rate_hz:.1f} Hz "
            f"({msg_type})"
        )
        self.get_logger().info(
            f"Max Estimate Age: {self.max_estimate_age_s*1000:.0f} ms"
        )
        self.get_logger().info(
            f"EMA alpha: pos={self.position_alpha:.2f}, rot={self.rotation_alpha:.2f}"
        )
        self.get_logger().info(
            f"min_tags_required: {self.min_tags_required}"
        )
        self.get_logger().info(
            f"RANSAC: base_threshold={self.ransac_base_threshold_m*1000:.1f} mm "
            f"@ {self.ransac_reference_distance_m:.2f} m, "
            f"min_inliers_ratio={self.ransac_min_inliers_ratio:.2f}, "
            f"refinement_iter={self.ransac_refinement_iterations}"
        )
        if self.use_covariance:
            self.get_logger().info(
                f"Kovarianz: pos_sigma_base={self.cov_pos_base_sigma*1000:.1f} mm, "
                f"rot_sigma_base={math.degrees(self.cov_rot_base_sigma):.2f} deg "
                f"@ {self.cov_reference_distance:.2f} m / "
                f"{self.cov_reference_tag_size*100:.1f} cm Tag. "
                f"Wichtig: PX4 EKF2_EV_NOISE_MD=1 setzen!"
            )

        # Info zur Tag-Groessen-Skalierung
        self.get_logger().info(
            f"Isaac-Referenzgroesse: {self.isaac_reference_size:.4f} m "
            f"(default_tag_size aus Map: {self.map_default_tag_size:.4f} m). "
            f"{len(self.tag_scale)} Tag(s) werden skaliert."
        )
        # Warnung, falls Map-default und gewaehlte Referenz nicht
        # uebereinstimmen. Mathematisch ist das ok (auch "Default-Tags"
        # werden dann skaliert), aber meistens ist es ein Konfig-Fehler.
        if (self.map_default_tag_size > 0.0 and
                abs(self.map_default_tag_size - self.isaac_reference_size) > 1e-6):
            self.get_logger().warn(
                f"Map default_tag_size ({self.map_default_tag_size:.4f} m) "
                f"!= Isaac-Referenzgroesse ({self.isaac_reference_size:.4f} m). "
                f"Auch Default-Tags werden skaliert. "
                f"Tipp: Isaac AprilTagNode auf "
                f"{self.map_default_tag_size:.4f} m setzen."
            )

        # ------------------------------------------------------------
        # Optional: Tag-Karte als Static-TF veroeffentlichen (Debug).
        # Komplett ueberspringen, wenn nicht aktiviert -> keine CPU.
        # ------------------------------------------------------------
        publish_tag_map_tf = bool(
            self.get_parameter("publish_tag_map_tf").value
        )
        if publish_tag_map_tf:
            tag_frame_prefix = str(
                self.get_parameter("tag_frame_prefix").value
            )
            self._publish_tag_map_static_tf(tag_frame_prefix)
            self.get_logger().info(
                f"Tag-Map TF aktiv: {len(self.map_tags)} Tags als "
                f"static TF unter '{self.map_frame}' "
                f"(Prefix '{tag_frame_prefix}')."
            )
        else:
            self.get_logger().info("Tag-Map TF deaktiviert.")

    # ====================================================================
    # TF-Veroeffentlichung der Tag-Karte (optional, Debug)
    # ====================================================================

    def _publish_tag_map_static_tf(self, tag_frame_prefix):
        """
        Publiziert alle Tags aus der geladenen Karte einmalig als
        statische Transformationen auf /tf_static.

        Wird nur aufgerufen, wenn publish_tag_map_tf=True ist.
        Static-TF ist latched: einmal senden reicht, spaeter
        verbindende Subscriber (z.B. RViz) bekommen die Daten
        automatisch. Danach Null-CPU-Last.
        """
        # Bewusst lokaler Import: wenn das Feature aus ist, wird
        # tf2_ros nicht einmal geladen.
        from tf2_ros import StaticTransformBroadcaster

        # Wichtig: Broadcaster als Instanz-Attribut behalten.
        # Wenn er lokal bliebe, wuerde Python ihn am Funktionsende
        # garbage-collecten, und /tf_static waere kurz nach dem Start
        # wieder leer.
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        now = self.get_clock().now().to_msg()
        transforms = []

        for tag_id, (t, q) in self.map_tags.items():
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = f"{tag_frame_prefix}{tag_id}"
            tf_msg.transform.translation.x = float(t[0])
            tf_msg.transform.translation.y = float(t[1])
            tf_msg.transform.translation.z = float(t[2])
            tf_msg.transform.rotation.x = float(q[0])
            tf_msg.transform.rotation.y = float(q[1])
            tf_msg.transform.rotation.z = float(q[2])
            tf_msg.transform.rotation.w = float(q[3])
            transforms.append(tf_msg)

        # sendTransform akzeptiert eine Liste -> ein einziger Call,
        # alle Tags in einer /tf_static-Message.
        self._static_tf_broadcaster.sendTransform(transforms)

    # ====================================================================
    # Laden der Karten / Kalibrierungen
    # ====================================================================

    def load_map(self, path):
        """
        Laedt TagSLAM-Karte. Unterstuetzt:
          A) Top-Level 'tags' (altes Format)
          B) 'bodies' mit Body 'tag_map' (TagSLAM-Standardformat)

        Setzt zusaetzlich:
          - self.map_tag_sizes : {tag_id: size_in_meter}
          - self.map_default_tag_size : float (0.0 wenn nicht gesetzt)
        """
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f"Map file not found: {path}")

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            raise ValueError(f"Invalid map YAML: {path}")

        # Default-Tag-Groesse zuruecksetzen. Wird ggf. unten ueberschrieben.
        self.map_default_tag_size = 0.0

        # Variante A: Top-Level tags
        # (Format ohne 'bodies' und ohne default_tag_size auf Body-Ebene.)
        if "tags" in data:
            return self._parse_tag_list(
                data["tags"],
                body_t=(0.0, 0.0, 0.0),
                body_q=(0.0, 0.0, 0.0, 1.0),
                default_size=0.0,
            )

        # Variante B: bodies (TagSLAM-Standardformat)
        if "bodies" in data:
            body_t = (0.0, 0.0, 0.0)
            body_q = (0.0, 0.0, 0.0, 1.0)
            body_default_size = 0.0
            tag_list = []

            for item in data["bodies"]:
                if not isinstance(item, dict):
                    continue
                for name, body in item.items():
                    if not isinstance(body, dict):
                        continue
                    body_tags = body.get("tags", [])
                    if not body_tags:
                        continue
                    if name == "tag_map":
                        body_t, body_q = pose_to_transform(body.get("pose", {}))
                        # TagSLAM nennt das Feld 'default_tag_size'.
                        # Es gilt fuer alle Tags im Body, die keine
                        # eigene 'size' definieren.
                        body_default_size = float(
                            body.get("default_tag_size", 0.0)
                        )
                        tag_list = body_tags
                        break
                if tag_list:
                    break

            if not tag_list:
                raise ValueError("Kein Body 'tag_map' mit Tags in Map gefunden.")

            # Body-Default merken, damit der __init__ darauf zugreifen kann,
            # um die Isaac-Referenzgroesse zu setzen.
            self.map_default_tag_size = body_default_size

            return self._parse_tag_list(
                tag_list,
                body_t=body_t,
                body_q=body_q,
                default_size=body_default_size,
            )

        raise ValueError("Unbekanntes Map-Format.")

    def _parse_tag_list(self, tag_list, body_t, body_q, default_size=0.0):
        """
        Tag-Posen aus YAML in die interne Struktur uebernehmen.

        Zusaetzlich wird die echte Tag-Groesse pro ID nach
        self.map_tag_sizes geschrieben. Wenn ein Tag keinen
        eigenen 'size'-Eintrag hat, erbt er default_size vom
        umgebenden Body (in TagSLAM 'default_tag_size').

        Diese Groessen brauchen wir spaeter, um die Isaac-PnP-
        Translation pro Tag korrekt zu skalieren (Isaac kennt
        nur EINE 'size' fuer alle Detections).
        """
        tags = {}
        # map_tag_sizes ist ein Klassen-Attribut, das im __init__
        # vor dem ersten load_map-Aufruf nicht existiert. Wir legen
        # es hier lazy an, falls noch nicht vorhanden, damit auch
        # bei mehreren Body-Parsing-Durchlaeufen die Sammlung
        # erhalten bleibt.
        if not hasattr(self, "map_tag_sizes"):
            self.map_tag_sizes = {}

        for tag in tag_list:
            if not isinstance(tag, dict):
                continue
            tag_id = int(tag["id"])
            tag_t, tag_q = pose_to_transform(tag.get("pose", {}))
            map_t, map_q = transform_multiply(body_t, body_q, tag_t, tag_q)
            tags[tag_id] = (map_t, map_q)

            # Tag-spezifische 'size' aus dem YAML-Eintrag lesen.
            # Wenn nicht vorhanden -> default vom Body uebernehmen.
            self.map_tag_sizes[tag_id] = float(
                tag.get("size", default_size)
            )

        return tags

    def load_camera_to_drone(self, path, camera_name):
        """
        Laedt die Transformation camera_X -> drone_center aus der
        Kalibrierungs-YAML.

        Erwartetes Format:
            cameras:
              camera_X:
                transform_matrix_4x4:
                  - [r11, r12, r13, tx]
                  - [r21, r22, r23, ty]
                  - [r31, r32, r33, tz]
                  - [0, 0, 0, 1]

        Die YAML beschreibt drone_center -> camera_X.
        Wir geben die Inverse zurueck: camera_X -> drone_center.
        """
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f"Calibration file not found: {path}")

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        cam_data = data.get("cameras", {}).get(camera_name, None)
        if cam_data is None:
            raise ValueError(
                f"Kamera '{camera_name}' nicht in Kalibrierung gefunden."
            )

        T = cam_data["transform_matrix_4x4"]

        # T_drone_cam zerlegen
        R_drone_cam = [
            [float(T[0][0]), float(T[0][1]), float(T[0][2])],
            [float(T[1][0]), float(T[1][1]), float(T[1][2])],
            [float(T[2][0]), float(T[2][1]), float(T[2][2])],
        ]
        t_drone_cam = (
            float(T[0][3]),
            float(T[1][3]),
            float(T[2][3]),
        )
        q_drone_cam = rot_matrix_to_quat(R_drone_cam)

        # Invertieren: T_cam_drone
        t_cam_drone, q_cam_drone = transform_inverse(t_drone_cam, q_drone_cam)
        return t_cam_drone, q_cam_drone

    # ====================================================================
    # Callbacks pro Kamera
    # ====================================================================

    def callback_cam1(self, msg):
        result = self.process_detections(
            msg,
            t_cam_drone=self.t_cam_drone_1,
            q_cam_drone=self.q_cam_drone_1,
            weight=self.cam1_weight,
            cam_label="cam1",
        )
        if result is not None:
            with self._lock:
                self._last_cam1 = result

    def callback_cam2(self, msg):
        result = self.process_detections(
            msg,
            t_cam_drone=self.t_cam_drone_2,
            q_cam_drone=self.q_cam_drone_2,
            weight=self.cam2_weight,
            cam_label="cam2",
        )
        if result is not None:
            with self._lock:
                self._last_cam2 = result

    def process_detections(self, msg, t_cam_drone, q_cam_drone, weight, cam_label):
        """
        Rechnet aus einer AprilTagDetectionArray-Nachricht die Pose der
        Drohnenmitte in tag_map.

        Schritte:
            1) Pro Detection: Kamera-Pose in tag_map berechnen
               (T_map_camera = T_map_tag * inverse(T_camera_tag)).
            2) RANSAC-Outlier-Filter ueber alle Tag-Schaetzungen.
            3) Gewichtete Mittelung -> Kamera-Pose in tag_map.
            4) Mit T_cam_drone multiplizieren -> Drohnen-Pose in tag_map.
            5) Falls use_covariance: pro Inlier-Tag eine Varianz schaetzen,
               basierend auf Tag-Groesse (aus Map) und Tag-Distanz.

        Returns:
            Tuple (timestamp_node, position, quaternion, weight,
                   var_pos_list, var_rot_list) oder None, wenn nicht
            genug Tags sichtbar sind.

            var_pos_list / var_rot_list enthalten je eine Varianz pro
            Inlier-Tag. Wenn use_covariance=False sind beide Listen leer.
            Im publish_timer werden sie ueber alle Kameras hinweg via
            inverse-Varianz-Gewichtung kombiniert.
        """
        raw_estimates = []

        for det in msg.detections:
            tag_id = self._get_tag_id(det)
            if tag_id not in self.map_tags:
                continue

            estimate = self._estimate_camera_pose_from_tag(det, tag_id)
            if estimate is not None:
                raw_estimates.append(estimate)

        if len(raw_estimates) < self.min_tags_required:
            return None

        # ----------------------------------------------------------------
        # RANSAC-Outlier-Filter (adaptiv).
        # Ersetzt den frueheren achsenweisen Median-Filter.
        # Vorteile: robust gegen bimodale Verteilungen (z.B. PnP-Front-
        # Back-Flip bei schraegen Tags), distanz-adaptive Schwelle.
        # ----------------------------------------------------------------
        filtered = self._filter_outliers_ransac(raw_estimates)

        if len(filtered) < self.min_tags_required:
            return None

        # Gewichtete Fusion der Kamera-Pose in tag_map
        t_cam_map, q_cam_map = self._fuse_estimates(filtered)

        # Kamera-Pose in tag_map zur Drohnen-Pose umrechnen:
        # T_map_drone = T_map_camera * T_camera_drone
        t_drone_map, q_drone_map = transform_multiply(
            t_cam_map, q_cam_map,
            t_cam_drone, q_cam_drone,
        )

        # ----------------------------------------------------------------
        # Pro-Tag-Varianzen fuer Kovarianz-Schaetzung sammeln.
        #
        # Wichtig: Wir berechnen die Varianzen pro Tag JETZT - mit der
        # echten Tag-Groesse und der echten Tag-Distanz. Spaeter im
        # publish_timer werden alle Varianzen via inverse-Varianz-
        # Gewichtung kombiniert.
        #
        # Vorteil dieser Pro-Tag-Berechnung gegenueber "Distanz mitteln,
        # dann Varianz":
        #   - Tag-Groessen koennen unterschiedlich sein.
        #   - Ein grosser, naher Tag dominiert korrekt die Schaetzung.
        #   - Ein kleiner, weit entfernter Tag traegt korrekt weniger bei.
        #
        # Tupel-Layout pro Tag in filtered: (weight, t, q, tag_id, distance)
        # ----------------------------------------------------------------
        var_pos_list = []
        var_rot_list = []
        if self.use_covariance:
            for est in filtered:
                _, _, _, tag_id, distance = est
                vp, vr = self._per_tag_variance(tag_id, distance)
                var_pos_list.append(vp)
                var_rot_list.append(vr)

        now = self.get_clock().now()

        # Tupel-Layout fuer den Hot-Path:
        #   (stamp, t, q, weight, var_pos_list, var_rot_list)
        # Wenn use_covariance=False, sind die Listen leer - der
        # publish_timer behandelt das transparent.
        return (now, t_drone_map, q_drone_map, weight,
                var_pos_list, var_rot_list)

    def _get_tag_id(self, det):
        """Isaac liefert det.id mal als int, mal als Liste."""
        if isinstance(det.id, (list, tuple)):
            return int(det.id[0])
        return int(det.id)

    def _estimate_camera_pose_from_tag(self, det, tag_id):
        """
        Aus einer einzelnen Detection: Kamera-Pose in tag_map.

        Returns:
            (weight, position, quaternion, tag_id, distance) oder None.
        """
        t_map_tag, q_map_tag = self.map_tags[tag_id]

        p = det.pose.pose.pose.position
        q = det.pose.pose.pose.orientation

        t_camera_tag = (float(p.x), float(p.y), float(p.z))
        q_camera_tag_isaac = quat_normalize((
            float(q.x), float(q.y), float(q.z), float(q.w)
        ))

        # ----------------------------------------------------------
        # Multi-Size-Korrektur
        # ----------------------------------------------------------
        # Isaac hat die Translation mit isaac_reference_size berechnet.
        # Wenn der echte Tag eine andere Groesse hat, multiplizieren
        # wir die Translation linear hoch/runter. Die Rotation bleibt
        # unveraendert -- sie haengt nicht von der Tag-Groesse ab.
        # dict.get liefert 1.0 fuer alle Tags ohne Override -> dann
        # ueberspringen wir den Multiplikations-Block ganz.
        # ----------------------------------------------------------
        scale = self.tag_scale.get(tag_id, 1.0)
        if scale != 1.0:
            t_camera_tag = (
                t_camera_tag[0] * scale,
                t_camera_tag[1] * scale,
                t_camera_tag[2] * scale,
            )

        # Konvention auf TagSLAM-Karte angleichen
        # (war Teil des funktionierenden alten Setups)
        q_camera_tag = apply_apriltag_ros_rotation(q_camera_tag_isaac)

        # T_map_camera = T_map_tag * inverse(T_camera_tag)
        t_tag_camera, q_tag_camera = transform_inverse(t_camera_tag, q_camera_tag)
        t_map_camera, q_map_camera = transform_multiply(
            t_map_tag, q_map_tag,
            t_tag_camera, q_tag_camera,
        )

        # Gewicht: nahe Tags sind genauer (1/d^2).
        # Wichtig: distance wird aus dem BEREITS SKALIERTEN
        # t_camera_tag berechnet, damit die Gewichtung kleiner Tags
        # relativ zu grossen Tags physikalisch korrekt ist.
        distance = max(math.sqrt(
            t_camera_tag[0]**2 +
            t_camera_tag[1]**2 +
            t_camera_tag[2]**2
        ), 0.001)
        weight = 1.0 / (distance * distance)

        return (weight, t_map_camera, q_map_camera, tag_id, distance)

    # ====================================================================
    # RANSAC-Outlier-Filter (adaptiv)
    # ====================================================================

    def _filter_outliers_ransac(self, estimates):
        """
        Adaptiver RANSAC-Outlier-Filter fuer Pose-Schaetzungen.

        Idee: Jeder Tag liefert eine Hypothese fuer die Kamera-Position
        in tag_map. Bei korrekten Tags muessen diese Hypothesen ueberein-
        stimmen. RANSAC sucht das groesste Inlier-Set, also die Tags,
        deren Hypothesen konsistent sind.

        Anpassungen fuer den Flug-Use-Case:

          1) Distanz-adaptive Schwelle.
             Bei groesserer Distanz zwischen Kamera und Tags wird das
             PnP-Rauschen quadratisch groesser (Pixel-Quantisierung
             projiziert sich auf groessere Strecken). Die Inlier-Schwelle
             wird daher mit (mean_distance / reference_distance)^2
             hochskaliert. Begrenzt durch ransac_max_distance_factor.

          2) Fallback fuer wenige Tags.
             - 0 Tags: leere Liste.
             - 1 Tag: durchreichen (im Aufrufer wird min_tags_required
               geprueft, ein einzelner Tag wird normalerweise blockiert).
             - 2 Tags: Konsistenz-Check; wenn beide konsistent sind,
               beide durchreichen; sonst leere Liste (kein Konsens).
             - >= 3 Tags: vollwertiges RANSAC.

          3) Exhaustive Sampling wenn moeglich.
             Bei n <= ransac_max_iterations probieren wir JEDEN Tag
             einmal als Hypothese durch. Das ist deterministisch (kein
             Random Seed Drift zwischen Frames) und garantiert das
             Finden des optimalen Inlier-Sets. Erst bei mehr Tags
             wird zufaellig gesamplet.

          4) Mindest-Inlier-Anteil.
             Erst ab ransac_min_inliers_ratio * n Inliern gilt ein
             Konsens als zuverlaessig. Bei schwachem Konsens wird
             gar nichts gemeldet - PX4 EKF2 ueberbrueckt die Luecke
             dann per IMU-Dead-Reckoning.

          5) Refinement.
             Nach dem initialen RANSAC wird mehrfach iterativ der
             Schwerpunkt aller Inlier berechnet und dagegen neu
             gefiltert. Konvergiert in 1-2 Iterationen und liefert
             eine genauere Endschaetzung als ein einzelner Tag.

        Args:
            estimates: Liste von (weight, t, q, tag_id, distance)-
                       Tupeln (Output von _estimate_camera_pose_from_tag).

        Returns:
            Liste der Inlier-Estimates. Leer, wenn kein Konsens gefunden.
        """
        n = len(estimates)

        if n == 0:
            return []

        # --------------------------------------------------------------
        # Sonderfall: Nur ein Tag.
        # Kein Filtern moeglich (keine zweite Schaetzung zum Vergleich).
        # Wir reichen den Tag durch; ob er publiziert wird, entscheidet
        # die min_tags_required-Pruefung im Aufrufer.
        # --------------------------------------------------------------
        if n == 1:
            return list(estimates)

        # --------------------------------------------------------------
        # Adaptive Inlier-Schwelle (Quadrat, weil wir ueberall mit
        # quadrierten Distanzen arbeiten, um sqrt() zu vermeiden).
        # --------------------------------------------------------------
        threshold_sq = self._compute_adaptive_threshold_sq(estimates)

        # --------------------------------------------------------------
        # Sonderfall: Genau zwei Tags.
        # Wir koennen pruefen, ob die Schaetzungen konsistent sind.
        # Wenn ja, beide durchreichen. Wenn nein, koennen wir nicht
        # entscheiden, welcher der richtige ist - lieber gar nichts
        # publishen, als womoeglich den falschen zu nehmen.
        # --------------------------------------------------------------
        if n == 2:
            _, t1, _, _, _ = estimates[0]
            _, t2, _, _, _ = estimates[1]
            d_sq = (
                (t1[0] - t2[0])**2 +
                (t1[1] - t2[1])**2 +
                (t1[2] - t2[2])**2
            )
            if d_sq <= threshold_sq:
                return list(estimates)
            else:
                # Inkonsistente Zwei-Tag-Situation. Vorsicht: lieber
                # auslassen als eine evtl. falsche Pose senden.
                return []

        # --------------------------------------------------------------
        # Vollwertiges RANSAC (n >= 3).
        # --------------------------------------------------------------
        # Wahl der Hypothesen-Indizes:
        #   - Wenn n klein genug: alle Indizes durchprobieren
        #     (deterministisch, optimal).
        #   - Sonst: zufaellige Auswahl.
        if n <= self.ransac_max_iterations:
            sample_indices = list(range(n))
        else:
            sample_indices = self._ransac_rng.sample(
                range(n), self.ransac_max_iterations
            )

        best_inliers = []
        # Frueher Abbruch, wenn fast alle Tags Inlier sind.
        # Weiteres Sampeln wuerde dann nichts mehr verbessern.
        early_exit_count = int(n * 0.9)

        for idx in sample_indices:
            _, t_hyp, _, _, _ = estimates[idx]

            inliers = []
            for est in estimates:
                _, t, _, _, _ = est
                d_sq = (
                    (t[0] - t_hyp[0])**2 +
                    (t[1] - t_hyp[1])**2 +
                    (t[2] - t_hyp[2])**2
                )
                if d_sq <= threshold_sq:
                    inliers.append(est)

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                if len(inliers) >= early_exit_count:
                    break

        # --------------------------------------------------------------
        # Konsens-Pruefung: ist das Inlier-Set gross genug?
        # Wir nehmen das groessere von:
        #   - min_tags_required (harte Untergrenze)
        #   - min_inliers_ratio * n (relativer Anteil)
        # --------------------------------------------------------------
        min_required = max(
            self.min_tags_required,
            int(round(n * self.ransac_min_inliers_ratio)),
        )
        if len(best_inliers) < min_required:
            return []

        # --------------------------------------------------------------
        # Refinement: mit den Inliern den gewichteten Schwerpunkt
        # bilden und nochmal alle Tags dagegen pruefen. Das ist
        # genauer als der Einzeltag-Vergleich, weil der Schwerpunkt
        # eine bessere Schaetzung der wahren Position ist.
        # --------------------------------------------------------------
        if self.ransac_refinement_iterations > 0:
            best_inliers = self._refine_inliers(
                estimates,
                best_inliers,
                threshold_sq,
            )

        return best_inliers

    def _compute_adaptive_threshold_sq(self, estimates):
        """
        Berechnet die adaptive RANSAC-Inlier-Schwelle als Quadrat.

        Die Schwelle skaliert quadratisch mit der mittleren Tag-
        Distanz, weil das PnP-Rauschen ungefaehr mit d^2 waechst:
        Pixel-Quantisierung an den Tag-Ecken projiziert sich bei
        groesserer Tag-Entfernung auf groessere reale Strecken.

        Formel:
            factor   = (mean_distance / reference_distance)^2
            factor   = clamp(factor, 0.5, max_distance_factor)
            threshold = base_threshold * factor

        Args:
            estimates: Liste von (weight, t, q, tag_id, distance)-Tupeln.

        Returns:
            Quadrat der Inlier-Schwelle in m^2.
        """
        # Mittlere Tag-Distanz aus den Estimates (Distance ist das
        # 5. Element des Tupels, gefuellt von _estimate_camera_pose_from_tag).
        mean_distance = sum(e[4] for e in estimates) / len(estimates)

        distance_factor = (mean_distance / self.ransac_reference_distance_m)**2

        # Untergrenze: bei sehr nahen Tags soll die Schwelle nicht
        # unrealistisch klein werden (Kamera-Kalibrier-Fehler erzeugen
        # immer eine gewisse Mindest-Streuung).
        # Obergrenze: gegen Runaway bei sehr grossen Distanzen.
        if distance_factor < 0.5:
            distance_factor = 0.5
        elif distance_factor > self.ransac_max_distance_factor:
            distance_factor = self.ransac_max_distance_factor

        threshold = self.ransac_base_threshold_m * distance_factor
        return threshold * threshold

    def _refine_inliers(self, all_estimates, inliers, threshold_sq):
        """
        Iteratives Refinement der Inlier-Menge.

        Nach dem initialen RANSAC ist die Hypothese ein einzelner Tag.
        Der gewichtete Schwerpunkt aller bisherigen Inlier ist aber
        i.d.R. eine bessere Schaetzung der wahren Position. Wir
        wiederholen daher:
            1) Schwerpunkt der aktuellen Inlier berechnen.
            2) Alle (originalen) Tags gegen diesen Schwerpunkt pruefen.
            3) Neue Inlier-Menge bilden.
        bis Konvergenz oder max. Iterationen erreicht.

        Args:
            all_estimates: vollstaendige Liste aller Tag-Schaetzungen.
            inliers: aktuell als Inlier markierte Estimates.
            threshold_sq: quadrierte Inlier-Schwelle.

        Returns:
            Verfeinerte Inlier-Liste.
        """
        for _ in range(self.ransac_refinement_iterations):
            # Gewichteter Schwerpunkt der aktuellen Inlier
            total_w = sum(e[0] for e in inliers)
            if total_w <= 0.0:
                # Kann passieren, wenn alle Inlier weight=0 haben
                # (sollte nicht vorkommen, aber sicherheitshalber).
                break

            cx = sum(w * t[0] for w, t, _, _, _ in inliers) / total_w
            cy = sum(w * t[1] for w, t, _, _, _ in inliers) / total_w
            cz = sum(w * t[2] for w, t, _, _, _ in inliers) / total_w

            # Alle Tags neu gegen den Schwerpunkt pruefen
            new_inliers = []
            for est in all_estimates:
                _, t, _, _, _ = est
                d_sq = (
                    (t[0] - cx)**2 +
                    (t[1] - cy)**2 +
                    (t[2] - cz)**2
                )
                if d_sq <= threshold_sq:
                    new_inliers.append(est)

            # Konvergenz: keine Aenderung in der Inlier-Menge
            if len(new_inliers) == len(inliers):
                break

            # Sicherheitscheck: nicht unter Mindestanzahl fallen
            if len(new_inliers) < self.min_tags_required:
                # Lieber alte Menge behalten als zu wenige zurueckgeben.
                break

            inliers = new_inliers

        return inliers

    # ====================================================================
    # Fusion (gewichtete Mittelung)
    # ====================================================================

    def _fuse_estimates(self, estimates):
        """Gewichtete Mittelung aller Tag-Schaetzungen in tag_map."""
        total_w = sum(e[0] for e in estimates)
        if total_w <= 0.0:
            # Fallback: ungewichtete Mittelung
            n = len(estimates)
            tx = sum(e[1][0] for e in estimates) / n
            ty = sum(e[1][1] for e in estimates) / n
            tz = sum(e[1][2] for e in estimates) / n
            ref_q = estimates[0][2]
            return (tx, ty, tz), ref_q

        tx = sum(w * t[0] for w, t, q, _, _ in estimates) / total_w
        ty = sum(w * t[1] for w, t, q, _, _ in estimates) / total_w
        tz = sum(w * t[2] for w, t, q, _, _ in estimates) / total_w

        # Quaternion-Mittelung mit Vorzeichenangleichung
        ref_q = estimates[0][2]
        qx = qy = qz = qw = 0.0
        for w, t, q, _, _ in estimates:
            dot = ref_q[0]*q[0] + ref_q[1]*q[1] + ref_q[2]*q[2] + ref_q[3]*q[3]
            if dot < 0.0:
                q = (-q[0], -q[1], -q[2], -q[3])
            qx += w * q[0]
            qy += w * q[1]
            qz += w * q[2]
            qw += w * q[3]

        q_fused = quat_normalize((qx, qy, qz, qw))
        return (tx, ty, tz), q_fused

    # ====================================================================
    # Kovarianz-Schaetzung
    # ====================================================================

    def _per_tag_variance(self, tag_id, distance):
        """
        Berechnet die erwartete Pose-Varianz fuer EINEN einzelnen Tag.

        Rauschmodell:
            sigma_pos = base_sigma * (d/d_ref)^2 * (s_ref/s_tag)
            sigma_rot = base_rot   * (d/d_ref)^2 * (s_ref/s_tag)

        Begruendung:
            - (d/d_ref)^2: PnP-Translation-Rauschen waechst quadratisch
              mit der Distanz (Pixel-Quantisierung projiziert sich auf
              groessere reale Strecken).
            - (s_ref/s_tag): Bei groesseren Tags ist die Pixel-Aufloesung
              der Tag-Ecken im Bild besser, das PnP-Rauschen sinkt
              linear mit der Tag-Groesse.

        Mathematisch sauberer waere noch ein Faktor fuer den Tag-Winkel
        zur Kamera (schraege Tags = schlechter), aber den haben wir
        hier nicht verlaesslich. RANSAC filtert die schlimmsten
        Schraeg-Faelle (Front-Back-Flip) bereits raus.

        Args:
            tag_id: ID des Tags - wird genutzt, um die Tag-Groesse aus
                    der Map zu lesen (self.map_tag_sizes).
            distance: Aktuelle Tag-zu-Kamera-Distanz in Metern.

        Returns:
            Tuple (var_pos, var_rot) in (m^2, rad^2).
        """
        # Tag-Groesse aus der Karte holen. Fallback auf Referenzgroesse,
        # falls der Tag aus irgendeinem Grund keine Groesse hat (sollte
        # nach load_map nicht vorkommen, aber defensiv programmieren).
        tag_size = self.map_tag_sizes.get(tag_id, self.cov_reference_tag_size)
        if tag_size <= 0.0:
            tag_size = self.cov_reference_tag_size

        # Distanz-Faktor (quadratisch)
        distance_factor = (distance / self.cov_reference_distance)**2

        # Untergrenze: bei sehr nahen Tags bleibt ein Kalibrier-Restfehler
        if distance_factor < 0.5:
            distance_factor = 0.5
        # Obergrenze: gegen Runaway bei sehr grossen Distanzen
        elif distance_factor > self.cov_max_distance_factor:
            distance_factor = self.cov_max_distance_factor

        # Tag-Groessen-Faktor (linear, invers).
        # Groesserer Tag = kleinere Sigma. Kleinerer Tag = groessere Sigma.
        size_factor = self.cov_reference_tag_size / tag_size

        # Untergrenze: 0.1 verhindert, dass ein extrem grosser Tag die
        # Sigma unter den Kalibrier-Restfehler druecken kann.
        if size_factor < 0.1:
            size_factor = 0.1
        # Obergrenze: schuetzt vor unrealistischen Sigma bei winzigen Tags
        elif size_factor > self.cov_max_size_factor:
            size_factor = self.cov_max_size_factor

        # Pro-Tag-Sigma berechnen
        sigma_pos = self.cov_pos_base_sigma * distance_factor * size_factor
        sigma_rot = self.cov_rot_base_sigma * distance_factor * size_factor

        # Varianzen = Sigma quadriert
        var_pos = sigma_pos * sigma_pos
        var_rot = sigma_rot * sigma_rot

        return var_pos, var_rot

    def _combine_variances(self, variances):
        """
        Kombiniert mehrere Pro-Tag-Varianzen zu einer Gesamtvarianz.

        Verwendet inverse-Varianz-Gewichtung (Maximum-Likelihood-
        Schaetzung): Tags mit kleinerer Varianz dominieren, Tags mit
        grosser Varianz tragen kaum bei.

        Formel:
            1/var_total = sum(1/var_i)
            var_total   = 1 / sum(1/var_i)

        Beispiel:
            Tag A: sigma = 5 mm  -> Var = 25 mm^2
            Tag B: sigma = 50 mm -> Var = 2500 mm^2
            Var_total = 1 / (1/25 + 1/2500) = 24.75 mm^2
            Sigma_total ~ 5 mm
            -> Der gute Tag dominiert, der schlechte hat fast keinen
               Einfluss. Genau das, was wir wollen.

        Vergleich zur Multi-Tag-Standardformel sigma/sqrt(n):
            - Bei gleichen Varianzen (alle Tags gleich gut): identisch.
            - Bei sehr unterschiedlichen Varianzen: inverse-Varianz
              ist deutlich besser (gewichtet automatisch korrekt).

        Args:
            variances: Liste von Pro-Tag-Varianzen (alle in m^2 oder
                       alle in rad^2 - die Funktion ist generisch).

        Returns:
            Kombinierte Varianz in derselben Einheit.
        """
        if not variances:
            # Sollte nie passieren, defensive Programmierung.
            # Hoher Wert -> EKF2 ignoriert die Messung weitgehend.
            return 1.0

        # Summe der inversen Varianzen
        inv_sum = 0.0
        for v in variances:
            if v > 1e-12:
                inv_sum += 1.0 / v

        if inv_sum <= 1e-12:
            # Alle Varianzen waren null/negativ - unrealistisch,
            # aber wir bleiben defensiv.
            return 1.0

        return 1.0 / inv_sum

    def _estimate_covariance(self, var_pos_list, var_rot_list):
        """
        Erzeugt die finale 6x6-Kovarianzmatrix aus den gesammelten
        Pro-Tag-Varianzen.

        Schritt 1: Pro-Tag-Varianzen mittels inverse-Varianz-Gewichtung
                   zu Gesamt-Varianzen kombinieren.
        Schritt 2: Diagonale der 6x6-Matrix mit diesen Varianzen
                   belegen. Off-Diagonalen bleiben 0 (Annahme:
                   Achsen sind unkorreliert - das ist eine Vereinfachung,
                   aber fuer PX4 EKF2 in der Praxis ausreichend).

        Args:
            var_pos_list: Liste der Pro-Tag-Positions-Varianzen (m^2).
            var_rot_list: Liste der Pro-Tag-Rotations-Varianzen (rad^2).
                          Muss dieselbe Laenge wie var_pos_list haben.

        Returns:
            Liste von 36 Float-Werten (row-major 6x6-Matrix).
            Diagonale: Varianzen. Off-Diagonale: 0.
        """
        var_pos = self._combine_variances(var_pos_list)
        var_rot = self._combine_variances(var_rot_list)

        # 6x6 row-major Layout in flacher 36-Element-Liste.
        # Diagonal-Indizes: 0 (xx), 7 (yy), 14 (zz),
        #                   21 (roll-roll), 28 (pitch-pitch), 35 (yaw-yaw).
        cov = [0.0] * 36
        cov[0]  = var_pos   # x
        cov[7]  = var_pos   # y
        cov[14] = var_pos   # z
        cov[21] = var_rot   # roll
        cov[28] = var_rot   # pitch
        cov[35] = var_rot   # yaw

        return cov

    # ====================================================================
    # Publish-Timer
    # ====================================================================

    def publish_timer(self):
        """
        Wird mit publish_rate_hz aufgerufen.
        Fusioniert die juengsten gueltigen Kamera-Schaetzungen und
        publiziert auf publish_topic. Je nach use_covariance als
        PoseStamped oder PoseWithCovarianceStamped.
        """
        now = self.get_clock().now()

        with self._lock:
            cam1 = self._last_cam1
            cam2 = self._last_cam2

        # ------------------------------------------------------------
        # Alter pruefen und valide Schaetzungen sammeln.
        # Tupel-Layout aus process_detections:
        #   (stamp, t, q, weight, var_pos_list, var_rot_list)
        # In valid_estimates speichern wir umsortiert:
        #   (weight, t, q, var_pos_list, var_rot_list)
        # ------------------------------------------------------------
        valid_estimates = []
        latest_stamp = None

        for est in (cam1, cam2):
            if est is None:
                continue
            stamp, t, q, weight, var_pos_list, var_rot_list = est
            age = (now - stamp).nanoseconds * 1e-9
            if age > self.max_estimate_age_s:
                continue
            valid_estimates.append(
                (weight, t, q, var_pos_list, var_rot_list)
            )
            if latest_stamp is None or stamp.nanoseconds > latest_stamp.nanoseconds:
                latest_stamp = stamp

        if not valid_estimates:
            # Nichts zu publizieren. PX4 fuellt diese Luecke per Inertial
            # Dead Reckoning ueberbrueckt, kurze Luecken sind okay.
            return

        # Gewichte normieren
        total_w = sum(w for w, _, _, _, _ in valid_estimates)
        if total_w <= 0.0:
            return

        # Position gewichtet mitteln
        tx = sum(w * t[0] for w, t, _, _, _ in valid_estimates) / total_w
        ty = sum(w * t[1] for w, t, _, _, _ in valid_estimates) / total_w
        tz = sum(w * t[2] for w, t, _, _, _ in valid_estimates) / total_w

        # Quaternion gewichtet mitteln (Vorzeichen-Aligned)
        ref_q = valid_estimates[0][2]
        qx = qy = qz = qw = 0.0
        for w, _, q, _, _ in valid_estimates:
            dot = ref_q[0]*q[0] + ref_q[1]*q[1] + ref_q[2]*q[2] + ref_q[3]*q[3]
            if dot < 0.0:
                q = (-q[0], -q[1], -q[2], -q[3])
            qx += w * q[0]
            qy += w * q[1]
            qz += w * q[2]
            qw += w * q[3]
        q_fused = quat_normalize((qx, qy, qz, qw))

        # EMA-Glaettung
        t_smoothed, q_smoothed = self._apply_ema((tx, ty, tz), q_fused)

        # ------------------------------------------------------------
        # Publish je nach Message-Typ.
        # ------------------------------------------------------------
        if self.use_covariance:
            # --------------------------------------------------------
            # PoseWithCovarianceStamped: Pro-Tag-Varianzen aus ALLEN
            # beitragenden Kameras zu einer einzigen Gesamt-Liste
            # zusammenfuehren.
            #
            # Das ist mathematisch saubererer als pro Kamera separat
            # zu rechnen und dann zu mitteln: Wenn Cam1 9 Inlier hat
            # und Cam2 nur 2, sollen die 9 Tags von Cam1 entsprechend
            # mehr Gewicht bekommen. Die inverse-Varianz-Gewichtung
            # in _combine_variances macht das automatisch korrekt.
            #
            # Bemerkung: cam_weight (cam1_weight/cam2_weight) wird
            # hier bewusst NICHT auf die Varianzen angewendet, weil
            # diese Gewichte fuer die FUSION der Punkt-Schaetzung
            # gedacht sind. Die statistische Unsicherheit haengt
            # nur von der tatsaechlichen Tag-Geometrie ab, nicht von
            # einer benutzerdefinierten Kamera-Bevorzugung.
            # --------------------------------------------------------
            all_var_pos = []
            all_var_rot = []
            for _, _, _, vp_list, vr_list in valid_estimates:
                all_var_pos.extend(vp_list)
                all_var_rot.extend(vr_list)

            covariance = self._estimate_covariance(all_var_pos, all_var_rot)

            msg = PoseWithCovarianceStamped()
            # Stamp = juengste verwendete Schaetzung (echter Aufnahme-
            # zeitpunkt plus Pipeline-Verzoegerung). Wichtig fuer
            # EKF2_EV_DELAY.
            msg.header.stamp = latest_stamp.to_msg()
            msg.header.frame_id = self.map_frame

            msg.pose.pose.position.x = float(t_smoothed[0])
            msg.pose.pose.position.y = float(t_smoothed[1])
            msg.pose.pose.position.z = float(t_smoothed[2])
            msg.pose.pose.orientation.x = float(q_smoothed[0])
            msg.pose.pose.orientation.y = float(q_smoothed[1])
            msg.pose.pose.orientation.z = float(q_smoothed[2])
            msg.pose.pose.orientation.w = float(q_smoothed[3])

            msg.pose.covariance = covariance

            self.pub.publish(msg)

        else:
            # --------------------------------------------------------
            # PoseStamped: Legacy-Modus ohne Kovarianz. PX4 EKF2 nutzt
            # dann feste Werte aus EKF2_EVP_NOISE / EKF2_EVA_NOISE.
            # --------------------------------------------------------
            msg = PoseStamped()
            msg.header.stamp = latest_stamp.to_msg()
            msg.header.frame_id = self.map_frame

            msg.pose.position.x = float(t_smoothed[0])
            msg.pose.position.y = float(t_smoothed[1])
            msg.pose.position.z = float(t_smoothed[2])
            msg.pose.orientation.x = float(q_smoothed[0])
            msg.pose.orientation.y = float(q_smoothed[1])
            msg.pose.orientation.z = float(q_smoothed[2])
            msg.pose.orientation.w = float(q_smoothed[3])

            self.pub.publish(msg)

    def _apply_ema(self, t_new, q_new):
        """Einfacher EMA-Filter auf Position und Rotation."""
        if self._filtered_t is None or self._filtered_q is None:
            self._filtered_t = t_new
            self._filtered_q = q_new
            return t_new, q_new

        a_pos = self.position_alpha
        a_rot = self.rotation_alpha

        self._filtered_t = (
            self._filtered_t[0] + a_pos * (t_new[0] - self._filtered_t[0]),
            self._filtered_t[1] + a_pos * (t_new[1] - self._filtered_t[1]),
            self._filtered_t[2] + a_pos * (t_new[2] - self._filtered_t[2]),
        )

        # SLERP fuer Rotation
        self._filtered_q = self._slerp(self._filtered_q, q_new, a_rot)

        return self._filtered_t, self._filtered_q

    def _slerp(self, q_old, q_new, alpha):
        """Spherical Linear Interpolation."""
        dot = (
            q_old[0]*q_new[0] +
            q_old[1]*q_new[1] +
            q_old[2]*q_new[2] +
            q_old[3]*q_new[3]
        )
        if dot < 0.0:
            q_new = (-q_new[0], -q_new[1], -q_new[2], -q_new[3])
            dot = -dot

        if dot > 0.9995:
            q = (
                q_old[0] + alpha * (q_new[0] - q_old[0]),
                q_old[1] + alpha * (q_new[1] - q_old[1]),
                q_old[2] + alpha * (q_new[2] - q_old[2]),
                q_old[3] + alpha * (q_new[3] - q_old[3]),
            )
            return quat_normalize(q)

        theta_0 = math.acos(dot)
        theta = theta_0 * alpha
        sin_theta = math.sin(theta)
        sin_theta_0 = math.sin(theta_0)

        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        q = (
            s0*q_old[0] + s1*q_new[0],
            s0*q_old[1] + s1*q_new[1],
            s0*q_old[2] + s1*q_new[2],
            s0*q_old[3] + s1*q_new[3],
        )
        return quat_normalize(q)


def main(args=None):
    """Standard-ROS2-Entry-Point."""
    rclpy.init(args=args)
    node = VisionPoseDirectNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()