#!/usr/bin/env python3
# =============================================================================
# Drone-Camera-Kalibrierungs-Node (eigenstaendig, ohne live_lokalisierung)
# -----------------------------------------------------------------------------
# Zweck dieses Nodes:
#   Berechnet die statische Transformation von der Drohnenmitte (drone_center)
#   zu jeder montierten Kamera. Dafuer wird die Drohne unter einer
#   vorab vermessenen AprilTag-Map platziert. Ein zentraler Tag (z.B. id=100)
#   ist starr auf der Drohne montiert und definiert den drone_center-Frame.
#   Die anderen Tags sind in der Welt verteilt (map-Frame).
#
#   NEU: Dieser Node bringt die Live-Lokalisierung selbst mit und ist damit
#   NICHT mehr auf das Paket 'live_lokalisierung' (localization_node)
#   angewiesen. Er abonniert direkt die AprilTag-Detektionen je Kamera
#   (/camera_X/tag_detections, isaac_ros_apriltag_interfaces) und rechnet
#   daraus T_map_camera aus.
#
#   Verwendeter Lokalisierungs-Umfang (bewusste Wahl):
#     - Isaac -> apriltag_ros-Drehkonvention
#     - Pro-Tag-Kameraposen-Schaetzung
#     - gewichtete Fusion mehrerer sichtbarer bekannter Tags
#   NICHT enthalten (im Vergleich zum urspruenglichen localization_node):
#     - Median-basierter Ausreisserfilter
#     - EMA-Glaettung (Tiefpass)
#
# Datenfluss pro Sample:
#   1) AprilTag-Detektionen einer Kamera -> T_map_camera:
#         "Wo sitzt die Kamera, ausgedrueckt im map-Frame?"
#      Diese Pose wird zusaetzlich als PoseStamped (liveposition) und als
#      TF (parent_frame -> camera_frame) veroeffentlicht (zum Mitschauen).
#   2) Aus der Kalibrierungs-Map lesen wir T_map_drone:
#         "Wo sitzt der drone_center-Tag im map-Frame?"
#   3) Wir wollen T_drone_camera (statische Befestigung der Kamera am Rumpf):
#         T_drone_camera = inv(T_map_drone) @ T_map_camera
#   4) Pro Kamera werden viele solche Messungen gesammelt und am Ende
#      gemittelt (Translation arithmetisch, Quaternion vorzeichenrobust).
#
# Ergebnis:
#   Ein YAML mit Translation, Quaternion, Rotationsmatrix und 4x4-Matrix
#   pro aktiver Kamera. Dieses YAML kann spaeter von anderen Nodes als
#   feste Extrinsik (drone -> camera) geladen werden.
# =============================================================================

import math
import os

import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

from tf2_ros import TransformBroadcaster


# -----------------------------------------------------------------------------
# Quaternion-/Pose-Helfer fuer die Live-Lokalisierung.
# (Tupel-basiert, (x, y, z, w). Bewusst frei, damit leicht testbar.)
# -----------------------------------------------------------------------------

def quat_normalize(q):
    """Quaternion (x, y, z, w) auf Einheitslaenge normalisieren.

    Faengt den Sonderfall einer (fast) verschwindenden Norm ab und gibt
    dann die Identitaetsrotation zurueck, um Division durch 0 / NaN zu
    vermeiden.

    Args:
        q: Quaternion als 4-Tupel (x, y, z, w).

    Returns:
        tuple: Normalisiertes Quaternion (x, y, z, w).
    """
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)


def quat_conjugate(q):
    """Konjugiertes (= inverses fuer Einheitsquaternionen) Quaternion bilden.

    Args:
        q: Quaternion als 4-Tupel (x, y, z, w).

    Returns:
        tuple: Konjugiertes Quaternion (-x, -y, -z, w).
    """
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(q1, q2):
    """Zwei Quaternionen (x, y, z, w) multiplizieren (Hamilton-Produkt).

    Args:
        q1: Linkes Quaternion (x, y, z, w).
        q2: Rechtes Quaternion (x, y, z, w).

    Returns:
        tuple: Produkt q1 * q2 als (x, y, z, w).
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def rotate_vector(q, v):
    """Vektor v mit dem Quaternion q rotieren (q * v * q^-1).

    Args:
        q: Rotations-Quaternion (x, y, z, w).
        v: Zu rotierender Vektor als 3-Tupel (x, y, z).

    Returns:
        tuple: Rotierter Vektor als 3-Tupel (x, y, z).
    """
    qv = (v[0], v[1], v[2], 0.0)
    qr = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return (qr[0], qr[1], qr[2])


def transform_inverse(t, q):
    """Inverse einer Pose (Translation t, Rotation q) berechnen.

    Args:
        t: Translation als 3-Tupel (x, y, z).
        q: Rotation als Quaternion (x, y, z, w).

    Returns:
        tuple: (t_inv, q_inv) der inversen Transformation.
    """
    q_inv = quat_conjugate(q)
    t_inv = rotate_vector(q_inv, (-t[0], -t[1], -t[2]))
    return t_inv, q_inv


def transform_multiply(t1, q1, t2, q2):
    """Zwei Posen verketten: (t1, q1) * (t2, q2).

    Args:
        t1: Translation der ersten Pose (x, y, z).
        q1: Rotation der ersten Pose (x, y, z, w).
        t2: Translation der zweiten Pose (x, y, z).
        q2: Rotation der zweiten Pose (x, y, z, w).

    Returns:
        tuple: (t, q) der verketteten Transformation; q ist normalisiert.
    """
    t2_rot = rotate_vector(q1, t2)
    t = (
        t1[0] + t2_rot[0],
        t1[1] + t2_rot[1],
        t1[2] + t2_rot[2],
    )
    q = quat_multiply(q1, q2)
    return t, quat_normalize(q)


def apply_apriltag_ros_rotation(q_isaac):
    """Isaac-Pose (camera -> tag) auf apriltag_ros/TagSLAM-Konvention drehen.

    Es gilt q_out = q_isaac * RotX(pi) mit RotX(pi) = (1, 0, 0, 0), was
    komponentenweise zu (w, z, -y, -x) fuehrt.

    Args:
        q_isaac: Quaternion aus der Isaac-Detektion (x, y, z, w).

    Returns:
        tuple: Umgedrehtes, normalisiertes Quaternion (x, y, z, w).
    """
    x, y, z, w = q_isaac
    return quat_normalize((w, z, -y, -x))


# -----------------------------------------------------------------------------
# Numpy-Helfer rund um Rotation und Pose (fuer Mittelung / YAML-Ausgabe).
# -----------------------------------------------------------------------------

def quat_to_rot(q):
    """Quaternion [x, y, z, w] in 3x3-Rotationsmatrix umrechnen.

    Schritt fuer Schritt:
      1) Quaternion normalisieren (sonst fehlerhaftes R).
      2) Sonderfall abfangen: extrem kleine Norm -> Einheitsrotation
         zurueckgeben (vermeidet Division durch 0).
      3) Standard-Formel fuer R aus (x, y, z, w) anwenden.
    """
    x, y, z, w = q
    n = np.linalg.norm(q)

    # Sicherheitsnetz: bei einem (fast) Null-Quaternion gibt es keine
    # sinnvolle Rotation. Wir geben die Identitaet zurueck, damit der
    # Aufrufer keinen NaN bekommt.
    if n < 1e-12:
        return np.eye(3)

    # Normalisierte Komponenten als Floats; ohne diese Zeile bauen wir die
    # Matrix mit dem unnormalisierten Quaternion und das R waere keine
    # echte Rotationsmatrix (det != 1).
    x, y, z, w = q / n

    # Klassische Direktformel fuer R aus dem Quaternion.
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def rot_to_quat(R):
    """3x3-Rotationsmatrix in Quaternion [x, y, z, w] umrechnen.

    Verwendet die numerisch stabile Variante (Shepperd's Methode):
      - Je nachdem, welches Diagonalelement bzw. die Spur am groessten ist,
        wird ein anderer Zweig benutzt.
      - Damit vermeiden wir Wurzeln aus kleinen oder negativen Zahlen.
    """
    tr = np.trace(R)

    if tr > 0:
        # Spur positiv -> w ist die "groesste" Komponente, stabilster Zweig.
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        # Spur <= 0 -> waehle den Zweig nach groesstem Diagonalelement.
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            # x dominiert.
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            # y dominiert.
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            # z dominiert.
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

    # Rueckgabe normalisieren: kleine numerische Drift wegbuegeln.
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def pose_to_matrix(position, quaternion):
    """Position + Quaternion zu 4x4-Homogentransformation kombinieren.

    Aufbau:
      [ R  t ]
      [ 0  1 ]
    R kommt aus dem Quaternion, t aus der Position.
    """
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(np.array(quaternion, dtype=float))
    T[:3, 3] = np.array(position, dtype=float)
    return T


def average_quaternions(quats):
    """Robuste Mittelung mehrerer Quaternionen.

    Hintergrund:
      q und -q beschreiben dieselbe Rotation (Doppeldeckung der Quaternionen).
      Naive Mittelung kann sich daher gegenseitig ausloeschen.

    Vorgehen:
      1) Referenz waehlen (erstes Quaternion).
      2) Jedes weitere Quaternion ggf. mit -1 multiplizieren, sodass sein
         Skalarprodukt mit der Referenz >= 0 ist. Damit liegen alle
         Quaternionen in derselben Halbsphaere.
      3) Komponentenweise mitteln und das Ergebnis normalisieren.

    Hinweis: Fuer kleine Streuungen (statische Kalibrierung) ist das
    eine vollkommen ausreichende Naeherung an die SLERP-Mittelung.
    """
    if len(quats) == 0:
        # Leere Liste -> Identitaetsrotation als sicherer Fallback.
        return np.array([0.0, 0.0, 0.0, 1.0])

    ref = quats[0]
    aligned = []

    for q in quats:
        q = np.array(q, dtype=float)
        q = q / np.linalg.norm(q)

        # Halbsphaeren-Korrektur: gleiches Vorzeichen wie Referenz.
        if np.dot(ref, q) < 0:
            q = -q

        aligned.append(q)

    # Arithmetisches Mittel + Normalisierung.
    q_avg = np.mean(aligned, axis=0)
    return q_avg / np.linalg.norm(q_avg)


# -----------------------------------------------------------------------------
# Live-Lokalisierung je Kamera (Kernpfad, ohne Outlierfilter / ohne EMA).
# -----------------------------------------------------------------------------

class CameraLocalizer:
    """Berechnet T_map_camera aus AprilTag-Detektionen einer einzelnen Kamera.

    Enthaelt nur den Kernpfad der urspruenglichen Live-Lokalisierung:
      - Pro erkanntem, bekanntem Tag eine Kamerapose schaetzen.
      - Diese Einzelschaetzungen gewichtet fusionieren.
    Bewusst NICHT enthalten: Median-Ausreisserfilter und EMA-Glaettung.
    """

    def __init__(self, map_tags, min_tags_required, min_decision_margin_weight):
        """Localizer mit Kartendaten und Filterschwellen initialisieren.

        Args:
            map_tags: Dict tag_id -> (t, q) der bekannten Tag-Posen im map-Frame.
            min_tags_required: Mindestanzahl gueltiger Einzelschaetzungen,
                damit ueberhaupt eine Pose ausgegeben wird.
            min_decision_margin_weight: Mindestgewicht aus decision_margin
                (bei Isaac oft 0.0, daher Untergrenze).
        """
        self.map_tags = map_tags
        self.min_tags_required = int(min_tags_required)
        self.min_decision_margin_weight = float(min_decision_margin_weight)

    def estimate_camera_pose_from_tag(self, det):
        """Aus einer einzelnen Tag-Detektion eine Kamerapose im map-Frame bilden.

        Args:
            det: Eine AprilTagDetection-Nachricht (Isaac).

        Returns:
            tuple: (weight, t_map_camera, q_map_camera, tag_id, distance).
        """
        tag_id = int(det.id)

        # Bekannte Pose aus Karte: T_map_tag
        t_map_tag, q_map_tag = self.map_tags[tag_id]

        # Aktuelle Messung von Isaac: T_camera_tag
        p = det.pose.pose.pose.position
        q = det.pose.pose.pose.orientation

        t_camera_tag = (
            float(p.x),
            float(p.y),
            float(p.z),
        )

        q_camera_tag_isaac = quat_normalize((
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        ))

        # Auf gleiche Tag-Konvention wie TagSLAM bringen.
        q_camera_tag = apply_apriltag_ros_rotation(q_camera_tag_isaac)

        # Gesucht: T_map_camera = T_map_tag * inverse(T_camera_tag)
        t_tag_camera, q_tag_camera = transform_inverse(
            t_camera_tag,
            q_camera_tag,
        )

        t_map_camera, q_map_camera = transform_multiply(
            t_map_tag,
            q_map_tag,
            t_tag_camera,
            q_tag_camera,
        )

        # Gewichtung:
        # Naeherer Tag bekommt mehr Gewicht, weil seine Pose meist stabiler ist.
        distance = max(math.sqrt(
            t_camera_tag[0]**2 +
            t_camera_tag[1]**2 +
            t_camera_tag[2]**2
        ), 0.001)

        weight = 1.0 / (distance * distance)

        # Falls decision_margin verfuegbar ist, kann sie zusaetzlich als
        # Qualitaetsgewicht dienen. Bei Isaac ist decision_margin je nach
        # Ausgabe oft 0.0; deshalb Mindestgewicht.
        if hasattr(det, "decision_margin"):
            weight *= max(float(det.decision_margin), self.min_decision_margin_weight)

        return (weight, t_map_camera, q_map_camera, tag_id, distance)

    def fuse_estimates(self, estimates):
        """Mehrere Einzelschaetzungen gewichtet zu einer Pose fusionieren.

        Args:
            estimates: Liste von (weight, t, q, tag_id, distance).

        Returns:
            tuple: ((tx, ty, tz), q_fused) der fusionierten Pose.
        """
        total_weight = sum(e[0] for e in estimates)

        tx = sum(w * t[0] for w, t, q, tag_id, distance in estimates) / total_weight
        ty = sum(w * t[1] for w, t, q, tag_id, distance in estimates) / total_weight
        tz = sum(w * t[2] for w, t, q, tag_id, distance in estimates) / total_weight

        # Quaternion-Mittelung:
        # Alle Quaternionen werden auf gleiche Halbkugel gebracht und dann
        # gewichtet gemittelt.
        ref_q = estimates[0][2]
        qx = qy = qz = qw = 0.0

        for w, t, q, tag_id, distance in estimates:
            dot = (
                ref_q[0]*q[0] +
                ref_q[1]*q[1] +
                ref_q[2]*q[2] +
                ref_q[3]*q[3]
            )

            if dot < 0.0:
                q = (-q[0], -q[1], -q[2], -q[3])

            qx += w * q[0]
            qy += w * q[1]
            qz += w * q[2]
            qw += w * q[3]

        q_fused = quat_normalize((qx, qy, qz, qw))

        return (tx, ty, tz), q_fused

    def estimate(self, msg):
        """Aus einem Detektions-Array die fusionierte Kamerapose berechnen.

        Args:
            msg: AprilTagDetectionArray einer Kamera.

        Returns:
            tuple | None: ((tx, ty, tz), q_fused) im map-Frame, oder None,
            wenn weniger als min_tags_required bekannte Tags sichtbar sind.
        """
        raw_estimates = []

        for det in msg.detections:
            tag_id = int(det.id)

            if tag_id not in self.map_tags:
                continue

            raw_estimates.append(self.estimate_camera_pose_from_tag(det))

        if len(raw_estimates) < self.min_tags_required:
            return None

        # Gewichtete Fusion aller sichtbaren bekannten Tags (kein Outlierfilter,
        # keine EMA-Glaettung).
        return self.fuse_estimates(raw_estimates)


# -----------------------------------------------------------------------------
# Eigentlicher ROS2-Node.
# -----------------------------------------------------------------------------

class DroneCameraCalibrationNode(Node):

    def __init__(self):
        """Node initialisieren, Lokalisierung aufsetzen und Kameras abonnieren.

        Reihenfolge der Initialisierung:
          1) Parameter deklarieren (Defaults setzen).
          2) Parameter auslesen, Pflichtfelder pruefen.
          3) Map-YAML einlesen: bekannte Tag-Posen (fuer Lokalisierung) und
             drone_center-Pose (fuer die Kalibrierung).
          4) Kamera-Konfigurationen bauen, deaktivierte ausfiltern.
          5) Plausibilitaetschecks (mind. 1 Kamera, eindeutige Namen).
          6) Pro Kamera: Localizer, liveposition-Publisher, Detektions-Sub.
          7) Status loggen, damit man im Terminal sieht, was aktiv ist.
        """
        super().__init__('drone_camera_calibration_node')

        # --- (1) Parameter deklarieren -------------------------------------
        # Pfade und Tag-ID werden ueblicherweise im Launch-File gesetzt.
        self.declare_parameter('map_file', '')
        self.declare_parameter('output_file', '')
        self.declare_parameter('drone_center_tag_id', 100)

        # samples_required: wie viele Messungen pro Kamera, bevor wir
        # die Kalibrierung als "fertig" werten.
        # write_every_sample: True = laufend rausschreiben (gut zum Mitlesen
        # waehrend der Aufnahme); False = nur einmal am Ende speichern.
        self.declare_parameter('samples_required', 100)
        self.declare_parameter('write_every_sample', True)

        # Lokalisierungs-Parameter (global fuer alle Kameras).
        # parent_frame: Bezugsframe der Karte (Header der liveposition + TF-Parent).
        self.declare_parameter('parent_frame', 'tag_map')
        self.declare_parameter('min_tags_required', 1)
        self.declare_parameter('min_decision_margin_weight', 1.0)

        # Pro Kamera-Slot: enabled (an/aus), semantischer Name
        # ("front"/"bottom"), Eingangs-Topic der Detektionen, Ausgangs-Topic
        # der liveposition sowie der camera_frame (TF-Child).
        self.declare_parameter('camera_1_enabled', True)
        self.declare_parameter('camera_1_name', 'front')
        self.declare_parameter('camera_1_detections_topic', '/camera_1/tag_detections')
        self.declare_parameter('camera_1_pose_topic', '/liveposition_camera_1')
        self.declare_parameter('camera_1_frame', 'camera_1_optical_frame')

        self.declare_parameter('camera_2_enabled', True)
        self.declare_parameter('camera_2_name', 'bottom')
        self.declare_parameter('camera_2_detections_topic', '/camera_2/tag_detections')
        self.declare_parameter('camera_2_pose_topic', '/liveposition_camera_2')
        self.declare_parameter('camera_2_frame', 'camera_2_optical_frame')

        # --- (2) Parameter auslesen ----------------------------------------
        self.map_file = self.get_parameter('map_file').value
        self.output_file = self.get_parameter('output_file').value
        self.drone_center_tag_id = int(self.get_parameter('drone_center_tag_id').value)

        self.samples_required = int(self.get_parameter('samples_required').value)
        self.write_every_sample = bool(self.get_parameter('write_every_sample').value)

        self.parent_frame = self.get_parameter('parent_frame').value
        self.min_tags_required = int(self.get_parameter('min_tags_required').value)
        self.min_decision_margin_weight = float(
            self.get_parameter('min_decision_margin_weight').value
        )

        # Pflichtfelder hart abbrechen, statt spaeter mit einem kryptischen
        # IOError bei open() abzustuerzen.
        if not self.map_file:
            raise RuntimeError('Parameter map_file ist leer.')

        if not self.output_file:
            raise RuntimeError('Parameter output_file ist leer.')

        # --- (3) Map-Daten laden -------------------------------------------
        # a) Bekannte Tag-Posen fuer die Live-Lokalisierung.
        self.map_tags = self.load_map(self.map_file)

        # b) drone_center-Pose fuer die Kalibrierung. Wir cachen die 4x4-Matrix,
        #    damit wir in jedem Callback nur eine Matrix-Multiplikation
        #    ausfuehren und nicht das YAML erneut parsen muessen.
        self.T_map_drone = self.load_drone_center_pose(self.map_file)

        # --- (4) Kamera-Konfigurationen aufbauen ---------------------------
        # Liste statt zwei Hardcoded-Bloecke: macht Erweiterung auf weitere
        # Kameras spaeter trivial und vermeidet duplizierten Code.
        camera_configs = [
            {
                'slot': 'camera_1',
                'enabled': bool(self.get_parameter('camera_1_enabled').value),
                'name': self.get_parameter('camera_1_name').value,
                'detections_topic': self.get_parameter('camera_1_detections_topic').value,
                'pose_topic': self.get_parameter('camera_1_pose_topic').value,
                'camera_frame': self.get_parameter('camera_1_frame').value,
            },
            {
                'slot': 'camera_2',
                'enabled': bool(self.get_parameter('camera_2_enabled').value),
                'name': self.get_parameter('camera_2_name').value,
                'detections_topic': self.get_parameter('camera_2_detections_topic').value,
                'pose_topic': self.get_parameter('camera_2_pose_topic').value,
                'camera_frame': self.get_parameter('camera_2_frame').value,
            },
        ]

        # Nur enabled=True wird abonniert und gewertet.
        self.active_cameras = [c for c in camera_configs if c['enabled']]

        # --- (5) Plausibilitaetschecks -------------------------------------
        # Ohne aktive Kamera ist der Node sinnlos -> sofort abbrechen.
        if len(self.active_cameras) == 0:
            raise RuntimeError(
                'Keine Kamera aktiviert. Mindestens eine der Optionen '
                'camera_1_enabled oder camera_2_enabled muss True sein.'
            )

        # Doppelter Name (z.B. beide "front") wuerde sich im YAML
        # gegenseitig ueberschreiben -> hart abbrechen.
        names = [c['name'] for c in self.active_cameras]
        if len(set(names)) != len(names):
            raise RuntimeError(
                f'Doppelter camera_*_name in aktivierten Kameras: {names}. '
                'Bitte eindeutige Namen vergeben (z.B. "front", "bottom").'
            )

        # --- (6) Localizer, Publisher, Subscriptions -----------------------
        # Indexieren ueber den semantischen Namen (nicht ueber den Slot),
        # damit die Datenstruktur direkt zum Ausgabe-YAML passt.
        self.samples = {c['name']: [] for c in self.active_cameras}

        # Ein TF-Broadcaster fuer alle Kameras (unterschiedliche child_frames).
        self.tf_broadcaster = TransformBroadcaster(self)

        # Referenzen halten (Garbage-Collection-Schutz); ohne Referenz wuerde
        # rclpy Subscriptions/Publisher unter Umstaenden verwerfen.
        self.subscriptions_list = []
        self.pose_publishers = {}
        self.localizers = {}

        for cfg in self.active_cameras:
            name = cfg['name']

            # Eigener Localizer je Kamera (teilt sich die gemeinsame Karte).
            localizer = CameraLocalizer(
                map_tags=self.map_tags,
                min_tags_required=self.min_tags_required,
                min_decision_margin_weight=self.min_decision_margin_weight,
            )
            self.localizers[name] = localizer

            # liveposition-Publisher (zum Mitschauen in RViz o.ae.).
            pose_pub = self.create_publisher(PoseStamped, cfg['pose_topic'], 10)
            self.pose_publishers[name] = pose_pub

            # Detektions-Subscription. Default-Argumente im Lambda binden den
            # Kamera-Kontext frueh; sonst wuerden alle Callbacks am Ende
            # denselben (letzten) Kontext verwenden.
            sub = self.create_subscription(
                AprilTagDetectionArray,
                cfg['detections_topic'],
                lambda msg, n=name, loc=localizer, pub=pose_pub, cf=cfg['camera_frame']:
                    self.detection_callback(msg, n, loc, pub, cf),
                10,  # Queue-Tiefe; 10 ist fuer langsame Kalibrierung mehr als genug.
            )
            self.subscriptions_list.append(sub)

        # --- (7) Status loggen ---------------------------------------------
        # Hilft, beim Start im Terminal direkt zu sehen, welche Map,
        # welche Tag-ID und welche Kameras tatsaechlich verwendet werden.
        self.get_logger().info('Drone-Camera-Kalibrierung gestartet.')
        self.get_logger().info(f'Map-Datei: {self.map_file}')
        self.get_logger().info(f'Ausgabe-Datei: {self.output_file}')
        self.get_logger().info(
            f'Drohnenmittelpunkt Tag-ID: {self.drone_center_tag_id}'
        )
        self.get_logger().info(f'Bekannte Tags (Lokalisierung): {sorted(self.map_tags.keys())}')
        self.get_logger().info(
            f'Lokalisierung: min_tags_required={self.min_tags_required}, '
            f'min_decision_margin_weight={self.min_decision_margin_weight} '
            f'(ohne Outlierfilter, ohne EMA)'
        )

        for cfg in self.active_cameras:
            self.get_logger().info(
                f"Aktive Kamera: name='{cfg['name']}' (slot={cfg['slot']}, "
                f"detections={cfg['detections_topic']}, "
                f"liveposition={cfg['pose_topic']}, frame={cfg['camera_frame']})"
            )

        # Deaktivierte Kameras ebenfalls loggen -> hilfreich, wenn man sich
        # spaeter wundert, warum nur eine Kamera Samples liefert.
        disabled = [c for c in camera_configs if not c['enabled']]
        for cfg in disabled:
            self.get_logger().info(
                f"Deaktivierte Kamera: slot={cfg['slot']} "
                f"(name='{cfg['name']}', detections={cfg['detections_topic']})"
            )

    def load_map(self, path):
        """Bekannte Tag-Posen aus der Map-YAML fuer die Lokalisierung laden.

        Args:
            path: Pfad zur Map-YAML.

        Returns:
            dict: tag_id -> (t, q) mit t=(x, y, z) und q=(x, y, z, w).

        Raises:
            FileNotFoundError: Wenn die Datei nicht existiert.
        """
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f"Map file not found: {path}")

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        tags = {}

        for tag in data.get("tags", []):
            tag_id = int(tag["id"])
            pose = tag["pose"]

            p = pose["position"]
            r = pose["rotation"]

            t = (
                float(p["x"]),
                float(p["y"]),
                float(p["z"]),
            )

            q = quat_normalize((
                float(r["x"]),
                float(r["y"]),
                float(r["z"]),
                float(r["w"]),
            ))

            tags[tag_id] = (t, q)

        return tags

    def load_drone_center_pose(self, path):
        """Pose des Drohnenmittelpunkt-Tags aus der Map-YAML lesen.

        Erwartetes Format (vereinfacht):
          tags:
            - id: 100
              pose:
                position: {x, y, z}
                rotation: {x, y, z, w}

        Vorgehen:
          1) Datei einlesen.
          2) Liste 'tags' durchgehen.
          3) Beim Treffer (id == drone_center_tag_id) die Pose in eine
             4x4-Matrix wandeln und zurueckgeben.
          4) Kein Treffer -> hart abbrechen, denn ohne diese Pose koennen
             wir die Transformation drone -> camera nicht bilden.
        """
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        tags = data.get('tags', [])

        for tag in tags:
            if int(tag.get('id')) == self.drone_center_tag_id:
                pose = tag['pose']
                pos = pose['position']
                rot = pose['rotation']

                T = pose_to_matrix(
                    [pos['x'], pos['y'], pos['z']],
                    [rot['x'], rot['y'], rot['z'], rot['w']]
                )

                self.get_logger().info(
                    f'Drohnenmittelpunkt aus YAML geladen: '
                    f'Tag-ID {self.drone_center_tag_id}'
                )

                return T

        # Wenn wir hier landen, gibt es schlicht keine Datenbasis fuer
        # die Kalibrierung -> RuntimeError, kein stilles Weitermachen.
        raise RuntimeError(
            f'Drohnenmittelpunkt mit id={self.drone_center_tag_id} '
            f'nicht in {path} gefunden.'
        )

    def detection_callback(self, msg, camera_name, localizer, pose_pub, camera_frame):
        """Detektionen -> Kamerapose bestimmen, publishen und Sample sammeln.

        Wird fuer JEDES eingehende AprilTagDetectionArray genau einer Kamera
        aufgerufen. Schritte:
          1) Aus den Detektionen T_map_camera schaetzen (Kernpfad-Lokalisierung).
          2) Ohne gueltige Pose (zu wenige bekannte Tags) frueh abbrechen.
          3) liveposition (PoseStamped) und TF (parent_frame -> camera_frame)
             veroeffentlichen -- unabhaengig vom Sample-Stand, damit RViz
             auch nach Abschluss der Kalibrierung eine Live-Pose sieht.
          4) Nur solange noch Samples fehlen: T_map_camera in drone_center-Frame
             transformieren, sammeln und ggf. YAML schreiben.

        Args:
            msg: AprilTagDetectionArray der Kamera.
            camera_name: Semantischer Kameraname ("front"/"bottom").
            localizer: Zugehoeriger CameraLocalizer.
            pose_pub: liveposition-Publisher dieser Kamera.
            camera_frame: TF-Child-Frame dieser Kamera.
        """
        # (1) Lokalisierung: Kamerapose im map-Frame.
        result = localizer.estimate(msg)

        # (2) Keine gueltige Pose -> nichts zu tun.
        if result is None:
            return

        (t_map_camera, q_map_camera) = result
        stamp = msg.header.stamp

        # (3) liveposition + TF veroeffentlichen (Monitoring, immer).
        self.publish_pose(pose_pub, stamp, t_map_camera, q_map_camera)
        self.publish_tf(stamp, t_map_camera, q_map_camera, camera_frame)

        # (4) Solange das Sample-Limit nicht erreicht ist: Kalibrier-Sample
        # bilden. So bleibt der Mittelwert stabil und die Datei wird nicht
        # endlos weitergeschrieben.
        if len(self.samples[camera_name]) >= self.samples_required:
            return

        # T_map_camera als 4x4 (identisch zur bisherigen PoseStamped -> Matrix).
        T_map_camera = pose_to_matrix(t_map_camera, q_map_camera)

        # Umrechnen in den drone_center-Frame. inv(T_map_drone) ist
        # rechnerisch billig (4x4) und numerisch hier voellig unkritisch.
        # Mathematisch:  drone <- map  *  map <- camera  =  drone <- camera
        T_drone_camera = np.linalg.inv(self.T_map_drone) @ T_map_camera

        # Anhaengen und Fortschritt loggen.
        self.samples[camera_name].append(T_drone_camera)

        n = len(self.samples[camera_name])

        self.get_logger().info(
            f'{camera_name}: Sample {n}/{self.samples_required}'
        )

        # Speicher-Strategie umsetzen.
        if self.write_every_sample or self.all_done():
            self.write_result()

    def publish_pose(self, pose_pub, stamp, t, q):
        """Kamerapose als PoseStamped (liveposition) veroeffentlichen.

        Args:
            pose_pub: Publisher dieser Kamera.
            stamp: Zeitstempel (aus dem Detektions-Header).
            t: Translation (x, y, z) im parent_frame.
            q: Rotation (x, y, z, w) im parent_frame.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.parent_frame

        pose_msg.pose.position.x = t[0]
        pose_msg.pose.position.y = t[1]
        pose_msg.pose.position.z = t[2]

        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        pose_pub.publish(pose_msg)

    def publish_tf(self, stamp, t, q, camera_frame):
        """Kamerapose als TF parent_frame -> camera_frame senden.

        Args:
            stamp: Zeitstempel (aus dem Detektions-Header).
            t: Translation (x, y, z) im parent_frame.
            q: Rotation (x, y, z, w) im parent_frame.
            camera_frame: Child-Frame dieser Kamera.
        """
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = camera_frame

        tf_msg.transform.translation.x = t[0]
        tf_msg.transform.translation.y = t[1]
        tf_msg.transform.translation.z = t[2]

        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tf_msg)

    def average_transform(self, transforms):
        """Translationen arithmetisch, Quaternionen vorzeichenrobust mitteln.

        Schritt fuer Schritt:
          - Aus jeder 4x4-Matrix Translation und Quaternion extrahieren.
          - Translation: einfache komponentenweise Mittelung (np.mean).
          - Quaternion:  average_quaternions (Halbsphaeren-aware).
          - R aus dem gemittelten q neu bauen, damit die Matrix wirklich
            zum Quaternion passt (statt eine Mittelung von R-Eintraegen
            zu machen, die i.A. KEINE gueltige Rotationsmatrix mehr waere).
          - 4x4-Endmatrix zusammensetzen.
        """
        translations = []
        quaternions = []

        for T in transforms:
            translations.append(T[:3, 3])
            quaternions.append(rot_to_quat(T[:3, :3]))

        t_avg = np.mean(translations, axis=0)
        q_avg = average_quaternions(quaternions)
        R_avg = quat_to_rot(q_avg)

        T_avg = np.eye(4)
        T_avg[:3, :3] = R_avg
        T_avg[:3, 3] = t_avg

        return T_avg, t_avg, q_avg, R_avg

    def all_done(self):
        """True, wenn alle AKTIVEN Kameras die geforderten Samples haben.

        Wichtig: nur aktive Kameras werden betrachtet. Falls
        camera_2 deaktiviert ist, ist die Kalibrierung fertig, sobald
        camera_1 ihre Samples voll hat.
        """
        for cfg in self.active_cameras:
            if len(self.samples[cfg['name']]) < self.samples_required:
                return False
        return True

    def matrix_to_list(self, M):
        """Numpy-Matrix in einfache Python-Listen fuer YAML-Ausgabe wandeln.

        Hintergrund: yaml.safe_dump kann numpy-Skalare/-Arrays nicht
        zuverlaessig serialisieren. Wir wandeln alles explizit in float
        und verschachtelte Python-Listen.
        """
        return [[float(v) for v in row] for row in M]

    def write_result(self):
        """Aktuelle Ergebnisse in die Ausgabe-YAML schreiben.

        Wird je nach Konfiguration nach jedem Sample oder nur am Ende
        aufgerufen. Schreibt IMMER den vollstaendigen aktuellen Stand,
        d.h. die Datei ist nach jedem Aufruf in sich konsistent.

        Aufbau:
          - parent_frame: drone_center
          - cameras: dict mit semantischem Namen ("front", "bottom") als Key
              -> translation, rotation_quaternion, rotation_matrix,
                 transform_matrix_4x4 (alles redundant, fuer maximale
                 Bequemlichkeit beim Konsumieren).
        """
        # Liste der aktiven Kameras separat ausweisen, damit Folge-Nodes
        # ohne zusaetzlichen Kontext sehen, welche Kameras erfasst wurden.
        active_names = [c['name'] for c in self.active_cameras]

        output = {
            'parent_frame': 'drone_center',
            'description': (
                'Gemittelte Transformationen vom Drohnenmittelpunkt '
                'zu den Kameras'
            ),
            'samples_required': self.samples_required,
            'active_cameras': active_names,
            'cameras': {}
        }

        # Pro Kamera mitteln und Block bauen.
        for camera_name, transforms in self.samples.items():
            # Kameras ohne Samples ueberspringen (z.B. wenn der Topic noch
            # nichts geliefert hat). So enthaelt die Datei nur "echte" Eintraege.
            if len(transforms) == 0:
                continue

            T_avg, t_avg, q_avg, R_avg = self.average_transform(transforms)

            output['cameras'][camera_name] = {
                'samples_used': len(transforms),
                # child_frame: ueblicher TF-Name; passt zu Konventionen wie
                # 'front_optical_frame' / 'bottom_optical_frame'.
                'child_frame': f'{camera_name}_optical_frame',
                'translation': {
                    'x': float(t_avg[0]),
                    'y': float(t_avg[1]),
                    'z': float(t_avg[2]),
                },
                'rotation_quaternion': {
                    'x': float(q_avg[0]),
                    'y': float(q_avg[1]),
                    'z': float(q_avg[2]),
                    'w': float(q_avg[3]),
                },
                # Sowohl 3x3-Rotation als auch 4x4-Transformation mit ablegen,
                # damit nachgelagerte Nodes sich aussuchen koennen, was sie
                # brauchen (z.B. fuer numpy-Matrix-Anwendung direkt).
                'rotation_matrix': self.matrix_to_list(R_avg),
                'transform_matrix_4x4': self.matrix_to_list(T_avg),
            }

        # Datei atomar genug fuer Kalibrierung: ein einzelner Open+Dump.
        # Fuer wirklich atomares Schreiben muesste man in Temp + os.replace,
        # aber bei statischer Kalibrierung ist das uebertrieben.
        with open(self.output_file, 'w') as f:
            yaml.safe_dump(output, f, sort_keys=False)

        # Eindeutiges "Fertig"-Signal, damit der Benutzer im Terminal sieht,
        # dass er den Node jetzt beenden kann.
        if self.all_done():
            self.get_logger().info(
                'Kalibrierung abgeschlossen. YAML wurde final geschrieben.'
            )


def main(args=None):
    """Einstiegspunkt: rclpy initialisieren und Node spinnen.

    Lebenszyklus:
      1) rclpy.init: ROS2-Client-Library hochfahren.
      2) Node instanziieren (Parameter, Subscriptions, Map-Load).
      3) spin: blockiert, bis Ctrl+C / Shutdown -> verarbeitet Callbacks.
      4) Aufraeumen: Node destroyen und rclpy herunterfahren.
    """
    rclpy.init(args=args)
    node = DroneCameraCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
