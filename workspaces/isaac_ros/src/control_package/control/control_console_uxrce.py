#!/usr/bin/env python3
#
# control_console.py  -- uXRCE-DDS-Variante
#
# Curses-Bedienkonsole, die DIREKT mit PX4 ueber uXRCE-DDS spricht
# (kein MAVROS). Prinzipieller Aufbau identisch zur MAVROS-Version:
#
#   - Zeigt Fahrzeug-Status (armed, nav_state) und aktuelle Position.
#   - Zeigt, ob die Kameras Bilder liefern (gscam-Topics, unveraendert).
#   - Publiziert kontinuierlich Position-Setpoints.
#   - SPACE armt; wenn bereits armed, startet eine Landung an Ort und
#     Stelle (Offboard-Sinkflug: X/Y/Yaw eingefroren, Z langsam abgesenkt,
#     Disarm bei Bodenkontakt). KEIN PX4-AUTO.LAND, daher kein Konflikt
#     mit dem laufenden Offboard-Stream (kein Motor-Ruckeln).
#   - RC-AUTO.LAND bleibt als Notfall nutzbar: sobald PX4 NICHT mehr im
#     Offboard-Modus ist, schweigt die Konsole vollstaendig und uebergibt
#     sauber an den Piloten/Autopiloten.
#   - WASD / O,L / Q,E verschieben den Setpoint (nur wenn armed).
#
# Unterschiede zur MAVROS-Version (durch uXRCE bedingt):
#   - Status:    /fmu/out/vehicle_status_v1     (statt /mavros/state)
#   - Position:  /fmu/out/vehicle_local_position (statt local_position/pose)
#   - Setpoint:  /fmu/in/trajectory_setpoint  + /fmu/in/offboard_control_mode
#                (statt /mavros/setpoint_position/local)
#   - Arm/Land:  /fmu/in/vehicle_command       (statt MAVROS-Services)
#   - Frames:    PX4 arbeitet in NED/FRD. Die Konsole denkt intern in
#                ENU (z nach oben, wie die tag_map). Setpoints werden vor
#                dem Senden ENU->NED gedreht, die Anzeige-Position NED->ENU.
#
# Wichtig (wie bisher): Der Offboard-Mode wird NICHT von der Konsole
# gesetzt. Du schaltest weiter extern (RC / QGroundControl) in den
# Offboard-Mode. Die Konsole streamt aber von Anfang an kontinuierlich
# OffboardControlMode + TrajectorySetpoint, damit PX4 den Offboard-Wechsel
# ueberhaupt akzeptiert (PX4 verlangt diesen Stream mit >=2 Hz, bevor man
# nach Offboard schalten kann).

import curses
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import Image

from px4_msgs.msg import (
    VehicleStatus,
    VehicleLocalPosition,
    VehicleLandDetected,
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
)


# =========================================================================
# Hilfsfunktionen
# =========================================================================

def quat_to_rpy(x, y, z, w):
    """Quaternion (x, y, z, w) in Roll, Pitch, Yaw [rad] umrechnen."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_angle(a):
    """Winkel auf den Bereich (-pi, pi] normalisieren."""
    return math.atan2(math.sin(a), math.cos(a))


# --- Frame-Konvertierung ENU (ROS, z oben) <-> NED (PX4, z unten) --------
#
# Position: der Achsentausch ist seine eigene Inverse.
#   ENU (x, y, z) -> NED (y, x, -z)
#   NED (x, y, z) -> ENU (y, x, -z)
#
# Yaw: ENU misst CCW von Ost, NED misst CW von Nord.
#   yaw_ned = pi/2 - yaw_enu   (und umgekehrt, ebenfalls self-inverse)

def enu_pos_to_ned(x, y, z):
    """Position von ENU nach NED."""
    return y, x, -z


def ned_pos_to_enu(x, y, z):
    """Position von NED nach ENU."""
    return y, x, -z


def yaw_enu_to_ned(yaw_enu):
    """Yaw von ENU nach NED."""
    return wrap_angle(math.pi / 2.0 - yaw_enu)


def yaw_ned_to_enu(yaw_ned):
    """Yaw von NED nach ENU."""
    return wrap_angle(math.pi / 2.0 - yaw_ned)


# Lesbare Namen fuer nav_state, versionssicher aus den Message-Konstanten
# aufgebaut (funktioniert unabhaengig davon, welche Nummern die jeweilige
# PX4-Version vergibt).
_NAV_STATE_NAMES = {}
for _name in dir(VehicleStatus):
    if _name.startswith("NAVIGATION_STATE_"):
        _NAV_STATE_NAMES[getattr(VehicleStatus, _name)] = _name.replace(
            "NAVIGATION_STATE_", ""
        )


# =========================================================================
# Hauptknoten
# =========================================================================

class ControlConsole(Node):
    def __init__(self):
        """ROS2-Node fuer die uXRCE-DDS-Bedienkonsole initialisieren."""
        super().__init__("control_console")

        # ------------------------------------------------------------
        # Topic-Parameter (uXRCE-DDS / PX4)
        # ------------------------------------------------------------
        self.declare_parameter("status_topic", "/fmu/out/vehicle_status_v1")
        self.declare_parameter(
            "local_position_topic", "/fmu/out/vehicle_local_position"
        )
        self.declare_parameter(
            "trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint"
        )
        self.declare_parameter(
            "offboard_control_mode_topic", "/fmu/in/offboard_control_mode"
        )
        self.declare_parameter(
            "vehicle_command_topic", "/fmu/in/vehicle_command"
        )
        self.declare_parameter(
            "land_detected_topic", "/fmu/out/vehicle_land_detected"
        )

        # Kamera-Topics (gscam, unveraendert gegenueber MAVROS-Version)
        self.declare_parameter("camera_front_raw_topic", "/camera_1/image_raw")
        self.declare_parameter("camera_front_rect_topic", "/camera_1/image_rect")
        self.declare_parameter("camera_down_raw_topic", "/camera_2/image_raw")
        self.declare_parameter("camera_down_rect_topic", "/camera_2/image_rect")

        # PX4 target/source IDs fuer VehicleCommand
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("source_system", 1)
        self.declare_parameter("source_component", 1)

        # Raten und Timeouts
        self.declare_parameter("display_rate_hz", 2.0)
        self.declare_parameter("setpoint_publish_rate_hz", 20.0)
        self.declare_parameter("camera_timeout_s", 1.0)
        self.declare_parameter("status_timeout_s", 2.0)
        self.declare_parameter("pose_timeout_s", 2.0)

        # Schrittweiten
        self.declare_parameter("xy_step_m", 0.05)
        self.declare_parameter("z_step_m", 0.05)
        self.declare_parameter("yaw_step_deg", 5.0)

        # Hoehenoffset ueber der aktuellen Position, solange nicht armed.
        # Dies ist zugleich die Start-/Flughoehe, auf die die Drohne beim
        # Armen im Offboard-Modus steigt (ueber Launch-File anpassbar).
        self.declare_parameter("takeoff_offset_m", 1.5)

        # ------------------------------------------------------------
        # Demo-Modus: autonom ein Quadrat abfliegen
        # ------------------------------------------------------------
        # Die Drohne faehrt mit konstanter Geschwindigkeit zur naechst-
        # gelegenen Ecke eines Quadrats, fliegt es einmal ab und schwebt
        # danach an dieser Startecke. Yaw und Hoehe bleiben fest. Da der
        # Node Positions-Setpoints sendet (keine Geschwindigkeit), wird die
        # "Geschwindigkeit" dadurch umgesetzt, dass der Ziel-Setpoint pro
        # Tick nur um speed * dt weitergeschoben wird.
        #
        # Kantenlaenge des Quadrats in Metern.
        self.declare_parameter("demo_edge_length_m", 2.0)
        # Fluggeschwindigkeit im Demo-Modus in m/s (bewusst langsam halten).
        self.declare_parameter("demo_speed_mps", 0.3)
        # Zentrum des Quadrats in ENU. Default (0, 0) = Mitte der Karte
        # (tag_map-Ursprung).
        self.declare_parameter("demo_center_x", 0.0)
        self.declare_parameter("demo_center_y", 0.0)

        # ------------------------------------------------------------
        # Landung an Ort und Stelle (Offboard-Sinkflug)
        # ------------------------------------------------------------
        # Statt PX4-AUTO.LAND (das mit dem laufenden Offboard-Stream
        # kollidiert) sinkt die Drohne innerhalb des Offboard-Modus:
        # X/Y/Yaw werden eingefroren, nur der Z-Sollwert wird langsam
        # abgesenkt. So bleibt EINE Kommandoquelle aktiv -> kein Ruckeln.
        #
        # Sinkgeschwindigkeit in m/s (Default: 0.3). Ueber Launch-File
        # einstellbar. Das Landen per Leertaste nutzt genau diesen Wert.
        self.declare_parameter("descent_speed_mps", 0.3)

        # Sicherheits-Untergrenze (ENU-z, Meter). Falls die Bodenkontakt-
        # Erkennung (vehicle_land_detected) ausbleibt, wird beim Erreichen
        # dieser Hoehe trotzdem disarmt. Bezogen auf den tag_map-Ursprung.
        # Etwas unter Bodenniveau waehlen, damit die Erkennung normal
        # zuerst greift.
        self.declare_parameter("land_min_z_m", -0.3)

        # Fallback-Sollwerte (ENU), solange noch keine Pose empfangen wurde.
        self.declare_parameter("start_x", -0.2030704)
        self.declare_parameter("start_y", -0.1153477)
        self.declare_parameter("start_z", 1.36)
        self.declare_parameter("start_qx", -0.0012295)
        self.declare_parameter("start_qy", 0.00490711)
        self.declare_parameter("start_qz", -0.7050490)
        self.declare_parameter("start_qw", -0.7091406)

        # ------------------------------------------------------------
        # Parameter auslesen
        # ------------------------------------------------------------
        self.status_topic = self.get_parameter("status_topic").value
        self.local_position_topic = self.get_parameter(
            "local_position_topic"
        ).value
        self.trajectory_setpoint_topic = self.get_parameter(
            "trajectory_setpoint_topic"
        ).value
        self.offboard_control_mode_topic = self.get_parameter(
            "offboard_control_mode_topic"
        ).value
        self.vehicle_command_topic = self.get_parameter(
            "vehicle_command_topic"
        ).value
        self.land_detected_topic = self.get_parameter(
            "land_detected_topic"
        ).value

        self.camera_front_raw_topic = self.get_parameter(
            "camera_front_raw_topic"
        ).value
        self.camera_front_rect_topic = self.get_parameter(
            "camera_front_rect_topic"
        ).value
        self.camera_down_raw_topic = self.get_parameter(
            "camera_down_raw_topic"
        ).value
        self.camera_down_rect_topic = self.get_parameter(
            "camera_down_rect_topic"
        ).value

        self.target_system = int(self.get_parameter("target_system").value)
        self.target_component = int(
            self.get_parameter("target_component").value
        )
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(
            self.get_parameter("source_component").value
        )

        self.display_rate_hz = float(self.get_parameter("display_rate_hz").value)
        self.setpoint_publish_rate_hz = float(
            self.get_parameter("setpoint_publish_rate_hz").value
        )
        self.camera_timeout_s = float(self.get_parameter("camera_timeout_s").value)
        self.status_timeout_s = float(self.get_parameter("status_timeout_s").value)
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)

        self.xy_step_m = float(self.get_parameter("xy_step_m").value)
        self.z_step_m = float(self.get_parameter("z_step_m").value)
        self.yaw_step_rad = math.radians(
            float(self.get_parameter("yaw_step_deg").value)
        )
        self.takeoff_offset_m = float(self.get_parameter("takeoff_offset_m").value)
        self.demo_edge_length_m = float(
            self.get_parameter("demo_edge_length_m").value
        )
        self.demo_speed_mps = float(self.get_parameter("demo_speed_mps").value)
        self.demo_center_x = float(self.get_parameter("demo_center_x").value)
        self.demo_center_y = float(self.get_parameter("demo_center_y").value)
        self.descent_speed_mps = float(
            self.get_parameter("descent_speed_mps").value
        )
        self.land_min_z_m = float(self.get_parameter("land_min_z_m").value)

        # ------------------------------------------------------------
        # Zustand
        # ------------------------------------------------------------
        self.vehicle_status = None
        self.local_position = None
        self.vehicle_land_detected = None

        # Landung an Ort und Stelle aktiv?
        self._landing = False
        self.last_land_detected_time = 0.0

        # Demo-Modus (Quadrat abfliegen) aktiv?
        self._demo_active = False
        # Liste der anzufliegenden Ecken (ENU x, y) in Reihenfolge.
        self._demo_waypoints = []
        # Index des aktuell angesteuerten Wegpunkts.
        self._demo_index = 0
        # Waehrend der Demo eingefrorene Hoehe und Yaw.
        self._demo_z = 0.0
        self._demo_yaw = 0.0

        self.last_status_time = 0.0
        self.last_pose_time = 0.0

        self.last_front_raw = 0.0
        self.last_front_rect = 0.0
        self.last_down_raw = 0.0
        self.last_down_rect = 0.0

        # Sollwerte intern in ENU (z oben), wie die tag_map.
        self.target_x = float(self.get_parameter("start_x").value)
        self.target_y = float(self.get_parameter("start_y").value)
        self.target_z = float(self.get_parameter("start_z").value)

        qx = float(self.get_parameter("start_qx").value)
        qy = float(self.get_parameter("start_qy").value)
        qz = float(self.get_parameter("start_qz").value)
        qw = float(self.get_parameter("start_qw").value)
        _, _, self.target_yaw = quat_to_rpy(qx, qy, qz, qw)

        self._was_armed = False

        # ------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------
        # PX4-Topics (in & out) laufen ueber uXRCE mit BEST_EFFORT. Mit
        # BEST_EFFORT als Subscriber bleiben wir kompatibel zu BEST_EFFORT-
        # und RELIABLE-Publishern. Fuer die Publisher Richtung PX4 ist
        # BEST_EFFORT Pflicht, sonst nimmt die Bruecke nichts an.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------
        self.create_subscription(
            VehicleStatus, self.status_topic, self.status_cb, px4_qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.local_position_topic,
            self.local_position_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleLandDetected,
            self.land_detected_topic,
            self.land_detected_cb,
            px4_qos,
        )
        self.create_subscription(
            Image, self.camera_front_raw_topic, self.front_raw_cb, px4_qos
        )
        self.create_subscription(
            Image, self.camera_front_rect_topic, self.front_rect_cb, px4_qos
        )
        self.create_subscription(
            Image, self.camera_down_raw_topic, self.down_raw_cb, px4_qos
        )
        self.create_subscription(
            Image, self.camera_down_rect_topic, self.down_rect_cb, px4_qos
        )

        # ------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, self.trajectory_setpoint_topic, px4_qos
        )
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, self.offboard_control_mode_topic, px4_qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, self.vehicle_command_topic, px4_qos
        )

        # Kontinuierlicher Setpoint-/Offboard-Stream (wie bisher).
        self.create_timer(
            1.0 / max(self.setpoint_publish_rate_hz, 0.1),
            self.publish_setpoint,
        )

    # ====================================================================
    # Zeit
    # ====================================================================

    def now_us(self):
        """Aktuelle Zeit in Mikrosekunden (fuer PX4-Nachrichten)."""
        return int(self.get_clock().now().nanoseconds / 1000)

    # ====================================================================
    # Callbacks
    # ====================================================================

    def status_cb(self, msg):
        """Callback fuer /fmu/out/vehicle_status."""
        self.vehicle_status = msg
        self.last_status_time = time.monotonic()

    def local_position_cb(self, msg):
        """Callback fuer /fmu/out/vehicle_local_position."""
        self.local_position = msg
        self.last_pose_time = time.monotonic()

    def land_detected_cb(self, msg):
        """Callback fuer /fmu/out/vehicle_land_detected."""
        self.vehicle_land_detected = msg
        self.last_land_detected_time = time.monotonic()

    def front_raw_cb(self, msg):
        self.last_front_raw = time.monotonic()

    def front_rect_cb(self, msg):
        self.last_front_rect = time.monotonic()

    def down_raw_cb(self, msg):
        self.last_down_raw = time.monotonic()

    def down_rect_cb(self, msg):
        self.last_down_rect = time.monotonic()

    # ====================================================================
    # Status-Helfer
    # ====================================================================

    def is_recent(self, stamp, timeout_s):
        """Pruefen, ob ein Zeitstempel juenger als timeout_s ist."""
        if stamp <= 0.0:
            return False
        return time.monotonic() - stamp <= timeout_s

    def camera_online(self, raw_stamp, rect_stamp):
        """Kamera gilt als online, wenn raw oder rect innerhalb Timeout kam."""
        now = time.monotonic()
        return (
            now - raw_stamp <= self.camera_timeout_s
            or now - rect_stamp <= self.camera_timeout_s
        )

    def is_armed(self):
        """True, wenn ein frischer VehicleStatus 'armed' meldet."""
        if self.vehicle_status is None:
            return False
        if not self.is_recent(self.last_status_time, self.status_timeout_s):
            return False
        return (
            self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

    def is_offboard(self):
        """True, wenn PX4 frisch den Offboard-Modus meldet."""
        if self.vehicle_status is None:
            return False
        if not self.is_recent(self.last_status_time, self.status_timeout_s):
            return False
        return (
            self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def is_landed(self):
        """True, wenn PX4 Bodenkontakt meldet (vehicle_land_detected)."""
        if self.vehicle_land_detected is None:
            return False
        if not self.is_recent(
            self.last_land_detected_time, self.status_timeout_s
        ):
            return False
        return bool(self.vehicle_land_detected.landed)

    def get_pose_enu(self):
        """Aktuelle Pose in ENU zurueckgeben oder None.

        Liest VehicleLocalPosition (NED) und rechnet nach ENU um.
        Returns:
            (x, y, z, yaw) in ENU oder None, wenn keine frische Pose vorliegt.
        """
        if self.local_position is None:
            return None
        if not self.is_recent(self.last_pose_time, self.pose_timeout_s):
            return None
        lp = self.local_position
        x, y, z = ned_pos_to_enu(float(lp.x), float(lp.y), float(lp.z))
        yaw = yaw_ned_to_enu(float(lp.heading))
        return x, y, z, yaw

    # ====================================================================
    # Setpoint-Logik
    # ====================================================================

    def track_pose_as_setpoint(self):
        """Setpoint an aktuelle Pose koppeln (x, y, z + takeoff_offset_m).

        Wird nur aufgerufen, solange die Drohne nicht armed ist. Solange noch
        keine frische Pose vorliegt, bleiben die Fallback-Werte erhalten.
        Yaw wird aus der aktuellen Orientierung uebernommen.
        """
        pose = self.get_pose_enu()
        if pose is None:
            return
        x, y, z, yaw = pose
        self.target_x = x
        self.target_y = y
        self.target_z = z + self.takeoff_offset_m
        self.target_yaw = yaw

    def publish_setpoint(self):
        """Timer-Callback: OffboardControlMode + TrajectorySetpoint senden.

        Verhalten:
          - Disarmed: Setpoint folgt der aktuellen Pose (+ Hoehenoffset);
            es wird gestreamt, damit ein Offboard-Wechsel moeglich ist.
          - Armed + Offboard: Setpoint wird per Tastatur bewegt; im
            Landemodus sinkt der Z-Sollwert langsam (X/Y/Yaw eingefroren).
          - Armed + NICHT Offboard: ein anderer Modus hat uebernommen
            (z.B. RC-AUTO.LAND im Notfall). Dann wird NICHTS gesendet,
            damit der Autopilot/Pilot nicht gestoert wird. Das ist die
            saubere Uebergabe -- kein konkurrierender Stream, kein Ruckeln.
        """
        armed_now = self.is_armed()
        offboard_now = self.is_offboard()

        # --------------------------------------------------------------
        # Uebergabe an RC/Autopilot: armed, aber nicht im Offboard-Modus.
        # Wir schweigen komplett, damit z.B. RC-AUTO.LAND ungestoert
        # laeuft. (Kein Failsafe-Risiko, weil der Pilot die Kontrolle hat.)
        # --------------------------------------------------------------
        if armed_now and not offboard_now:
            self._landing = False
            self._demo_active = False
            self._was_armed = armed_now
            return

        # --------------------------------------------------------------
        # Vor dem Arm: Setpoint an aktuelle Pose koppeln (Tracking).
        # --------------------------------------------------------------
        if not armed_now:
            self._landing = False
            self._demo_active = False
            self.track_pose_as_setpoint()

        # --------------------------------------------------------------
        # Landung an Ort und Stelle (Offboard-Sinkflug).
        # X/Y/Yaw bleiben unveraendert eingefroren, nur Z wird abgesenkt.
        # --------------------------------------------------------------
        if self._landing and armed_now and offboard_now:
            dt = 1.0 / max(self.setpoint_publish_rate_hz, 0.1)
            # ENU: nach unten = z verringern.
            self.target_z -= self.descent_speed_mps * dt

            # Primär: Bodenkontakt-Erkennung von PX4.
            if self.is_landed():
                self.call_disarm()
                self._landing = False
            # Fallback: Sicherheits-Untergrenze erreicht, falls die
            # Erkennung ausbleibt -> trotzdem disarmen.
            elif self.target_z <= self.land_min_z_m:
                self.call_disarm()
                self._landing = False

        # --------------------------------------------------------------
        # Demo-Modus: Quadrat abfliegen.
        # X/Y werden entlang der Wegpunkte verschoben, Z/Yaw bleiben fix.
        # Landung und Demo schliessen sich gegenseitig aus.
        # --------------------------------------------------------------
        if self._demo_active and armed_now and offboard_now and not self._landing:
            self._run_demo_step()

        self._was_armed = armed_now

        stamp = self.now_us()

        # 1) OffboardControlMode: wir steuern Position.
        ocm = OffboardControlMode()
        ocm.timestamp = stamp
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.offboard_control_mode_pub.publish(ocm)

        # 2) TrajectorySetpoint in NED.
        nx, ny, nz = enu_pos_to_ned(self.target_x, self.target_y, self.target_z)
        nyaw = yaw_enu_to_ned(self.target_yaw)

        sp = TrajectorySetpoint()
        sp.timestamp = stamp
        sp.position = [float(nx), float(ny), float(nz)]
        nan = float("nan")
        sp.velocity = [nan, nan, nan]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = float(nyaw)
        sp.yawspeed = nan
        self.trajectory_setpoint_pub.publish(sp)

    # ====================================================================
    # VehicleCommand
    # ====================================================================

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """VehicleCommand an PX4 senden."""
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = self.source_system
        msg.source_component = self.source_component
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def call_arm(self):
        """Armen per VehicleCommand (ARM_DISARM, param1=1)."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    def call_disarm(self):
        """Disarmen per VehicleCommand (ARM_DISARM, param1=0)."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )

    def start_landing(self):
        """Landung an Ort und Stelle starten (Offboard-Sinkflug).

        Setzt nur das Landing-Flag. X/Y/Yaw bleiben auf ihrem aktuellen
        Sollwert eingefroren; der publish_setpoint-Timer senkt ab dann
        den Z-Sollwert mit descent_speed_mps ab, bis Bodenkontakt erkannt
        wird (dann automatisches Disarm). Ein eventuell laufender Demo-Modus
        wird abgebrochen.
        """
        self.stop_demo()
        self._landing = True

    # ====================================================================
    # Demo-Modus (Quadrat abfliegen)
    # ====================================================================

    def handle_demo_toggle(self):
        """Taste 'g': Demo-Modus starten oder abbrechen.

        Starten ist nur erlaubt, wenn die Drohne armed und im Offboard-Modus
        ist und gerade nicht landet. Erneutes Druecken bricht eine laufende
        Demo ab und haelt die aktuelle Position.
        """
        if self._demo_active:
            self.stop_demo()
            return
        if not self.is_armed() or not self.is_offboard() or self._landing:
            return
        self.start_demo()

    def start_demo(self):
        """Demo-Modus starten: Wegpunkte des Quadrats berechnen.

        Hoehe und Yaw werden auf den aktuellen Sollwert eingefroren. Die vier
        Ecken werden um (demo_center_x, demo_center_y) mit halber Kantenlaenge
        gebildet. Startecke ist die zur aktuellen Position naechstgelegene
        Ecke; danach wird das Quadrat einmal umrundet und zur Startecke
        zurueckgekehrt (dort schwebt die Drohne anschliessend).
        """
        if self.demo_speed_mps <= 0.0 or self.demo_edge_length_m <= 0.0:
            # Ungueltige Konfiguration -> nicht starten (wuerde haengen).
            return

        # Hoehe und Yaw fuer die gesamte Demo festhalten.
        self._demo_z = self.target_z
        self._demo_yaw = self.target_yaw

        half = 0.5 * self.demo_edge_length_m
        cx = self.demo_center_x
        cy = self.demo_center_y
        # Ecken gegen den Uhrzeigersinn (Reihenfolge egal, da Yaw fix).
        corners = [
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + half, cy + half),
            (cx - half, cy + half),
        ]

        # Naechstgelegene Ecke zur aktuellen Soll-Position als Start.
        px, py = self.target_x, self.target_y
        dists = [(px - c[0]) ** 2 + (py - c[1]) ** 2 for c in corners]
        k = dists.index(min(dists))

        # Reihenfolge: Startecke, drei weitere Ecken, dann zurueck zur
        # Startecke (Quadrat einmal komplett umrundet).
        order = [corners[(k + i) % 4] for i in range(4)]
        order.append(corners[k])

        self._demo_waypoints = order
        self._demo_index = 0
        self._demo_active = True

    def stop_demo(self):
        """Demo-Modus beenden und Wegpunkte verwerfen.

        Die aktuelle Soll-Position bleibt unveraendert, die Drohne haelt
        also dort, wo sie gerade ist.
        """
        self._demo_active = False
        self._demo_waypoints = []
        self._demo_index = 0

    def _run_demo_step(self):
        """Ein Demo-Tick: Ziel-Setpoint Richtung aktuellem Wegpunkt schieben.

        Hoehe und Yaw werden auf die eingefrorenen Werte gehalten. X/Y werden
        pro Tick um speed * dt in Richtung des Wegpunkts verschoben. Ist der
        Wegpunkt erreicht, wird auf ihn eingerastet und der naechste Wegpunkt
        angesteuert. Nach dem letzten Wegpunkt (Startecke) endet die Demo,
        die Drohne schwebt dort.
        """
        # Hoehe und Yaw waehrend der Demo konstant halten.
        self.target_z = self._demo_z
        self.target_yaw = self._demo_yaw

        if self._demo_index >= len(self._demo_waypoints):
            self.stop_demo()
            return

        gx, gy = self._demo_waypoints[self._demo_index]
        dx = gx - self.target_x
        dy = gy - self.target_y
        dist = math.hypot(dx, dy)

        dt = 1.0 / max(self.setpoint_publish_rate_hz, 0.1)
        step = self.demo_speed_mps * dt

        if dist <= max(step, 1e-6):
            # Wegpunkt erreicht -> exakt einrasten und weiterschalten.
            self.target_x = gx
            self.target_y = gy
            self._demo_index += 1
            if self._demo_index >= len(self._demo_waypoints):
                # Letzter Wegpunkt (Startecke) erreicht -> Demo beenden.
                self.stop_demo()
        else:
            # Ein Stueck in Richtung Wegpunkt schieben.
            self.target_x += step * dx / dist
            self.target_y += step * dy / dist

    # ====================================================================
    # Tastatur
    # ====================================================================

    def handle_key(self, key):
        """Tastatureingaben auf Sollwertaenderungen abbilden.

        Position/Yaw-Tasten wirken nur, wenn die Drohne armed ist und NICHT
        im Landeanflug. Vor dem Arm folgt der Setpoint der aktuellen Pose.
        SPACE (arm / Landung starten) ist immer aktiv.
        """
        if key == ord(" "):
            self.handle_space()
            return

        if key in (ord("g"), ord("G")):
            self.handle_demo_toggle()
            return

        # Waehrend der Landung ODER der Demo keine manuellen Bewegungen
        # zulassen (X/Y/Yaw werden dort automatisch gefuehrt bzw. eingefroren).
        if not self.is_armed() or self._landing or self._demo_active:
            return

        if key in (ord("w"), ord("W")):
            self.target_y -= self.xy_step_m
        elif key in (ord("s"), ord("S")):
            self.target_y += self.xy_step_m
        elif key in (ord("a"), ord("A")):
            self.target_x += self.xy_step_m
        elif key in (ord("d"), ord("D")):
            self.target_x -= self.xy_step_m
        elif key in (ord("o"), ord("O")):
            self.target_z += self.z_step_m
        elif key in (ord("l"), ord("L")):
            self.target_z -= self.z_step_m
        elif key in (ord("q"), ord("Q")):
            self.target_yaw = wrap_angle(self.target_yaw - self.yaw_step_rad)
        elif key in (ord("e"), ord("E")):
            self.target_yaw = wrap_angle(self.target_yaw + self.yaw_step_rad)

    def handle_space(self):
        """SPACE: armen, oder wenn bereits armed, Landung an Ort und Stelle.

        Statt PX4-AUTO.LAND wird ein Offboard-Sinkflug gestartet
        (X/Y/Yaw eingefroren, Z langsam abgesenkt). Erneutes SPACE
        waehrend der Landung bricht den Sinkflug ab (Hold an aktueller
        Hoehe), damit man im Zweifel stoppen kann.
        """
        if self.is_armed():
            if self._landing:
                # Laufende Landung abbrechen -> Position halten.
                self._landing = False
            else:
                self.start_landing()
        else:
            self.call_arm()

    # ====================================================================
    # Anzeige
    # ====================================================================

    def nav_state_name(self):
        """Lesbarer Name des aktuellen nav_state."""
        if self.vehicle_status is None:
            return "-"
        ns = self.vehicle_status.nav_state
        return _NAV_STATE_NAMES.get(ns, f"#{ns}")

    def get_display_lines(self):
        """Anzuzeigende Textzeilen fuer das curses-UI zusammenstellen."""
        front_online = self.camera_online(self.last_front_raw, self.last_front_rect)
        down_online = self.camera_online(self.last_down_raw, self.last_down_rect)

        status_recent = self.is_recent(self.last_status_time, self.status_timeout_s)
        pose = self.get_pose_enu()
        armed_now = self.is_armed()

        lines = []
        lines.append("ROS2 uXRCE-DDS Control Console")
        lines.append("=" * 60)
        lines.append("")

        if self.vehicle_status is None:
            lines.append("PX4 status: noch keine Nachricht empfangen")
            lines.append("verbunden: False")
            lines.append("armed: -")
            lines.append("nav_state: -")
        elif not status_recent:
            age = time.monotonic() - self.last_status_time
            armed_txt = (
                self.vehicle_status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED
            )
            lines.append(f"PX4 status: veraltet seit {age:.1f} s")
            lines.append("verbunden: False")
            lines.append(f"armed: {armed_txt}  (letzter Wert)")
            lines.append(f"nav_state: {self.nav_state_name()}  (letzter Wert)")
        else:
            lines.append("PX4 status:")
            lines.append("verbunden: True")
            lines.append(f"armed: {armed_now}")
            lines.append(f"nav_state: {self.nav_state_name()}")

        lines.append("")
        lines.append(f"Camera_Front: {'online' if front_online else 'offline'}")
        lines.append(f"Camera_Down:  {'online' if down_online else 'offline'}")

        lines.append("")
        if self.local_position is None:
            lines.append("Aktuelle Position: noch keine Nachricht empfangen")
            lines.append("x: -    y: -    z: -")
        elif pose is None:
            x, y, z = ned_pos_to_enu(
                float(self.local_position.x),
                float(self.local_position.y),
                float(self.local_position.z),
            )
            age = time.monotonic() - self.last_pose_time
            lines.append(f"Aktuelle Position (ENU): veraltet seit {age:.1f} s")
            lines.append(
                f"x: {x:.2f} m    y: {y:.2f} m    z: {z:.2f} m    (letzter Wert)"
            )
        else:
            x, y, z, _yaw = pose
            lines.append(f"Aktuelle Position (ENU) {self.local_position_topic}:")
            lines.append(f"x: {x:.2f} m    y: {y:.2f} m    z: {z:.2f} m")

        # Setpoint-Modus
        offboard_now = self.is_offboard()
        if armed_now and not offboard_now:
            setpoint_mode = "UEBERGABE -> RC/Autopilot aktiv (Konsole sendet NICHT)"
        elif self._landing:
            setpoint_mode = (
                f"LANDUNG an Ort und Stelle (Sinkflug {self.descent_speed_mps:.2f} m/s)"
            )
        elif self._demo_active:
            wp = min(self._demo_index + 1, len(self._demo_waypoints))
            setpoint_mode = (
                f"DEMO Quadrat (Kante {self.demo_edge_length_m:.2f} m, "
                f"v {self.demo_speed_mps:.2f} m/s) -> Wegpunkt {wp}/"
                f"{len(self._demo_waypoints)}"
            )
        elif armed_now:
            setpoint_mode = "armed -> Tastatur"
        else:
            if pose is not None:
                setpoint_mode = (
                    f"nicht armed -> Tracking (Pose + {self.takeoff_offset_m:.2f} m)"
                )
            else:
                setpoint_mode = "nicht armed -> Fallback (start_*-Parameter)"

        lines.append("")
        lines.append(
            f"Zielposition (ENU) {self.trajectory_setpoint_topic}  "
            f"[{setpoint_mode}]:"
        )
        lines.append(
            f"x: {self.target_x:.2f} m    "
            f"y: {self.target_y:.2f} m    "
            f"z: {self.target_z:.2f} m"
        )
        lines.append(f"yaw: {math.degrees(self.target_yaw):.1f} deg")

        # Bodenkontakt-Anzeige
        if self.vehicle_land_detected is not None:
            landed_txt = "ja" if self.is_landed() else "nein"
            lines.append(f"Bodenkontakt (PX4): {landed_txt}")

        lines.append("")
        lines.append("Hinweis: Offboard-Mode extern (RC/QGC) setzen.")
        lines.append("RC-AUTO.LAND bleibt jederzeit als Notfall nutzbar.")
        lines.append("Tasten:")
        lines.append("w/s: y -/+    a/d: x +/-    o/l: z +/-")
        lines.append("q/e: yaw -/+")
        lines.append("SPACE: arm | wenn armed: Landung an Ort und Stelle starten")
        lines.append("       | waehrend Landung: SPACE bricht ab (Hold)")
        lines.append(
            "g: Demo-Modus (Quadrat) starten | erneut g: abbrechen (Hold)"
        )
        lines.append("Bewegungs-Tasten wirken nur armed, nicht im Lande-/Demo-Modus.")
        lines.append("ESC oder Ctrl+C: beenden")
        return lines


def curses_main(stdscr, node):
    """Haupt-Loop: ROS spinnen, Tasten lesen, Anzeige aktualisieren."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    display_dt = 1.0 / max(node.display_rate_hz, 0.1)
    last_display = 0.0

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)

        key = stdscr.getch()
        if key == 27:  # ESC
            break
        elif key != -1:
            node.handle_key(key)

        now = time.monotonic()
        if now - last_display >= display_dt:
            last_display = now

            stdscr.erase()
            for i, line in enumerate(node.get_display_lines()):
                try:
                    stdscr.addstr(i, 0, line)
                except curses.error:
                    pass
            stdscr.refresh()


def main(args=None):
    """Einstiegspunkt: rclpy initialisieren, Node + curses starten."""
    rclpy.init(args=args)
    node = ControlConsole()

    try:
        curses.wrapper(curses_main, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()