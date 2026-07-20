#!/usr/bin/env python3

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

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image


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


def rpy_to_quat(roll, pitch, yaw):
    """Roll, Pitch, Yaw [rad] in normalisiertes Quaternion (x, y, z, w) umrechnen."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


def wrap_angle(a):
    """Winkel auf den Bereich (-pi, pi] normalisieren."""
    return math.atan2(math.sin(a), math.cos(a))


class ControlConsole(Node):
    def __init__(self):
        """ROS2-Node fuer die MAVROS-Bedienkonsole initialisieren."""
        super().__init__("control_console")

        self.declare_parameter("state_topic", "/mavros/state")
        self.declare_parameter("local_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("setpoint_topic", "/mavros/setpoint_position/local")

        self.declare_parameter("camera_front_raw_topic", "/camera_1/image_raw")
        self.declare_parameter("camera_front_rect_topic", "/camera_1/image_rect")
        self.declare_parameter("camera_down_raw_topic", "/camera_2/image_raw")
        self.declare_parameter("camera_down_rect_topic", "/camera_2/image_rect")

        self.declare_parameter("arming_service", "/mavros/cmd/arming")
        self.declare_parameter("set_mode_service", "/mavros/set_mode")

        self.declare_parameter("display_rate_hz", 2.0)
        self.declare_parameter("setpoint_publish_rate_hz", 20.0)
        self.declare_parameter("camera_timeout_s", 1.0)

        # Wenn laenger als diese Zeit keine neue MAVROS-State-Nachricht kommt,
        # wird der Zustand als veraltet behandelt.
        self.declare_parameter("state_timeout_s", 2.0)

        # Wenn laenger als diese Zeit keine neue lokale Pose kommt,
        # wird die Position als veraltet markiert.
        self.declare_parameter("pose_timeout_s", 2.0)

        self.declare_parameter("xy_step_m", 0.05)
        self.declare_parameter("z_step_m", 0.05)
        self.declare_parameter("yaw_step_deg", 5.0)

        # Default-Offset ueber der aktuellen Position, solange die Drohne
        # nicht armed ist. Setpoint = aktuelle Position + (0, 0, takeoff_offset).
        self.declare_parameter("takeoff_offset_m", 1.5)

        self.declare_parameter("frame_id", "map")

        # Wenn True wird header.stamp leer gelassen (wie bei
        # `ros2 topic pub`). MAVROS akzeptiert das problemlos und es vermeidet
        # Probleme durch Zeitstempel-Inkonsistenzen zwischen Bordrechner und FCU.
        self.declare_parameter("use_empty_stamp", True)

        # Fallback-Werte: werden genutzt, solange noch keine frische lokale
        # Pose empfangen wurde. Sobald Pose verfuegbar ist, wird der
        # Setpoint im "nicht armed"-Zustand aus der Pose abgeleitet.
        self.declare_parameter("start_x", -0.2030704)
        self.declare_parameter("start_y", -0.1153477)
        self.declare_parameter("start_z", 1.36)
        self.declare_parameter("start_qx", -0.0012295)
        self.declare_parameter("start_qy", 0.00490711)
        self.declare_parameter("start_qz", -0.7050490)
        self.declare_parameter("start_qw", -0.7091406)

        self.state_topic = self.get_parameter("state_topic").value
        self.local_pose_topic = self.get_parameter("local_pose_topic").value
        self.setpoint_topic = self.get_parameter("setpoint_topic").value

        self.camera_front_raw_topic = self.get_parameter("camera_front_raw_topic").value
        self.camera_front_rect_topic = self.get_parameter("camera_front_rect_topic").value
        self.camera_down_raw_topic = self.get_parameter("camera_down_raw_topic").value
        self.camera_down_rect_topic = self.get_parameter("camera_down_rect_topic").value

        self.arming_service = self.get_parameter("arming_service").value
        self.set_mode_service = self.get_parameter("set_mode_service").value

        self.display_rate_hz = float(self.get_parameter("display_rate_hz").value)
        self.setpoint_publish_rate_hz = float(
            self.get_parameter("setpoint_publish_rate_hz").value
        )
        self.camera_timeout_s = float(self.get_parameter("camera_timeout_s").value)
        self.state_timeout_s = float(self.get_parameter("state_timeout_s").value)
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)

        self.xy_step_m = float(self.get_parameter("xy_step_m").value)
        self.z_step_m = float(self.get_parameter("z_step_m").value)
        self.yaw_step_rad = math.radians(float(self.get_parameter("yaw_step_deg").value))
        self.takeoff_offset_m = float(self.get_parameter("takeoff_offset_m").value)

        self.frame_id = self.get_parameter("frame_id").value
        self.use_empty_stamp = bool(self.get_parameter("use_empty_stamp").value)

        self.state = None
        self.local_pose = None

        # Zeitpunkte der letzten empfangenen Nachrichten.
        # Dadurch bleibt die Anzeige nicht dauerhaft auf einem alten Wert stehen.
        self.last_state_time = 0.0
        self.last_pose_time = 0.0

        self.last_front_raw = 0.0
        self.last_front_rect = 0.0
        self.last_down_raw = 0.0
        self.last_down_rect = 0.0

        # Fallback-Sollwerte aus den start_*-Parametern. Werden genutzt,
        # solange noch keine frische Pose vorliegt.
        self.target_x = float(self.get_parameter("start_x").value)
        self.target_y = float(self.get_parameter("start_y").value)
        self.target_z = float(self.get_parameter("start_z").value)

        qx = float(self.get_parameter("start_qx").value)
        qy = float(self.get_parameter("start_qy").value)
        qz = float(self.get_parameter("start_qz").value)
        qw = float(self.get_parameter("start_qw").value)
        self.target_roll, self.target_pitch, self.target_yaw = quat_to_rpy(
            qx, qy, qz, qw
        )

        # Merker fuer die Arm-Flanke. Solange False -> Setpoint trackt die
        # aktuelle Pose + takeoff_offset_m. Bei steigender Flanke False -> True
        # wird der Setpoint eingefroren und nur noch ueber die Tastatur veraendert.
        self._was_armed = False

        # MAVROS und Kamera-Topics koennen je nach Publisher mit BEST_EFFORT laufen.
        # Mit BEST_EFFORT als Subscriber bleiben wir kompatibel zu BEST_EFFORT UND RELIABLE Publishern.
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Fuer den Setpoint-Publisher lassen wir die ROS2-Default-QoS ueber depth=10.
        # Das ist fuer MAVROS-Setpoints in der Regel passend.
        self.create_subscription(
            State,
            self.state_topic,
            self.state_cb,
            qos_best_effort,
        )
        self.create_subscription(
            PoseStamped,
            self.local_pose_topic,
            self.pose_cb,
            qos_best_effort,
        )
        self.create_subscription(
            Image,
            self.camera_front_raw_topic,
            self.front_raw_cb,
            qos_best_effort,
        )
        self.create_subscription(
            Image,
            self.camera_front_rect_topic,
            self.front_rect_cb,
            qos_best_effort,
        )
        self.create_subscription(
            Image,
            self.camera_down_raw_topic,
            self.down_raw_cb,
            qos_best_effort,
        )
        self.create_subscription(
            Image,
            self.camera_down_rect_topic,
            self.down_rect_cb,
            qos_best_effort,
        )

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            self.setpoint_topic,
            10,
        )
        self.arming_client = self.create_client(CommandBool, self.arming_service)
        self.set_mode_client = self.create_client(SetMode, self.set_mode_service)

        self.create_timer(
            1.0 / max(self.setpoint_publish_rate_hz, 0.1),
            self.publish_setpoint,
        )

    def state_cb(self, msg):
        """Callback fuer /mavros/state. Aktualisiert State und Zeitstempel."""
        self.state = msg
        self.last_state_time = time.monotonic()

    def pose_cb(self, msg):
        """Callback fuer die lokale Pose. Aktualisiert Pose und Zeitstempel."""
        self.local_pose = msg
        self.last_pose_time = time.monotonic()

    def front_raw_cb(self, msg):
        """Callback fuer die vordere Kamera (raw). Nur Zeitstempel relevant."""
        self.last_front_raw = time.monotonic()

    def front_rect_cb(self, msg):
        """Callback fuer die vordere Kamera (rect). Nur Zeitstempel relevant."""
        self.last_front_rect = time.monotonic()

    def down_raw_cb(self, msg):
        """Callback fuer die untere Kamera (raw). Nur Zeitstempel relevant."""
        self.last_down_raw = time.monotonic()

    def down_rect_cb(self, msg):
        """Callback fuer die untere Kamera (rect). Nur Zeitstempel relevant."""
        self.last_down_rect = time.monotonic()

    def is_recent(self, stamp, timeout_s):
        """Pruefen, ob ein Zeitstempel juenger als timeout_s ist."""
        if stamp <= 0.0:
            return False
        return time.monotonic() - stamp <= timeout_s

    def camera_online(self, raw_stamp, rect_stamp):
        """Kamera gilt als online, wenn raw oder rect innerhalb des Timeouts kam."""
        now = time.monotonic()
        return (
            now - raw_stamp <= self.camera_timeout_s
            or now - rect_stamp <= self.camera_timeout_s
        )

    def is_armed(self):
        """True, wenn aktueller, frischer MAVROS-State 'armed' meldet."""
        if self.state is None:
            return False
        if not self.is_recent(self.last_state_time, self.state_timeout_s):
            return False
        return bool(self.state.armed)

    def track_pose_as_setpoint(self):
        """Setpoint an aktuelle Pose koppeln: (x, y, z + takeoff_offset_m).

        Wird nur aufgerufen, solange die Drohne nicht armed ist. Solange noch
        keine frische Pose vorliegt, bleiben die Fallback-Werte aus den
        start_*-Parametern erhalten. Yaw wird aus der aktuellen Orientierung
        uebernommen, Roll und Pitch werden auf 0 gesetzt, da der Setpoint
        immer aufrecht sein soll.
        """
        if self.local_pose is None:
            return
        if not self.is_recent(self.last_pose_time, self.pose_timeout_s):
            return

        p = self.local_pose.pose.position
        o = self.local_pose.pose.orientation

        self.target_x = p.x
        self.target_y = p.y
        self.target_z = p.z + self.takeoff_offset_m

        _, _, yaw = quat_to_rpy(o.x, o.y, o.z, o.w)
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = yaw

    def publish_setpoint(self):
        """Timer-Callback: Setpoint-Quelle waehlen und PoseStamped publishen.

        Die Nachricht entspricht in Form 1:1 dem, was `ros2 topic pub` mit
        leerem header (nur frame_id, kein stamp) sendet. Konkret bleibt
        header.stamp bei sec=0, nanosec=0, wenn use_empty_stamp True ist.
        """
        armed_now = self.is_armed()

        # Vor dem Arm: Setpoint folgt der aktuellen Pose (+ Hoehenoffset).
        # Beim Uebergang nicht-armed -> armed wird der Setpoint eingefroren,
        # indem dieser Zweig ueberspringen wird. Danach veraendern nur noch
        # Tastatureingaben target_*.
        if not armed_now:
            self.track_pose_as_setpoint()

        self._was_armed = armed_now

        msg = PoseStamped()
        if self.use_empty_stamp:
            # Wie bei `ros2 topic pub` ohne stamp-Feld: sec=0, nanosec=0.
            msg.header.stamp = TimeMsg()
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z

        qx, qy, qz, qw = rpy_to_quat(
            self.target_roll,
            self.target_pitch,
            self.target_yaw,
        )
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.setpoint_pub.publish(msg)

    def handle_key(self, key):
        """Tastatureingaben auf Sollwertaenderungen abbilden.

        Position/Yaw-Tasten werden nur wirksam, wenn die Drohne armed ist.
        Vor dem Arm folgt der Setpoint automatisch der aktuellen Pose, die
        Tastatureingaben fuer x/y/z/yaw werden in diesem Zustand bewusst
        ignoriert, damit sie keinen Effekt haben, der gleich wieder
        ueberschrieben wuerde. SPACE (arm/land) ist immer aktiv.
        """
        if key == ord(" "):
            self.handle_space()
            return

        if not self.is_armed():
            # Vor dem Arm regelt das Pose-Tracking den Setpoint.
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
        """SPACE: armen, oder wenn bereits armed, AUTO.LAND ausloesen."""
        if self.is_armed():
            self.call_land()
        else:
            self.call_arm()

    def call_arm(self):
        """Arming-Service mit value=True asynchron aufrufen."""
        if not self.arming_client.service_is_ready():
            self.get_logger().warn(f"Service not ready: {self.arming_service}")
            return

        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)

    def call_land(self):
        """SetMode-Service mit AUTO.LAND asynchron aufrufen."""
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn(f"Service not ready: {self.set_mode_service}")
            return

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "AUTO.LAND"
        self.set_mode_client.call_async(req)

    def get_display_lines(self):
        """Anzuzeigende Textzeilen fuer das curses-UI zusammenstellen."""
        front_online = self.camera_online(self.last_front_raw, self.last_front_rect)
        down_online = self.camera_online(self.last_down_raw, self.last_down_rect)

        state_recent = self.is_recent(self.last_state_time, self.state_timeout_s)
        pose_recent = self.is_recent(self.last_pose_time, self.pose_timeout_s)
        armed_now = self.is_armed()

        lines = []
        lines.append("ROS2 MAVROS Control Console")
        lines.append("=" * 60)
        lines.append("")

        if self.state is None:
            lines.append("MAVROS state: noch keine Nachricht empfangen")
            lines.append("Mavros connected: False")
            lines.append("armed: -")
            lines.append("system_status: -")
            lines.append("mode: -")
        elif not state_recent:
            # Kein frischer /mavros/state mehr.
            # Damit bleibt die Anzeige nicht faelschlich auf True stehen.
            age = time.monotonic() - self.last_state_time
            lines.append(f"MAVROS state: veraltet seit {age:.1f} s")
            lines.append("Mavros connected: False")
            lines.append(f"armed: {self.state.armed}  (letzter Wert)")
            lines.append(f"system_status: {self.state.system_status}  (letzter Wert)")
            lines.append(f"mode: {self.state.mode}  (letzter Wert)")
        else:
            lines.append("MAVROS state:")
            lines.append(f"Mavros connected: {self.state.connected}")
            lines.append(f"armed: {self.state.armed}")
            lines.append(f"system_status: {self.state.system_status}")
            lines.append(f"mode: {self.state.mode}")

        lines.append("")
        lines.append(f"Camera_Front: {'online' if front_online else 'offline'}")
        lines.append(f"Camera_Down:  {'online' if down_online else 'offline'}")

        lines.append("")
        if self.local_pose is None:
            lines.append("Aktuelle Position: noch keine Nachricht empfangen")
            lines.append("x: -    y: -    z: -")
        elif not pose_recent:
            p = self.local_pose.pose.position
            age = time.monotonic() - self.last_pose_time
            lines.append(f"Aktuelle Position: veraltet seit {age:.1f} s")
            lines.append(
                f"x: {p.x:.2f} m    y: {p.y:.2f} m    z: {p.z:.2f} m    (letzter Wert)"
            )
        else:
            p = self.local_pose.pose.position
            lines.append(f"Aktuelle Position {self.local_pose_topic}:")
            lines.append(f"x: {p.x:.2f} m    y: {p.y:.2f} m    z: {p.z:.2f} m")

        qx, qy, qz, qw = rpy_to_quat(
            self.target_roll,
            self.target_pitch,
            self.target_yaw,
        )

        # Statuszeile, wer den Setpoint gerade bestimmt.
        if armed_now:
            setpoint_mode = "armed -> Tastatur"
        else:
            if self.local_pose is not None and pose_recent:
                setpoint_mode = (
                    f"nicht armed -> Tracking (Pose + {self.takeoff_offset_m:.2f} m)"
                )
            else:
                setpoint_mode = "nicht armed -> Fallback (start_*-Parameter)"

        stamp_info = "leerer stamp" if self.use_empty_stamp else "ROS-Now stamp"

        lines.append("")
        lines.append(
            f"Zielposition {self.setpoint_topic}  "
            f"[{setpoint_mode}, frame_id={self.frame_id}, {stamp_info}]:"
        )
        lines.append(
            f"x: {self.target_x:.2f} m    "
            f"y: {self.target_y:.2f} m    "
            f"z: {self.target_z:.2f} m"
        )
        lines.append(
            f"q: x={qx:.4f} y={qy:.4f} z={qz:.4f} w={qw:.4f}    "
            f"yaw={math.degrees(self.target_yaw):.1f} deg"
        )

        lines.append("")
        lines.append("Tasten:")
        lines.append("w/s: y -/+    a/d: x +/-    o/l: z +/-")
        lines.append("q/e: yaw -/+  SPACE: arm oder, wenn armed, AUTO.LAND")
        lines.append("Bewegungs-Tasten wirken nur, wenn die Drohne armed ist.")
        lines.append("ESC oder Ctrl+C: beenden")
        return lines


def curses_main(stdscr, node):
    """Haupt-Loop unter curses: ROS spinnen, Tasten lesen, Anzeige aktualisieren."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    display_dt = 1.0 / max(node.display_rate_hz, 0.1)
    last_display = 0.0

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)

        key = stdscr.getch()
        if key == 27:
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
    """Einstiegspunkt: rclpy initialisieren, Node + curses starten, aufraeumen."""
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