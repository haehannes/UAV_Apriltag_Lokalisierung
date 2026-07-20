from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch-Beschreibung fuer den Control-Console-Node erzeugen."""
    return LaunchDescription([
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("local_pose_topic", default_value="/mavros/local_position/pose"),
        DeclareLaunchArgument("setpoint_topic", default_value="/mavros/setpoint_position/local"),

        DeclareLaunchArgument("camera_front_raw_topic", default_value="/camera_1/image_raw"),
        DeclareLaunchArgument("camera_front_rect_topic", default_value="/camera_1/image_rect"),
        DeclareLaunchArgument("camera_down_raw_topic", default_value="/camera_2/image_raw"),
        DeclareLaunchArgument("camera_down_rect_topic", default_value="/camera_2/image_rect"),

        DeclareLaunchArgument("arming_service", default_value="/mavros/cmd/arming"),
        DeclareLaunchArgument("set_mode_service", default_value="/mavros/set_mode"),

        DeclareLaunchArgument("display_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("setpoint_publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("camera_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("state_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("pose_timeout_s", default_value="2.0"),

        DeclareLaunchArgument("xy_step_m", default_value="0.05"),
        DeclareLaunchArgument("z_step_m", default_value="0.05"),
        DeclareLaunchArgument("yaw_step_deg", default_value="5.0"),

        # Hoehenoffset ueber der aktuellen Position fuer den Default-Setpoint
        # solange die Drohne nicht armed ist.
        DeclareLaunchArgument("takeoff_offset_m", default_value="1.5"),

        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("start_x", default_value="-0.0030704"),
        DeclareLaunchArgument("start_y", default_value="-0.0153477"),
        DeclareLaunchArgument("start_z", default_value="1.36"),
        DeclareLaunchArgument("start_qx", default_value="-0.0012295"),
        DeclareLaunchArgument("start_qy", default_value="0.00490711"),
        DeclareLaunchArgument("start_qz", default_value="-0.7050490"),
        DeclareLaunchArgument("start_qw", default_value="-0.7091406"),

        Node(
            package="control",
            executable="control_console",
            name="control_console",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "state_topic": LaunchConfiguration("state_topic"),
                "local_pose_topic": LaunchConfiguration("local_pose_topic"),
                "setpoint_topic": LaunchConfiguration("setpoint_topic"),

                "camera_front_raw_topic": LaunchConfiguration("camera_front_raw_topic"),
                "camera_front_rect_topic": LaunchConfiguration("camera_front_rect_topic"),
                "camera_down_raw_topic": LaunchConfiguration("camera_down_raw_topic"),
                "camera_down_rect_topic": LaunchConfiguration("camera_down_rect_topic"),

                "arming_service": LaunchConfiguration("arming_service"),
                "set_mode_service": LaunchConfiguration("set_mode_service"),

                "display_rate_hz": LaunchConfiguration("display_rate_hz"),
                "setpoint_publish_rate_hz": LaunchConfiguration("setpoint_publish_rate_hz"),
                "camera_timeout_s": LaunchConfiguration("camera_timeout_s"),
                "state_timeout_s": LaunchConfiguration("state_timeout_s"),
                "pose_timeout_s": LaunchConfiguration("pose_timeout_s"),

                "xy_step_m": LaunchConfiguration("xy_step_m"),
                "z_step_m": LaunchConfiguration("z_step_m"),
                "yaw_step_deg": LaunchConfiguration("yaw_step_deg"),
                "takeoff_offset_m": LaunchConfiguration("takeoff_offset_m"),

                "frame_id": LaunchConfiguration("frame_id"),
                "start_x": LaunchConfiguration("start_x"),
                "start_y": LaunchConfiguration("start_y"),
                "start_z": LaunchConfiguration("start_z"),
                "start_qx": LaunchConfiguration("start_qx"),
                "start_qy": LaunchConfiguration("start_qy"),
                "start_qz": LaunchConfiguration("start_qz"),
                "start_qw": LaunchConfiguration("start_qw"),
            }],
        ),
    ])