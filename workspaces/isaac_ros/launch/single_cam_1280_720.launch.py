##Launch File auf meinem Jetson (container)

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([

        ComposableNodeContainer(
            name='isaac_ros_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[

                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                    name='rectify_1',
                    parameters=[{
                        'output_width': 1280,               #Geändert!!!!
                        'output_height': 720,               #Geändert!!!
                    }],
                    remappings=[
                        ('image_raw',        '/camera_1/image_raw'),
                        ('camera_info',      '/camera_1/camera_info'),
                        ('image_rect',       '/camera_1/image_rect'),
                        ('camera_info_rect', '/camera_1/camera_info_rect'),
                    ],
                ),

                ComposableNode(
                    package='isaac_ros_apriltag',
                    plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                    name='apriltag_1',
                    parameters=[{
                        'family':   'tag36h11',
                        'size':     0.170,
                        'max_tags': 40,
                    }],
                    remappings=[
                        ('image',       '/camera_1/image_rect'),
                        ('camera_info', '/camera_1/camera_info_rect'),
                        ('tag_detections', '/camera_1/tag_detections'),
                    ],
                ),


            ],
            output='screen',
        ),

    ])