from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{'fcu_url': 'udp://:14540@127.0.0.1:14557'}]
        ),
        # Gazebo -> ROS Image Bridge
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='gz_image_bridge',
            arguments=['/camera/image_raw'],
            output='screen'
        ),
        # Gazebo -> ROS String Bridge (for LoRa JSON)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_string_bridge',
            arguments=['/lora/ranging@std_msgs/msg/String[gz.msgs.StringMsg'],
            remappings=[('/lora/ranging', '/gz/lora/ranging')],
            output='screen'
        ),
        # LoRa JSON Parser (The missing link)
        Node(
            package='roland_lora',
            executable='lora_bridge.py',
            name='lora_parser',
            output='screen'
        ),
        # EKF Node
        Node(
            package='roland_lora',
            executable='ekf_node',
            name='ekf_node',
            output='screen'
        ),
        # AI Vision Node
        Node(
            package='roland_lora',
            executable='vision_node.py',
            name='vision_node',
            output='screen'
        ),
        # Autoland Mission Node
        Node(
            package='roland_lora',
            executable='autoland_node',
            name='autoland_node',
            output='screen'
        )
    ])
