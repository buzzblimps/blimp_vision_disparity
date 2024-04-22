import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='MaryO',
            description='Namespace for the node(s)'
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='camera3',
            description='Camera ID'
        ),
        Node(
            package='blimp_vision_disparity',
            executable='blimp_vision_disparity_node',
            name='blimp_vision_disparity_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                os.path.join(get_package_share_directory('blimp_vision_disparity'), 'param', 'elp_config_laptop.yaml'),
                {'camera_id': LaunchConfiguration('camera_id')}
            ],
            output='screen'
        )
    ])
