from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_file',
            default_value='path/to/ros2_bag',
            description='Path to the ROS 2 bag file.'
        ),
        DeclareLaunchArgument(
            'data_save_dir',
            default_value='path/to/save/data',
            description='Directory to save the data.'
        ),
        Node(
            package='ros2_bag_processor',
            executable='bag_processor',
            name='bag_processor',
            output='screen',
            parameters=[
                {'bag_file': LaunchConfiguration('bag_file')},
                {'data_save_dir': LaunchConfiguration('data_save_dir')}
            ]
        ),
    ])
