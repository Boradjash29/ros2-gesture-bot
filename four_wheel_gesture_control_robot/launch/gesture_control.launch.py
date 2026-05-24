import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'four_wheel_gesture_control'
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    hand_gesture_node = Node(
        package=pkg_name,
        executable='hand_gesture_node.py',
        name='hand_gesture_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    differential_drive_node = Node(
        package=pkg_name,
        executable='differential_drive_node.py',
        name='differential_drive_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    robot_visualizer_node = Node(
        package=pkg_name,
        executable='robot_visualizer_node.py',
        name='robot_visualizer_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        hand_gesture_node,
        differential_drive_node,
        robot_visualizer_node
    ])
