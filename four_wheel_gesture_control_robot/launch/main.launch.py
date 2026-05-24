import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'four_wheel_gesture_control'
    pkg_share = get_package_share_directory(pkg_name)

    # Path to the Xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    # Generate URDF from Xacro
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # Config file for parameters
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # 1. Gazebo classic launch
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'pause': 'false'}.items()
    )

    # 2. Node: robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 3. Node: spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'four_wheel_robot'],
        output='screen'
    )

    # 4. Include hand gesture control launch file
    gesture_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gesture_control.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        gesture_control_launch
    ])
