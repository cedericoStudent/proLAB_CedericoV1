import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        # 1. Gazebo mit TurtleBot 3 Waffle
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
            )
        ),

        # 2. Dein Recorder Node
        Node(
            package='tb4_cedric_pkg_1',
            executable='recorder_node',
            name='recorder_node',
            output='screen'
        ),

        # 3. Teleop Keyboard in einem neuen Fenster
        ExecuteProcess(
            cmd=['xterm', '-e', 'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
            output='screen'
        )
    ])