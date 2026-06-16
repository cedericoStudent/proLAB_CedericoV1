import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Pfad zu DEINEM eigenen Welt-Launchfile auflösen
    pkg_cedric = get_package_share_directory('tb4_cedric_pkg_1')

    return LaunchDescription([
        # 1. DEIN EIGENES LAUNCHFILE AUFRUFEN (Ersetzt turtlebot3_world.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_cedric, 'launch', 'my_turtlebot3_world.launch.py')
            )
        ),

        # 2. Dein Recorder Node
        Node(
            package='tb4_cedric_pkg_1',
            executable='recorder_node',
            name='recorder_node',
            output='screen'
        ),

        # 3. Joint State Noiser Node (Verrauscht die Encoder)
        Node(
            package='tb4_cedric_pkg_1',
            executable='joint_noise_node',
            name='joint_noise_node',
            output='screen',
            parameters=[{
                'noise_std_dev_vel': 0.08,
                'noise_std_dev_pos': 0.02
            }]
        ),

        # 4. Teleop Keyboard in einem neuen Fenster
        ExecuteProcess(
            cmd=['xterm', '-e', 'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
            output='screen'
        ),

        # 5. Noisy Odometry Node
        Node(
            package='tb4_cedric_pkg_1',
            executable='noisy_odometry_node',
            name='noisy_odometry_node',
            output='screen'
        ),

        # 6. Kalman Filter Node bzw. Extended Kalman Filter Node bzw. Particle Filter Node
        #Node(
        #    package='tb4_cedric_pkg_1',
        #    executable='extended_kalman_filter_node',
        #    name='extended_kalman_filter_node',
        #    output='screen'
        #),

        #Landmark testen
        #Node(
        #    package='tb4_cedric_pkg_1',
        #    executable='landmark_test_node',
        #    name='landmark_test_node',
        #    output='screen'
        #),

        Node(
            package='tb4_cedric_pkg_1',
            executable='ekf_landmark_node',
            name='ekf_landmark_node',
            output='screen',
            parameters=[{
                'q_var_pos': 0.03,
                'q_var_theta': 0.015,
                'r_var_odom': 0.06,
                'r_var_landmark_range': 0.04,
                'r_var_landmark_bearing': 0.03,
                # Geometrische Detektions- und Filterparameter:
                'target_radius': 0.35,
                'tolerance': 0.015,
                'stride': 4,
                'scan_skip': 1,
                'min_votes': 4,
                'max_jump_distance': 0.2
            }]
        ),
        Node(
            package='tb4_cedric_pkg_1',
            executable='particle_filter_node',
            name='particle_filter_node',
            output='screen',
            remappings=[
                ('/pf_estimated_pose', '/ekf_estimated_pose')
            ],
            parameters=[{
                'num_particles': 500,
                'q_var_pos': 0.04,        # Partikelfilter vertragen oft etwas mehr Prozessrauschen zum "Spreizen"
                'q_var_theta': 0.02,
                'r_var_landmark_range': 0.20,
                'r_var_landmark_bearing': 0.03,
                'target_radius': 0.35,
                'tolerance': 0.015,
                'stride': 4,
                'scan_skip': 1,
                'min_votes': 4,
                'max_jump_distance': 0.2
            }]
        )
    ])