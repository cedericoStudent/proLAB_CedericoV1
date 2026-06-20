import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_cedric = get_package_share_directory('tb4_cedric_pkg_1')

    # Definiere die Trajektorien-Node separat, um sie verzögern zu können
    trajectory_node = Node(
        package='tb4_cedric_pkg_1',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen'
    )

    # Verzögere den Start des Trajektorien-Generators um 5 Sekunden
    delayed_trajectory_node = TimerAction(
        period=7.0,
        actions=[trajectory_node]
    )

    return LaunchDescription([
        # 1. Gazebo mit deiner Welt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_cedric, 'launch', 'my_turtlebot3_world.launch.py')
            )
        ),

        # 2. Teleop Keyboard im separaten Fenster (Auskommentiert, da automatisierte Trajektorie läuft!)
        # ExecuteProcess(
        #     cmd=['xterm', '-e', 'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
        #     output='screen'
        # ),

        # 3. Gelenkrauschen
        Node(
            package='tb4_cedric_pkg_1',
            executable='joint_noise_node',
            name='joint_noise_node',
            output='screen',
            parameters=[{'noise_std_dev_vel': 0.08, 'noise_std_dev_pos': 0.02}]
        ),

        # 4. Verrauschte Odometrie
        Node(
            package='tb4_cedric_pkg_1',
            executable='noisy_odometry_node',
            name='noisy_odometry_node',
            output='screen'
        ),

        # 5. Lineares Kalman Filter (KF)
        Node(
            package='tb4_cedric_pkg_1',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            output='screen',
            parameters=[{'q_var_pos': 0.002, 'q_var_vel': 0.2, 'r_var_pos': 0.1, 'r_var_vel': 0.15}]
        ),

        # 6. Standard EKF
        Node(
            package='tb4_cedric_pkg_1',
            executable='extended_kalman_filter_node',
            name='extended_kalman_filter_node',
            output='screen',
            parameters=[{'q_var_pos': 0.05, 'q_var_theta': 0.02, 'r_var_odom': 0.08, 'r_var_imu': 0.05}]
        ),

        # 7. EKF mit Landmarke (Nutzt jetzt die relative Odom-Prädiktion)
        Node(
            package='tb4_cedric_pkg_1',
            executable='ekf_landmark_node',
            name='ekf_landmark_node',
            output='screen',
            parameters=[{
                'q_var_pos': 0.01, 'q_var_theta': 0.01,
                'r_var_landmark_range': 0.04, 'r_var_landmark_bearing': 0.03,
                'target_radius': 0.35, 'tolerance': 0.03, 'stride': 4, 'scan_skip': 1, 'min_votes': 4, 'max_jump_distance': 0.8
            }]
        ),

        # 8. Partikelfilter (PF)
        Node(
            package='tb4_cedric_pkg_1',
            executable='particle_filter_node',
            name='particle_filter_node',
            output='screen',
            # remappings=[('/pf_pose', '/pf_estimated_pose')],
            parameters=[{
                'num_particles': 1500, 'q_var_pos': 0.005, 'q_var_theta': 0.005,
                'target_radius': 0.35, 'tolerance': 0.01, 'stride': 4, 'scan_skip': 1, 'min_votes': 4, 'max_jump_distance': 0.5
            }]
        ),

        # 9. Multi-Recorder
        Node(
            package='tb4_cedric_pkg_1',
            executable='multi_recorder_node',
            name='multi_recorder_node',
            output='screen'
        ),

        # 10. Verzögerter Trajektorien-Generator (Startet nach 5 Sekunden Bedenkzeit)
        delayed_trajectory_node
    ])