import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_cedric = get_package_share_directory('tb4_cedric_pkg_1')

    return LaunchDescription([
        # 1. Gazebo mit deiner Welt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_cedric, 'launch', 'my_turtlebot3_world.launch.py')
            )
        ),

        # 2. Teleop Keyboard im separaten Fenster
        ExecuteProcess(
            cmd=['xterm', '-e', 'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
            output='screen'
        ),

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

        # 5. Lineares Kalman Filter (KF) -> Sendet nativ auf /kf_estimated_pose
        Node(
            package='tb4_cedric_pkg_1',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            output='screen',
            parameters=[{'q_var_pos': 0.002, 'q_var_vel': 0.02, 'r_var_pos': 0.08, 'r_var_vel': 0.15}]
        ),

        # 6. Standard EKF -> Wir mappen /ekf_estimated_pose um auf /ekf_pose_std
        Node(
            package='tb4_cedric_pkg_1',
            executable='extended_kalman_filter_node',
            name='extended_kalman_filter_node',
            output='screen',
            remappings=[('/ekf_estimated_pose', '/ekf_pose_std')],
            parameters=[{'q_var_pos': 0.05, 'q_var_theta': 0.02, 'r_var_odom': 0.08, 'r_var_imu': 0.05}]
        ),

        # 7. EKF mit Landmarke -> Wir mappen /ekf_pose um auf /ekf_pose_landmark
        Node(
            package='tb4_cedric_pkg_1',
            executable='ekf_landmark_node',
            name='ekf_landmark_node',
            output='screen',
            remappings=[('/ekf_pose', '/ekf_pose_landmark')],
            parameters=[{
                'q_var_pos': 0.03, 'q_var_theta': 0.015,
                'r_var_odom': 0.06, 'r_var_landmark_range': 0.04, 'r_var_landmark_bearing': 0.03,
                'target_radius': 0.35, 'tolerance': 0.015, 'stride': 4, 'scan_skip': 1, 'min_votes': 4, 'max_jump_distance': 0.2
            }]
        ),

        # 8. Partikelfilter (PF) -> Wir mappen /pf_estimated_pose um auf /pf_pose
        Node(
            package='tb4_cedric_pkg_1',
            executable='particle_filter_node',
            name='particle_filter_node',
            output='screen',
            remappings=[('/pf_estimated_pose', '/pf_pose')],
            parameters=[{
                'num_particles': 1200, 'q_var_pos': 0.005, 'q_var_theta': 0.005,
                'target_radius': 0.35, 'tolerance': 0.01, 'stride': 4, 'scan_skip': 1, 'min_votes': 4, 'max_jump_distance': 0.8
            }]
        ),

        # 9. Multi-Recorder (Hört auf die neu gemappten Topics)
        Node(
            package='tb4_cedric_pkg_1',
            executable='multi_recorder_node',
            name='multi_recorder_node',
            output='screen'
        )
    ])