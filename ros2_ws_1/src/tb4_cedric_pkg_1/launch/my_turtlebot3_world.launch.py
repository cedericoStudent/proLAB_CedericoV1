import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Pfade auflösen
    pkg_cedric = get_package_share_directory('tb4_cedric_pkg_1')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # HIER STEHT JETZT DIREKT DER PFAD ZU DEINER WELT-DATEI
    world = os.path.join(pkg_cedric, 'worlds', 'waffle.model')

    # Umgebungsvariablen setzen, damit Gazebo die Meshes findet
    tb3_model_path = os.path.join(pkg_tb3_gazebo, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += f":{tb3_model_path}"
    else:
        os.environ['GAZEBO_MODEL_PATH'] = tb3_model_path

    os.environ["TURTLEBOT3_MODEL"] = "waffle"

    return LaunchDescription([
        # 1. Gazebo Server mit deiner Welt starten
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),

        # 2. Gazebo Client (GUI) starten
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),

        # 3. Robot State Publisher aus dem TB3-Paket laden
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])