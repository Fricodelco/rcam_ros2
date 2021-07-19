import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'empty_world/empty.model'
    world = os.path.join(get_package_share_directory('rcam_sim'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('rcam_sim'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    sdf = os.path.join(get_package_share_directory('rcam_sim'), 'models/dif_drive/', 'model.sdf')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
        Node(package='rcam_sim', 
            executable='spawn_robot.py', 
            arguments=["--robot_sdf", sdf,
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0"],
        )
    ])