
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('rcam_ros2'),
        'params',
        'd415.yaml'
        )
    node=Node(
            package='realsense2_camera', 
            # namespace='came,
            name='rs_node',
            executable='realsense2_camera_node',
            parameters = [config],
            output='screen',
            emulate_tty=True,
        )
    ld.add_action(node)
    return ld
