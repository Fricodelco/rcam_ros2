import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    config = os.path.join(get_package_share_directory('rcam_ros2'), 'params', 'cost.yaml')
    param_substitutions = {
        'use_sim_time': use_sim_time}
    namespace = "costmap"
    configured_params = RewrittenYaml(
            source_file=config,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)
    remappings = [('/voxel_grid', '/costmapvoxel_grid')]

    return LaunchDescription([
        Node(package='nav2_costmap_2d', 
            executable='nav2_costmap_2d_cloud', 
            remappings=remappings
        ),
        Node(package='nav2_costmap_2d', 
            executable='nav2_costmap_2d',
            name="local_costmap", 
            parameters=[configured_params],
        )
    ])
