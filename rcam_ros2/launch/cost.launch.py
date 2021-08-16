import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
def generate_launch_description():
    bringup_dir = get_package_share_directory('rcam_ros2')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename', default=os.path.join(bringup_dir,'params', 'navigate_w_replanning_and_recovery.xml')) 
    params_file = LaunchConfiguration('params_file' ,default=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'))
    autostart = LaunchConfiguration('autostart',default=True)
    config = os.path.join(get_package_share_directory('rcam_ros2'), 'params', 'cost.yaml')
    param_substitutions = {
        'use_sim_time': use_sim_time}
    namespace = LaunchConfiguration('namespace' ,default="")
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
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'navigation.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),
    ])
