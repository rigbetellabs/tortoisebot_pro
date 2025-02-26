import os

from launch.actions import  IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    stats_dir = os.path.join(get_package_share_directory('rbl_robot_stats'), 'launch')
    network_data_streamer_node = launch_ros.actions.Node(
        package='tortoisebotpro_firmware',
        executable='network_status_publisher',
        name='network_data',
    )
    goal_status_publisher_node = launch_ros.actions.Node(
        package='tortoisebotpro_firmware',
        executable='goal_status_publisher',
        name='goal_status_streamer',
    )

    stats_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stats_dir, 'robot_stats.launch.py')),
    )
    
    return launch.LaunchDescription([
        network_data_streamer_node,
        goal_status_publisher_node,
        stats_node
    ])
    
