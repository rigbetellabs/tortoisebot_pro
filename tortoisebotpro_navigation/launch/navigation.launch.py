import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
  prefix_address = get_package_share_directory('tortoisebotpro_navigation') 
  params_file= os.path.join(prefix_address, 'config', 'nav2_params.yaml')

  cartographer_launch_dir = os.path.join(get_package_share_directory('tortoisebotpro_slam'), 'launch')
  params_file_robot = os.path.join(get_package_share_directory('tortoisebotpro_slam'), 'config', 'slam.lua')

  exploration = LaunchConfiguration('exploration', default='True')

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  map_dir = LaunchConfiguration(
      'map',
      default=os.path.join(
          get_package_share_directory('tortoisebotpro_navigation'),
          'maps',
          'room2.yaml'))

  param_file_name = 'nav2_params.yaml'
  param_dir = LaunchConfiguration(
      'params_file',
      default=os.path.join(
          get_package_share_directory('tortoisebotpro_navigation'),
          'config',
          param_file_name))
  
  navigation_launch_cmd=IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        )
  cartographer_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_launch_dir, 'cartographer.launch.py')),
        launch_arguments={'params_file': params_file_robot,
                          'exploration': exploration,
                          'use_sim_time': use_sim_time}.items())

  return LaunchDescription([
    navigation_launch_cmd,
    cartographer_launch_cmd   
  ]
)