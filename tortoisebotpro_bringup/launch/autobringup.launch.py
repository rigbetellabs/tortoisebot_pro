import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression,Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription,ExecuteProcess, TimerAction,RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import launch_ros
from launch_ros.descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess
from launch.actions import TimerAction 

def generate_launch_description():
  pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebotpro_description').find('tortoisebotpro_description')
  navigation_dir = os.path.join(get_package_share_directory('tortoisebotpro_navigation'), 'launch')
  rviz_launch_dir=os.path.join(get_package_share_directory('tortoisebotpro_description'), 'launch')
  gazebo_launch_dir=os.path.join(get_package_share_directory('tortoisebotpro_gazebo'), 'launch')
  ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch')
  # camera_launch_dir=os.path.join(get_package_share_directory('v4l2_camera'), 'launch')
  # cartographer_launch_dir=os.path.join(get_package_share_directory('tortoisebotpro_slam'), 'launch')
  micro_ros_launch_dir=os.path.join(get_package_share_directory('tortoisebotpro_firmware'), 'launch')
  prefix_address = get_package_share_directory('tortoisebotpro_navigation') 
  default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebotpro.xacro')
  default_rviz_config_path = os.path.join(get_package_share_directory('tortoisebotpro_description'), 'rviz/tortoisebotpro_sensor_display.rviz')
 
  
  params_file_sim = os.path.join(prefix_address, 'config', 'nav2_params_simulation.yaml')
  params_file_robot = os.path.join(prefix_address, 'config', 'nav2_params.yaml')
  ros_ip = LaunchConfiguration('ros_ip')
  map_file=LaunchConfiguration('map')
  map_directory = os.path.join(get_package_share_directory(
        'tortoisebotpro_navigation'), 'maps','maps.yaml')
  use_sim_time=LaunchConfiguration('use_sim_time')
  exploration=LaunchConfiguration('exploration')   
  enable_camera=LaunchConfiguration('enable_camera')


  esptool_cmd = ExecuteProcess(
        cmd=['esptool', '--port','/dev/esp', 'read_mac'],
        name='esptool_read_mac',
        output='screen'
    )
  rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters= [{'use_sim_time': use_sim_time}],

    )

  state_publisher_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time,
                              'model': default_model_path}.items())

  gazebo_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')),
        launch_arguments={'params_file': params_file_robot}.items())
  
  ydlidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'ydlidar_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())
  
  micro_ros_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(micro_ros_launch_dir, 'micro_ros.launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())
  
  pose_publisher_cmd = Node(
    package='tortoisebotpro_bringup',  # Replace with your actual package name
    executable='robot_pose_publisher',
    name='robot_pose_publisher',
    output='screen',
    parameters=[{
        'base_frame': 'base_link',
        'reference_frame': 'map',
        'publish_rate': 20.0,
        'topic_name': 'robot_pose',
        'use_sim_time': use_sim_time
    }], 
    )
  camera_drive_node = Node(
      package='v4l2_camera',
      executable='v4l2_camera_node',
      name='camera_publisher',
      condition=IfCondition(PythonExpression(['not ', use_sim_time, ' and ', enable_camera])),
      parameters=[{
          'image_size': [160, 120],
          'pixel_format': 'yuyv',
          'camera_name': 'camera',
          'camera_info_url': '',
          'frame_id': 'camera_link',
          'video_device': '/dev/video0',    # Set timeout for device operations
          'camera_frame_id': 'camera_link',
          'output_encoding': 'bgr8',
      }],
      remappings=[
          ('image_raw', '/camera/image_raw'),
          ('camera_info', '/camera/camera_info'),
      ],
      # Disable respawn to prevent endless error loops
      respawn=False,
      output='screen'
    )

  ros_nav_status = Node(
        package='tortoisebotpro_bringup',
        executable='goal_status_publisher',
        name='ros_nav_status',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        output='screen',
    )
  nav2_goal_canceller_node = Node(
        package='tortoisebotpro_bringup',
        executable='nav2_goal_canceller',
        name='nav2_goal_canceller_node',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        output='screen',
        respawn=True,
    )
  ros_tcp_endpoint_node= Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='tcp_endpoint',
            parameters=[
                {'ROS_IP': ros_ip},
                {'ROS_TCP_PORT': 10000}
            ],
            respawn=True,  # Automatically restart if node dies
            respawn_delay=1.0,  # Wait 2 seconds before restart  # Maximum 10 restart attempts
            output='screen'
        )
  start_rest_after_esptool = RegisterEventHandler(
        OnProcessExit(
            target_action=esptool_cmd,
            on_exit=[
                    rviz_node,
                    state_publisher_launch_cmd,
                    camera_drive_node,
                    gazebo_launch_cmd,
                    ydlidar_launch_cmd,
                    navigation_launch_cmd,
                    micro_ros_launch_cmd,
                    pose_publisher_cmd ,
                    ros_tcp_endpoint_node,
                    ros_nav_status,
                    nav2_goal_canceller_node,
            ]
        )
    )
  delayed_launch_actions = [
        TimerAction(
            period=6.0,  # Wait 5 seconds after launch starts
            actions=[
                    rviz_node,
                    state_publisher_launch_cmd,
                    camera_drive_node,
                    gazebo_launch_cmd,
                    ydlidar_launch_cmd,
                    navigation_launch_cmd,
                    micro_ros_launch_cmd,
                    pose_publisher_cmd ,
                    ros_tcp_endpoint_node,
                    ros_nav_status,
                    nav2_goal_canceller_node,
            ]
        )
    ]
  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='exploration', default_value='True',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                          description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='map',default_value=map_directory,
                                          description='Map to be used'),
    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
   launch.actions.DeclareLaunchArgument(name='ros_ip', default_value='192.168.0.105',
                                    description='ROS IP address for TCP endpoint'),
    launch.actions.DeclareLaunchArgument(name='enable_camera', default_value='False',
        description='Flag to enable camera (set to False to disable camera)'),
    Node(
        package='nav2_map_server',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file}
                    ]),
    Node(
        package='nav2_lifecycle_manager',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]),
    esptool_cmd,
    start_rest_after_esptool
  ]
)