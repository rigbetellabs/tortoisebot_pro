import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Joy node
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='tortoisebot_pro_joy_node',
            parameters=[{'dev_ff': '/dev/input/haptics'}, {'dev': '/dev/input/joy'}],
            output='screen',
            respawn=True
        ),

        # Auto joy teleop node
        launch_ros.actions.Node(
            package='auto_joy_teleop', executable='auto_joy_teleop', name='auto_joy_teleop',
            output='screen'
        )
    ])
