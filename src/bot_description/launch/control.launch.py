from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',  # Opens in a new terminal window
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/cmd_vel', '/cmd_vel')  # Remap if needed
            ],
        )
    ])
