from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='kinova_ops',
        #     executable='kinova_ops',
        #     name='kinova_ops',
        #     output='screen',
        # ),
        Node(
            package='kinova_ops',
            executable='kinova_ops3',
            name='kinova_ops3',
            output='screen',
        ),
        Node(
            package='kinova_ops',
            executable='kinova_ops2',
            name='kinova_ops2',
            output='screen',
        )

    ])
