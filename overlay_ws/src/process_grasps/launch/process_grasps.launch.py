from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='process_grasps',
            executable='process_grasps',
            name='process_grasps',
            output='screen',
          )
    ])
