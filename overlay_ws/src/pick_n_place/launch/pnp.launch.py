from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gen3", package_name="robot_launch").to_dict()

    # MTC Demo node
    pick_place_routine = Node(
        package="pick_n_place",
        executable="pick_n_place",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_routine])
