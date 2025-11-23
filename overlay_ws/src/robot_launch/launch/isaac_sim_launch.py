import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):

    vision = LaunchConfiguration("vision")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Isaac Sim topics
    isaac_joint_states = "/isaac_joint_states"
    isaac_joint_commands = "/isaac_joint_commands"

    # Xacro args
    xacro_args = {
        "parent": "world",
        "arm": "gen3",
        "prefix": "",
        "dof": "7",
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "sim_isaac": "true",
        "use_fake_hardware": "false",
        "use_internal_bus_gripper_comm": "false",
        "use_external_cable": "false",
        "isaac_joint_commands": isaac_joint_commands,
        "isaac_joint_states": isaac_joint_states,
    }

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="robot_launch")
        .robot_description(mappings=xacro_args)
        .robot_description_semantic(mappings=xacro_args)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits()
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # Move Group Node (IMPORTANT: use_sim_time)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("/joint_states", "/isaac_joint_states")
        ]
    )

    # Robot State Publisher (IMPORTANT: use_sim_time)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/joint_states", "/isaac_joint_states")
        ]
    )

    # RViz (already correct)
    rviz_config_file = os.path.join(
        get_package_share_directory("robot_launch"),
        "config",
        "isaac_rviz.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        arguments=["-d", rviz_config_file],
    )
    
    
    trajectory_bridge = Node(
        package="kinova_isaac",  # Replace with your package name
        executable="trajectory_bridge.py",
        name="trajectory_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time}
        ]
    )


    return [robot_state_publisher, move_group_node, trajectory_bridge,rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
