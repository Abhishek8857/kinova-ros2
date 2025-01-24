#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time
import numpy as np
import json
import os

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
import message_filters
from sensor_msgs.msg import Image
from tf_transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseStamped
from moveit.core.kinematic_constraints import construct_joint_constraint
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_multiply, quaternion_from_euler, quaternion_matrix



grasps_json_file = '/root/ws/kinova_repos/kinova-transfer/transformed_pose.json'
grasps_numpy_file = '/root/ws/kinova_repos/kinova-transfer/predictions_rgbd_image.npz'


def dict_to_pose_stamped(pose_dict):
    """Convert a dictionary back to a PoseStamped message."""
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = pose_dict['header']['frame_id']
    pose_stamped.header.stamp.sec = pose_dict['header']['stamp']['sec']
    pose_stamped.header.stamp.nanosec = pose_dict['header']['stamp']['nanosec']
    pose_stamped.pose.position.x = pose_dict['pose']['position']['x']
    pose_stamped.pose.position.y = pose_dict['pose']['position']['y']
    pose_stamped.pose.position.z = pose_dict['pose']['position']['z']
    pose_stamped.pose.orientation.x = pose_dict['pose']['orientation']['x']
    pose_stamped.pose.orientation.y = pose_dict['pose']['orientation']['y']
    pose_stamped.pose.orientation.z = pose_dict['pose']['orientation']['z']
    pose_stamped.pose.orientation.w = pose_dict['pose']['orientation']['w']
    return pose_stamped

def read_transformed_pose_from_file(timeout=600):
    """Read the transformed pose from a JSON file and convert it back to PoseStamped. Wait for the file to be generated."""
    start_time = time.time()
    
    # Wait until the file exists or until the timeout occurs
    while not os.path.exists(grasps_json_file):
        if time.time() - start_time > timeout:
            raise FileNotFoundError(f"File {grasps_json_file} was not created within {timeout} seconds.")
        time.sleep(1)  # Check every 1 second

    # Once the file exists, read and parse it
    with open(grasps_json_file, 'r') as json_file:
        pose_dict = json.load(json_file)
    
    return dict_to_pose_stamped(pose_dict)

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    logger.info("# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #")

    time.sleep(sleep_time)


def grasp_loop(gen3, gen3_gripper, gen3_manipulator, logger):
    ###################################################################
    # Move to named pose "Observe"
    ###################################################################
    # set plan start state using predefined state
    logger.info("Moving to named pose 'Observe'")
    gen3_manipulator.set_start_state_to_current_state()

    # set constraints message
    joint_values = {
        "joint_1": -0.78137,
        "joint_2": -0.95269,
        "joint_3": -3.15353,
        "joint_4": -2.34621,
        "joint_5": 0.00588,
        "joint_6": -1.14074,
        "joint_7": 1.55037,
    }
    robot_model = gen3.get_robot_model()
    robot_state = RobotState(robot_model)
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=gen3.get_robot_model().get_joint_model_group("manipulator"),
    )
    gen3_manipulator.set_goal_state(motion_plan_constraints=[joint_constraint]) #observe [−0.78137,−0.95269,−3.15353,−2.34621,0.00588,−1.14074,1.55037]

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)    
    ###################################################################
    # Open the gripper
    ###################################################################    
    # set plan start state using predefined state
    logger.info("Opening the gripper")
    gen3_gripper.set_start_state_to_current_state()

    # set pose goal using predefined state
    gen3_gripper.set_goal_state(configuration_name="Open")

    # plan to goal
    plan_and_execute(gen3, gen3_gripper, logger, sleep_time=10.0)
    ###################################################################
    # Delete the grasps_json_file and grasps_numpy_file containing the 
    # grasp poses to indicate that the grasp loop is finished
    ###################################################################
    if os.path.exists(grasps_json_file):
        os.remove(grasps_json_file)
    if os.path.exists(grasps_numpy_file):
        os.remove(grasps_numpy_file)
    ###################################################################
    # Load the numpy array containing grasp poses
    ###################################################################
    ###################################################################
    # Choose a grasp pose; if no grasp pose is available, exit the loop
    # by returning finish_grasp_loop = True
    ###################################################################

    ###################################################################
    # Calculate the first goalpose as 10 cm in z-direction from grasp pose
    # Multiply the 4x4 matrix of the grasp pose by the 4x1 vector [0, 0, 0.1, 0]
    ###################################################################
    ###################################################################
    # Plan and Execute to the first goalpose
    ###################################################################
    # set plan start state to current state
    logger.info("Moving to the goalpose offset by 10 cm in z-direction")
    gen3_manipulator.set_start_state_to_current_state()

    best_grasp = read_transformed_pose_from_file()

    gen3_manipulator.set_goal_state(pose_stamped_msg=best_grasp, pose_link="end_effector_link")

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=20.0)
    ###################################################################
    # The second goalpose is the grasp pose
    # Plan and Execute to the second goalpose
    ###################################################################
    # # set plan start state to current state
    # logger.info("Moving to the grasp pose")
    # gen3_manipulator.set_start_state_to_current_state()

    # gen3_manipulator.set_goal_state(pose_stamped_msg=offset_goal, pose_link="end_effector_link")

    # # plan to goal
    # plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=20.0)
    ###################################################################
    # Close the gripper
    ###################################################################    
    # set plan start state using predefined state
    logger.info("Closing the gripper")
    gen3_gripper.set_start_state_to_current_state()

    # set pose goal using predefined state
    gen3_gripper.set_goal_state(configuration_name="Close")

    # plan to goal
    plan_and_execute(gen3, gen3_gripper, logger, sleep_time=10.0)
    ###################################################################
    # Move up 30 cm in z-direction
    ###################################################################
    # # set plan start state to current state
    # logger.info("Moving to the goalpose offset by 10 cm in z-direction")
    # gen3_manipulator.set_start_state_to_current_state()

    # gen3_manipulator.set_goal_state(pose_stamped_msg=offset_goal, pose_link="end_effector_link")

    # # plan to goal
    # plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)
    ###################################################################
    # Move to the named pose "Drop"
    ###################################################################
    # [0.79267,0.02858,−3.14867,−1.92693,−0.00354,−1.19897,1.59179]
    # set plan start state using predefined state
    logger.info("Moving to named pose 'Drop'")
    gen3_manipulator.set_start_state_to_current_state()

    joint_values = {
        "joint_1": 0.79267,
        "joint_2": 0.02858,
        "joint_3": -3.14867,
        "joint_4": -1.92693,
        "joint_5": -0.00354,
        "joint_6": -1.19897,
        "joint_7": 1.59179,
    }
    robot_model = gen3.get_robot_model()
    robot_state = RobotState(robot_model)
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=gen3.get_robot_model().get_joint_model_group("manipulator"),
    )
    gen3_manipulator.set_goal_state(motion_plan_constraints=[joint_constraint]) #observe [−0.78137,−0.95269,−3.15353,−2.34621,0.00588,−1.14074,1.55037]

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0) 
    ###################################################################
    # Open the gripper
    ###################################################################    
    # set plan start state using predefined state
    logger.info("Opening the gripper")
    gen3_gripper.set_start_state_to_current_state()
    # gen3_gripper.set_start_state(configuration_name="Open")

    # set pose goal using predefined state
    gen3_gripper.set_goal_state(configuration_name="Open")

    # plan to goal
    plan_and_execute(gen3, gen3_gripper, logger, sleep_time=10.0)
    return False

def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    gen3 = MoveItPy(node_name="moveit_py")
    gen3_manipulator = gen3.get_planning_component("manipulator")
    gen3_gripper = gen3.get_planning_component("gripper")
    logger.info("MoveItPy instance created")

    finish_grasp_loop = False
    while not finish_grasp_loop:
        finish_grasp_loop = grasp_loop(gen3, gen3_gripper, gen3_manipulator, logger)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()