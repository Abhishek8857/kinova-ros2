#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time
import numpy as np
import json

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

def read_transformed_pose_from_file(file_path):
    """Read the transformed pose from a JSON file and convert it back to PoseStamped."""
    with open(file_path, 'r') as json_file:
        pose_dict = json.load(json_file)
    return dict_to_pose_stamped(pose_dict)

def apply_transform(pose, transform):
    # Extract translation and rotation from the transform
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    # Create a 4x4 transformation matrix from the translation and rotation of the transform
    transform_matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    transform_matrix[0:3, 3] = [translation.x, translation.y, translation.z]

    # Convert the pose position and quaternion into a 4x4 pose matrix
    pose_matrix = quaternion_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
    pose_matrix[0:3, 3] = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

    # Multiply the matrices to get the transformed pose matrix
    transformed_matrix = transform_matrix @ pose_matrix

    # Extract the transformed position and orientation from the matrix
    transformed_pose = PoseStamped()
    transformed_pose.header.frame_id = 'base_link'
    transformed_pose.pose.position.x = transformed_matrix[0, 3]
    transformed_pose.pose.position.y = transformed_matrix[1, 3]
    transformed_pose.pose.position.z = transformed_matrix[2, 3]

    transformed_orientation = quaternion_from_euler(
        transformed_matrix[0, 0], transformed_matrix[1, 1], transformed_matrix[2, 2]
    )
    transformed_pose.pose.orientation.x = transformed_orientation[0]
    transformed_pose.pose.orientation.y = transformed_orientation[1]
    transformed_pose.pose.orientation.z = transformed_orientation[2]
    transformed_pose.pose.orientation.w = transformed_orientation[3]

    return transformed_pose


def image_callback(color_msg, depth_msg):
    # Convert both messages to NumPy arrays
    color_image = image_msg_to_numpy(color_msg)  # RGB image: (height, width, 3)
    depth_image = image_msg_to_numpy(depth_msg)  # Depth image: (height, width)

    # Ensure depth image is reshaped to match the height and width of the color image
    if color_image.shape[:2] != depth_image.shape[:2]:
        get_logger().error('Color and depth images do not have matching sizes.')
        return

    # Normalize depth image to 0-255 and convert it to uint8 (optional)
    # depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Stack the color and depth images along the last axis to create a (height, width, 4) array
    rgbd_image = np.dstack((color_image, depth_image))  # Shape: (height, width, 4)

    # Save the combined RGB-D image as a NumPy array
    np.save('/kinova-ros2/image_transfer/rgbd_image.npy', rgbd_image)

    get_logger().info('Saved RGB-D image (4 channels).')

def image_msg_to_numpy(msg):
    """Convert ROS Image message to NumPy array."""
    height = msg.height
    width = msg.width
    encoding = msg.encoding

    if encoding == 'rgb8':
        np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
    elif encoding == 'mono16' or encoding == '16UC1':  # Handle 16-bit single-channel images
        np_array = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
    else:
        np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))

    return np_array


def convert_matrix_to_pose_stamped(logger, matrix, frame_id="end_effector_link"):
    # Ensure the matrix is 4x4
    if matrix.shape != (4, 4):
        raise ValueError("Input matrix must be a 4x4 transformation matrix.")
    
    # Extract position from the matrix (last column)
    position = matrix[:3, 3]

    # Convert the rotation matrix to quaternion
    quaternion = quaternion_from_matrix(matrix)

    # Create a PoseStamped message
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = frame_id

    # Set position
    pose_goal.pose.position.x = 0.3 # float(position[0]); logger.info(f"position: {position[0]}")
    pose_goal.pose.position.y = 0.3 # float(position[1]); logger.info(f"position: {position[1]}")
    pose_goal.pose.position.z = 0.3 # float(position[2]-0.15); logger.info(f"position: {position[2]}")

    # Set orientation
    pose_goal.pose.orientation.x = 0.0 # float(quaternion[0]); logger.info(f"Quaternion: {quaternion[0]}")
    pose_goal.pose.orientation.y = 0.0 # float(quaternion[1]); logger.info(f"Quaternion: {quaternion[1]}")
    pose_goal.pose.orientation.z = 0.0 # float(quaternion[2]); logger.info(f"Quaternion: {quaternion[2]}")
    pose_goal.pose.orientation.w = 1.0 # float(quaternion[3]); logger.info(f"Quaternion: {quaternion[3]}")

    return pose_goal

def get_best_grasp(logger):
    # Load the numpy array containing grasp poses
    grasp_poses = np.load('/kinova-ros2/image_transfer/predictions_rgbd_image.npz', allow_pickle=True)

    # Choose a grasp pose; if no grasp pose is available, exit the loop
    if grasp_poses['pred_grasps_cam'].item()[-1].shape[0] == 0:
        logger.info('No grasp poses available.')
        return None

    # Choose the grasp pose with the highest score
    pred_grasps_cam = grasp_poses['pred_grasps_cam'].item()[-1]
    scores = grasp_poses['scores'].item()[-1]
    best_grasp_idx = np.argmax(scores)
    best_grasp = pred_grasps_cam[best_grasp_idx]
    # Calculate the first goalpose as 10 cm in z-direction from grasp pose
    goal = np.copy(best_grasp)
    goal[2, 3] -= 0.1  # Add 10 cm in z-direction

    best_grasp_pose = convert_matrix_to_pose_stamped(logger, best_grasp, frame_id="end_effector_link")
    goal_pose = convert_matrix_to_pose_stamped(logger, goal, frame_id="end_effector_link")
    logger.info('Grasp poses processed.')

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, "moveit_py")
    transform = tf_buffer.lookup_transform("base_link", "end_effector_link", rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))

    transformed_best_grasp_pose = apply_transform(best_grasp_pose, transform)
    transformed_goal_pose = apply_transform(goal_pose, transform)

    # return best_grasp_pose, goal_pose
    return transformed_best_grasp_pose, transformed_goal_pose

# # Create subscribers for color and depth_registered image topics
# color_image_sub = message_filters.Subscriber( Image, '/camera/color/image_raw')
# depth_registered_image_sub = message_filters.Subscriber( Image, '/camera/depth_registered/image_rect')

# # Synchronize the color and depth_registered image topics
# ts = message_filters.ApproximateTimeSynchronizer([color_image_sub, depth_registered_image_sub], 10, 0.1)
# ts.registerCallback(image_callback)


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
    # Call the function image_callback to save the RGB-D image
    ###################################################################
    ###################################################################
    # Load the numpy array containing grasp poses
    ###################################################################
    logger.info("Choosing the best grasp pose")
    # best_grasp_pose, offset_goal_pose = get_best_grasp(logger)
    best_grasp, offset_goal = get_best_grasp(logger)
    ###################################################################
    # Choose a grasp pose; if no grasp pose is available, exit the loop
    # by returning finish_grasp_loop = True
    ###################################################################
    # # Create a PoseStamped message
    # pose_goal = PoseStamped()
    # pose_goal.header.frame_id = "base_link"

    # # Set position
    # pose_goal.pose.position.x = 0.3 # float(position[0]); logger.info(f"position: {position[0]}")
    # pose_goal.pose.position.y = 0.3 # float(position[1]); logger.info(f"position: {position[1]}")
    # pose_goal.pose.position.z = 0.3 # float(position[2]-0.15); logger.info(f"position: {position[2]}")

    # # Set orientation
    # pose_goal.pose.orientation.x = 0.0 # float(quaternion[0]); logger.info(f"Quaternion: {quaternion[0]}")
    # pose_goal.pose.orientation.y = 0.0 # float(quaternion[1]); logger.info(f"Quaternion: {quaternion[1]}")
    # pose_goal.pose.orientation.z = 0.0 # float(quaternion[2]); logger.info(f"Quaternion: {quaternion[2]}")
    # pose_goal.pose.orientation.w = 1.0 # float(quaternion[3]); logger.info(f"Quaternion: {quaternion[3]}")

    # # Create a PoseStamped message
    # pose_goal2 = PoseStamped()
    # pose_goal2.header.frame_id = "base_link"

    # # Set position
    # pose_goal2.pose.position.x = -0.3 # float(position[0]); logger.info(f"position: {position[0]}")
    # pose_goal2.pose.position.y = -0.3 # float(position[1]); logger.info(f"position: {position[1]}")
    # pose_goal2.pose.position.z = 0.3 # float(position[2]-0.15); logger.info(f"position: {position[2]}")

    # # Set orientation
    # pose_goal2.pose.orientation.x = 0.0 # float(quaternion[0]); logger.info(f"Quaternion: {quaternion[0]}")
    # pose_goal2.pose.orientation.y = 0.0 # float(quaternion[1]); logger.info(f"Quaternion: {quaternion[1]}")
    # pose_goal2.pose.orientation.z = 0.0 # float(quaternion[2]); logger.info(f"Quaternion: {quaternion[2]}")
    # pose_goal2.pose.orientation.w = 1.0 # float(quaternion[3]); logger.info(f"Quaternion: {quaternion[3]}")
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

    gen3_manipulator.set_goal_state(pose_stamped_msg=best_grasp, pose_link="end_effector_link")

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=20.0)
    ###################################################################
    # The second goalpose is the grasp pose
    # Plan and Execute to the second goalpose
    ###################################################################
    # set plan start state to current state
    logger.info("Moving to the grasp pose")
    gen3_manipulator.set_start_state_to_current_state()

    gen3_manipulator.set_goal_state(pose_stamped_msg=offset_goal, pose_link="end_effector_link")

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=20.0)
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
    # set plan start state to current state
    logger.info("Moving to the goalpose offset by 10 cm in z-direction")
    gen3_manipulator.set_start_state_to_current_state()

    gen3_manipulator.set_goal_state(pose_stamped_msg=offset_goal_pose, pose_link="end_effector_link")

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)
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