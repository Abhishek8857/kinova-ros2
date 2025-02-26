#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)


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

    # ###########################################################################
    # # Plan 1 - set states with predefined string
    # ###########################################################################

    # # set plan start state using predefined state
    # gen3_manipulator.set_start_state(configuration_name="vertical")

    # # set pose goal using predefined state
    # gen3_manipulator.set_goal_state(configuration_name="home")

    # # plan to goal
    # plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)

    # # set plan start state using predefined state
    # gen3_gripper.set_start_state_to_current_state()
    # # gen3_gripper.set_start_state(configuration_name="Open")

    # # set pose goal using predefined state
    # gen3_gripper.set_goal_state(configuration_name="Close")

    # # plan to goal
    # plan_and_execute(gen3, gen3_gripper, logger, sleep_time=10.0)

    # # set plan start state using predefined state
    # # gen3_gripper.set_start_state_to_current_state()
    # gen3_gripper.set_start_state(configuration_name="Close")

    # # set pose goal using predefined state
    # gen3_gripper.set_goal_state(configuration_name="Open")

    # # plan to goal
    # plan_and_execute(gen3, gen3_gripper, logger, sleep_time=10.0)

    # ###########################################################################
    # # Plan 2 - set goal state with RobotState object
    # ###########################################################################

    # instantiate a RobotState instance using the current robot model
    robot_model = gen3.get_robot_model()
    robot_state = RobotState(robot_model)

    # # randomize the robot state
    # robot_state.set_to_random_positions()

    # # set plan start state to current state
    # gen3_manipulator.set_start_state_to_current_state()

    # # set goal state to the initialized robot state
    # logger.info("Set goal state to the initialized robot state")
    # gen3_manipulator.set_goal_state(robot_state=robot_state)

    # # plan to goal
    # plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)

    # ###########################################################################
    # # Plan 3 - set goal state with PoseStamped message
    # ###########################################################################

    # # set plan start state to current state
    # gen3_manipulator.set_start_state_to_current_state()

    # # set pose goal with PoseStamped message
    # from geometry_msgs.msg import PoseStamped

    # pose_goal = PoseStamped()
    # pose_goal.header.frame_id = "base_link"
    # pose_goal.pose.orientation.w = 1.0
    # pose_goal.pose.position.x = 0.28
    # pose_goal.pose.position.y = -0.2
    # pose_goal.pose.position.z = 0.5
    # gen3_manipulator.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")

    # # plan to goal
    # plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################

    # set plan start state to current state
    gen3_manipulator.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    joint_values = {
        "joint_1": 0.0,
        "joint_2": -0.9443,
        "joint_3": -3.15353,
        "joint_4": -2.1302,
        "joint_5": 0.00588,
        "joint_6": -1.2077,
        "joint_7": 1.5503,
    }

    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=gen3.get_robot_model().get_joint_model_group("manipulator"),
    )
    gen3_manipulator.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(gen3, gen3_manipulator, logger, sleep_time=10.0)

    # ###########################################################################
    # # Plan 5 - Planning with Multiple Pipelines simultaneously
    # ###########################################################################

    # # set plan start state to current state
    # gen3_manipulator.set_start_state_to_current_state()

    # # set pose goal with PoseStamped message
    # gen3_manipulator.set_goal_state(configuration_name="retract")

    # # initialise multi-pipeline plan request parameters
    # multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
    #     gen3, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    # )

    # # plan to goal
    # plan_and_execute(
    #     gen3,
    #     gen3_manipulator,
    #     logger,
    #     multi_plan_parameters=multi_pipeline_plan_request_params,
    #     sleep_time=10.0,
    # )


if __name__ == "__main__":
    main()