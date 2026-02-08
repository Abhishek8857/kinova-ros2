#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <algorithm>
#include <map>
#include <mutex>
#include <thread>

#include "agent_action_interface/action/execute_motion.hpp"

using ExecuteMotion = agent_action_interface::action::ExecuteMotion;
using GoalHandleExecuteMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;

class MotionActionServer : public rclcpp::Node
{
public:
    MotionActionServer()
        : Node("motion_action_server")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Motion Action Server...");

        arm_group_name_ = this->declare_parameter<std::string>("arm_group_name", "manipulator");
        gripper_group_name_ = this->declare_parameter<std::string>("gripper_group_name", "gripper");
        gripper_open_target_ = this->declare_parameter<std::string>("gripper_open_target", "Open");
        gripper_close_target_ = this->declare_parameter<std::string>("gripper_close_target", "Close");

        // If gripper_joint_name is empty, fallback is disabled.
        gripper_joint_name_ = this->declare_parameter<std::string>("gripper_joint_name", "");
        gripper_open_value_ = this->declare_parameter<double>("gripper_open_value", 0.0);
        gripper_close_value_ = this->declare_parameter<double>("gripper_close_value", 1.0);

        // Set to -1 to disable. For a 7-DOF arm, set to 7.
        joint_dof_expected_ = this->declare_parameter<int>("joint_dof_expected", -1);

        // Grasp planning parameters
        default_pre_grasp_distance_ = this->declare_parameter<double>("default_pre_grasp_distance", 0.15);
        default_lift_distance_ = this->declare_parameter<double>("default_lift_distance", 0.15);
        grasp_approach_velocity_ = this->declare_parameter<double>("grasp_approach_velocity", 0.05);
        grasp_close_velocity_ = this->declare_parameter<double>("grasp_close_velocity", 0.3);
        grasp_lift_velocity_ = this->declare_parameter<double>("grasp_lift_velocity", 0.05);

        // Planner configuration parameters
        joint_planner_pipeline_ = this->declare_parameter<std::string>("joint_planner_pipeline", "ompl");
        joint_planner_id_ = this->declare_parameter<std::string>("joint_planner_id", "RRTConnect");
        joint_planning_time_ = this->declare_parameter<double>("joint_planning_time", 5.0);
        
        pose_planner_pipeline_ = this->declare_parameter<std::string>("pose_planner_pipeline", "pilz_industrial_motion_planner");
        pose_planner_id_ = this->declare_parameter<std::string>("pose_planner_id", "LIN");
        pose_planning_time_ = this->declare_parameter<double>("pose_planning_time", 5.0);
        pose_use_ompl_fallback_ = this->declare_parameter<bool>("pose_use_ompl_fallback", true);

        // We delay initializing MoveGroupInterface until first use.

        action_server_ = rclcpp_action::create_server<ExecuteMotion>(
            this,
            "execute_motion",
            std::bind(&MotionActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MotionActionServer::handle_accepted, this, std::placeholders::_1)
        );

        AddWorkspace();
    }

private:
    struct MotionOutcome
    {
        bool success{false};
        std::string error_code{"FAILURE"};
        std::string error_description{"Motion failed."};
    };

    // Add collision objects 
    void AddWorkspace()
    {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::msg::CollisionObject> workspace_elements;

        std::string frame_id = "world";

        // Add Base to the Robot
        moveit_msgs::msg::CollisionObject base;
        base.id = "base";
        base.header.frame_id = frame_id;

        shape_msgs::msg::SolidPrimitive base_primitives;
        base_primitives.type = base_primitives.BOX;
        base_primitives.dimensions = {0.8, 1.0, 0.01};

        geometry_msgs::msg::Pose base_pose;
        base_pose.orientation.w = 1.0;
        base_pose.position.z = -0.05;
        base_pose.position.x = 0.1;


        base.primitives.push_back(base_primitives);
        base.primitive_poses.push_back(base_pose);
        base.operation = base.ADD;
        workspace_elements.push_back(base);


        // Add Top Wall 
        moveit_msgs::msg::CollisionObject roof;
        roof.id = "roof";
        roof.header.frame_id = frame_id;

        shape_msgs::msg::SolidPrimitive roof_primitives;
        roof_primitives.type = roof_primitives.BOX;
        roof_primitives.dimensions = {0.8, 1.0, 0.01};

        geometry_msgs::msg::Pose roof_pose;
        roof_pose.orientation.w = 1.0;
        roof_pose.position.z = 1.44;
        roof_pose.position.x = 0.1;


        roof.primitives.push_back(roof_primitives);
        roof.primitive_poses.push_back(roof_pose);
        roof.operation = roof.ADD;
        workspace_elements.push_back(roof);

        // Add Back wall
        moveit_msgs::msg::CollisionObject back_wall;
        back_wall.id = "back_wall";
        back_wall.header.frame_id = frame_id;

        shape_msgs::msg::SolidPrimitive back_wall_primitives;
        back_wall_primitives.type = back_wall_primitives.BOX;
        back_wall_primitives.dimensions = {0.01, 1.01, 1.5};

        geometry_msgs::msg::Pose back_wall_pose;
        back_wall_pose.orientation.w = 1.0;
        back_wall_pose.position.x = -0.3;
        back_wall_pose.position.y = 0.0;
        back_wall_pose.position.z = 0.695;

        back_wall.primitives.push_back(back_wall_primitives);
        back_wall.primitive_poses.push_back(back_wall_pose);
        back_wall.operation = back_wall.ADD;
        workspace_elements.push_back(back_wall);

        // Add Left Wall
        moveit_msgs::msg::CollisionObject left_wall;
        left_wall.id = "left_wall";
        left_wall.header.frame_id = frame_id;

        shape_msgs::msg::SolidPrimitive left_wall_primitives;
        left_wall_primitives.type = left_wall_primitives.BOX;
        left_wall_primitives.dimensions = {0.8, 0.01, 1.5};

        geometry_msgs::msg::Pose left_wall_pose;
        left_wall_pose.orientation.w = 1.0;
        left_wall.pose.position.x = 0.1;
        left_wall.pose.position.y = -0.5;
        left_wall.pose.position.z = 0.695;

        left_wall.primitives.push_back(left_wall_primitives);
        left_wall.primitive_poses.push_back(left_wall_pose);
        left_wall.operation = left_wall.ADD;
        workspace_elements.push_back(left_wall);

        // Add Right Wall
        moveit_msgs::msg::CollisionObject right_wall;
        right_wall.id = "right_wall";
        right_wall.header.frame_id = frame_id;

        shape_msgs::msg::SolidPrimitive right_wall_primitives;
        right_wall_primitives.type = right_wall_primitives.BOX;
        right_wall_primitives.dimensions = {0.8, 0.01, 1.5};

        geometry_msgs::msg::Pose right_wall_pose;
        right_wall_pose.orientation.w = 1.0;
        right_wall_pose.position.x = 0.1;
        right_wall_pose.position.y = 0.5;
        right_wall_pose.position.z = 0.695;

        right_wall.primitives.push_back(right_wall_primitives);
        right_wall.primitive_poses.push_back(right_wall_pose);
        right_wall.operation = right_wall.ADD;
        workspace_elements.push_back(right_wall);

        planning_scene_interface.applyCollisionObjects(workspace_elements);
    }

    void log_pose(const std::string& label, const geometry_msgs::msg::Pose& p)
    {
        RCLCPP_INFO(this->get_logger(),
            "%s pose: position [x=%.3f, y=%.3f, z=%.3f], orientation [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
            label.c_str(),
            p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        );
    }

    void configure_for_joint_planning()
    {
        if (!arm_move_group_) return;
        arm_move_group_->setPlanningPipelineId(joint_planner_pipeline_);
        arm_move_group_->setPlannerId(joint_planner_id_);
        arm_move_group_->setPlanningTime(joint_planning_time_);
        arm_move_group_->setMaxVelocityScalingFactor(0.20);
        arm_move_group_->setMaxAccelerationScalingFactor(0.20);
    }

    void configure_for_pose_planning()
    {
        if (!arm_move_group_) return;
        arm_move_group_->setPlanningPipelineId(pose_planner_pipeline_);
        arm_move_group_->setPlannerId(pose_planner_id_);
        arm_move_group_->setPlanningTime(pose_planning_time_);
        arm_move_group_->setMaxVelocityScalingFactor(0.15);
        arm_move_group_->setMaxAccelerationScalingFactor(0.15);
    }

    void configure_for_gripper_closing()
    {
        if (!gripper_move_group_) return;
        gripper_move_group_->setMaxVelocityScalingFactor(grasp_close_velocity_);
        gripper_move_group_->setMaxAccelerationScalingFactor(grasp_close_velocity_);
    }

    void configure_for_lifting()
    {
        if (!arm_move_group_) return;
        arm_move_group_->setMaxVelocityScalingFactor(grasp_lift_velocity_);
        arm_move_group_->setMaxAccelerationScalingFactor(grasp_lift_velocity_);
    }

    void ensure_arm_move_group()
    {
        if (arm_move_group_) return;
        auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
        arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, arm_group_name_);
        arm_move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        arm_move_group_->setPlannerId("PTP");
        arm_move_group_->setPlanningTime(10.0);
        arm_move_group_->setMaxVelocityScalingFactor(0.10);
        arm_move_group_->setMaxAccelerationScalingFactor(0.10);
        RCLCPP_INFO(this->get_logger(), "Arm MoveGroupInterface initialized.");
    }

    void ensure_gripper_move_group()
    {
        if (gripper_move_group_) return;
        auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
        gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, gripper_group_name_);
        gripper_move_group_->setPlanningTime(2.0);
        gripper_move_group_->setMaxVelocityScalingFactor(1.0);
        gripper_move_group_->setMaxAccelerationScalingFactor(1.0);
        RCLCPP_INFO(this->get_logger(), "Gripper MoveGroupInterface initialized.");
    }


    geometry_msgs::msg::Pose compute_pre_grasp_pose(const geometry_msgs::msg::Pose& grasp_pose, double offset_distance)
    {
        // SIMPLE APPROACH: Just offset upward in world Z-axis
        // This is more reliable than trying to guess gripper frame orientation
        // Works for top-down grasps (most common in pick-and-place)
        
        // geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose;
        // pre_grasp_pose.position.z += offset_distance;  // Move UP in world frame
        
        // return pre_grasp_pose;
        
        // ALTERNATIVE: Use gripper frame Z-axis if you know your gripper convention
        // Uncomment below if gripper Z-axis points down toward object:
    
        tf2::Quaternion q(grasp_pose.orientation.x, grasp_pose.orientation.y, 
                         grasp_pose.orientation.z, grasp_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        
        // If gripper Z points DOWN (toward object), then negative Z is the retreat direction
        tf2::Vector3 retreat_direction(0, 0, -1);  // Negative Z in gripper = away from object
        tf2::Vector3 retreat_world = m * retreat_direction;
        
        geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose;
        pre_grasp_pose.position.x += retreat_world.x() * offset_distance;
        pre_grasp_pose.position.y += retreat_world.y() * offset_distance;
        pre_grasp_pose.position.z += retreat_world.z() * offset_distance;
        
        return pre_grasp_pose;
        
    }

    geometry_msgs::msg::Pose compute_lift_pose(const geometry_msgs::msg::Pose& grasp_pose, double lift_distance)
    {
        geometry_msgs::msg::Pose lift_pose = grasp_pose;
        lift_pose.position.z += lift_distance;
        return lift_pose;
    }


    // Action callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ExecuteMotion::Goal> goal)
    {
        if (goal->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rejected goal: data array is empty.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Accepted goal. Command type: %.1f", goal->data[0]);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteMotion>)
    {
        RCLCPP_WARN(this->get_logger(), "Cancel requested, but not supported.");
        return rclcpp_action::CancelResponse::REJECT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
        std::thread{
            std::bind(&MotionActionServer::execute, this, std::placeholders::_1),
            goal_handle
        }.detach();
    }

    // EXECUTION LOGIC
    void execute(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
        auto result = std::make_shared<ExecuteMotion::Result>();
        auto feedback = std::make_shared<ExecuteMotion::Feedback>();

        // Serialize goals. MoveIt interfaces are not safe to use concurrently.
        if (!exec_mutex_.try_lock()) {
            result->success = false;
            result->error_code = "BUSY";
            result->error_description = "Server is busy executing another goal.";
            goal_handle->abort(result);
            return;
        }
        std::unique_lock<std::mutex> lock(exec_mutex_, std::adopt_lock);

        const auto &data = goal_handle->get_goal()->data;
        const int command_type = static_cast<int>(data[0]);

        RCLCPP_INFO(this->get_logger(), "Executing motion goal (type=%d)...", command_type);

        // Initial feedback
        feedback->state = "RECEIVED";
        feedback->progress = 0.25f;
        goal_handle->publish_feedback(feedback);

        // Planning feedback
        feedback->state = "PLANNING";
        feedback->progress = 0.50f;
        goal_handle->publish_feedback(feedback);

        MotionOutcome outcome;

        switch (command_type)
        {
        case 0:
            ensure_arm_move_group();
            outcome = plan_and_execute_joint(data, goal_handle);
            break;

        case 1:
            ensure_arm_move_group();
            outcome = plan_and_execute_pose(data, goal_handle);
            break;

        case 2:
            ensure_gripper_move_group();
            outcome = execute_gripper_command(data, goal_handle);
            break;
        case 3:
            ensure_arm_move_group();
            ensure_gripper_move_group();
            outcome = execute_grasp_sequence(data, goal_handle);
            break;
        case 4:
            ensure_arm_move_group();
            ensure_gripper_move_group();
            outcome = execute_place_sequence(data, goal_handle);
            break;

        default:
            outcome.success = false;
            outcome.error_code = "INVALID_COMMAND";
            outcome.error_description = "Unknown command type. Use 0 (joint), 1 (pose), 2 (gripper), 3 (grasp), 4 (place).";
            break;
        }

        // Final feedback
        feedback->state = outcome.success ? "DONE" : "FAILED";
        feedback->progress = 1.0f;
        goal_handle->publish_feedback(feedback);

        result->success = outcome.success;
        result->error_code = outcome.error_code;
        result->error_description = outcome.error_description;

        if (outcome.success) {
            goal_handle->succeed(result);
        } else {
            goal_handle->abort(result);
        }
    }

    // MOVEIT EXECUTION HELPERS
    MotionOutcome plan_and_execute_joint(const std::vector<double>& data,
                                        const std::shared_ptr<GoalHandleExecuteMotion>& goal_handle)
    {
        MotionOutcome out;

        if (data.size() < 2) {
            out.success = false;
            out.error_code = "INVALID_GOAL";
            out.error_description = "Joint command requires at least 1 joint value: [0.0, q1, q2, ...].";
            return out;
        }

        const int n_joints = static_cast<int>(data.size()) - 1;
        if (joint_dof_expected_ > 0 && n_joints != joint_dof_expected_) {
            RCLCPP_WARN(this->get_logger(),
                        "Joint command has %d joints, expected %d (joint_dof_expected). Proceeding anyway.",
                        n_joints, joint_dof_expected_);
        }

        // take all joint values after the flag
        configure_for_joint_planning();
        std::vector<double> joints(data.begin() + 1, data.end());
        arm_move_group_->setJointValueTarget(joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_code = arm_move_group_->plan(plan);
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
            out.success = false;
            out.error_code = "PLAN_FAILED";
            out.error_description = "MoveIt planning failed for joint target.";
            return out;
        }

        // EXECUTING feedback
        auto fb = std::make_shared<ExecuteMotion::Feedback>();
        fb->state = "EXECUTING";
        fb->progress = 0.75f;
        goal_handle->publish_feedback(fb);

        auto exec_code = arm_move_group_->execute(plan);
        if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
            out.success = false;
            out.error_code = "EXEC_FAILED";
            out.error_description = "MoveIt execution failed for joint target.";
            return out;
        }

        out.success = true;
        out.error_code = "SUCCESS";
        out.error_description = "Joint motion completed.";
        return out;
    }

    MotionOutcome plan_and_execute_pose(const std::vector<double>& data,
                                        const std::shared_ptr<GoalHandleExecuteMotion>& goal_handle)
    {
        MotionOutcome out;

        // Expected layout: [1.0, x, y, z, qx, qy, qz, qw]
        if (data.size() != 8) {
            out.success = false;
            out.error_code = "INVALID_GOAL";
            out.error_description = "Pose command must be [1.0, x, y, z, qx, qy, qz, qw].";
            return out;
        }

        geometry_msgs::msg::Pose target;
        target.position.x = data[1];
        target.position.y = data[2];
        target.position.z = data[3];
        target.orientation.x = data[4];
        target.orientation.y = data[5];
        target.orientation.z = data[6];
        target.orientation.w = data[7];

        configure_for_pose_planning();
        
        // CRITICAL FIX: Set start state to current state before planning
        arm_move_group_->setStartStateToCurrentState();
        
        arm_move_group_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode plan_code;
        std::string planner_used = "NONE";

        // Strategy 1: Try PTP first
        RCLCPP_INFO(this->get_logger(), "Attempting pose planning with Pilz/PTP...");
        arm_move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        arm_move_group_->setPlannerId("PTP");
        arm_move_group_->setPlanningTime(pose_planning_time_);
        arm_move_group_->setPoseTarget(target);
        plan_code = arm_move_group_->plan(plan);
        
        if (plan_code == moveit::core::MoveItErrorCode::SUCCESS) {
            planner_used = "Pilz/PTP";
            RCLCPP_INFO(this->get_logger(), "Planning succeeded with Pilz/PTP");
        }
        
        // Strategy 2: If PTP fails, try LIN
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Pilz/PTP failed, trying Pilz/LIN...");
            arm_move_group_->setStartStateToCurrentState();  // Refresh state
            arm_move_group_->setPlannerId("LIN");
            arm_move_group_->setPoseTarget(target);
            plan_code = arm_move_group_->plan(plan);
            
            if (plan_code == moveit::core::MoveItErrorCode::SUCCESS) {
                planner_used = "Pilz/LIN";
                RCLCPP_INFO(this->get_logger(), "Planning succeeded with Pilz/LIN");
            }
        }
        
        // Strategy 3: If both Pilz planners fail, try OMPL fallback
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS && pose_use_ompl_fallback_) {
            RCLCPP_WARN(this->get_logger(), "Both Pilz planners failed, trying OMPL fallback...");
            configure_for_joint_planning();
            arm_move_group_->setStartStateToCurrentState();  // Refresh state for OMPL
            arm_move_group_->setPoseTarget(target);
            plan_code = arm_move_group_->plan(plan);
            
            if (plan_code == moveit::core::MoveItErrorCode::SUCCESS) {
                planner_used = joint_planner_pipeline_ + "/" + joint_planner_id_;
                RCLCPP_INFO(this->get_logger(), "Planning succeeded with OMPL fallback");
            }
        }
        
        // All strategies failed
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
            out.success = false;
            out.error_code = "PLAN_FAILED";
            out.error_description = pose_use_ompl_fallback_ 
                ? "All planning strategies failed (PTP, LIN, OMPL) for pose target."
                : "Both Pilz planners (PTP, LIN) failed for pose target.";
            return out;
        }


        // EXECUTING feedback
        auto fb = std::make_shared<ExecuteMotion::Feedback>();
        fb->state = "EXECUTING";
        fb->progress = 0.75f;
        goal_handle->publish_feedback(fb);

        auto exec_code = arm_move_group_->execute(plan);
        if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
            out.success = false;
            out.error_code = "EXEC_FAILED";
            out.error_description = "MoveIt execution failed for pose target.";
            return out;
        }

        out.success = true;
        out.error_code = "SUCCESS";
        out.error_description = "Pose motion completed.";
        return out;
    }

    MotionOutcome execute_gripper_command(const std::vector<double> &data,
                                        const std::shared_ptr<GoalHandleExecuteMotion>& goal_handle)
    {
        MotionOutcome out;

        // Expected: [2.0, value]
        if (data.size() < 2) {
            out.success = false;
            out.error_code = "INVALID_GOAL";
            out.error_description = "Gripper command requires 1 value: [2.0, value].";
            return out;
        }

        // value in [0,1] typically (0=open, 1=close). We clamp to be safe.
        const double value_raw = data[1];
        const double value = std::clamp(value_raw, 0.0, 1.0);

        RCLCPP_INFO(this->get_logger(), "Gripper command: raw=%.3f clamped=%.3f", value_raw, value);

        // EXECUTING feedback
        auto fb = std::make_shared<ExecuteMotion::Feedback>();
        fb->state = "EXECUTING";
        fb->progress = 0.75f;
        goal_handle->publish_feedback(fb);

        // Preferred: named targets (Open/Close)
        const bool close = (value >= 0.5);
        const std::string named_target = close ? gripper_close_target_ : gripper_open_target_;

        bool target_set = gripper_move_group_->setNamedTarget(named_target);

        // Fallback: joint interpolation if named target doesn't exist
        if (!target_set) {
            if (gripper_joint_name_.empty()) {
                out.success = false;
                out.error_code = "GRIPPER_TARGET_MISSING";
                out.error_description =
                    "Gripper named target not found and gripper_joint_name fallback is not configured.";
                RCLCPP_ERROR(this->get_logger(),
                            "Named target '%s' not found for group '%s', and fallback disabled.",
                            named_target.c_str(), gripper_group_name_.c_str());
                return out;
            }

            const double joint_val = gripper_open_value_ + (gripper_close_value_ - gripper_open_value_) * value;

            std::map<std::string, double> joint_target;
            joint_target[gripper_joint_name_] = joint_val;

            RCLCPP_WARN(this->get_logger(),
                        "Named target '%s' not found. Using fallback joint '%s' = %.6f.",
                        named_target.c_str(), gripper_joint_name_.c_str(), joint_val);

            gripper_move_group_->setJointValueTarget(joint_target);
        } else {
            RCLCPP_INFO(this->get_logger(), "Using gripper named target '%s'.", named_target.c_str());
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_code = gripper_move_group_->plan(plan);
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
            out.success = false;
            out.error_code = "PLAN_FAILED";
            out.error_description = "MoveIt planning failed for gripper command.";
            return out;
        }

        auto exec_code = gripper_move_group_->execute(plan);
        if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
            out.success = false;
            out.error_code = "EXEC_FAILED";
            out.error_description = "MoveIt execution failed for gripper command.";
            return out;
        }

        out.success = true;
        out.error_code = "SUCCESS";
        out.error_description = "Gripper command completed.";
        return out;
    }

    // NEW: Command type 3 - Full grasp sequence
    // Format: [3.0, x, y, z, qx, qy, qz, qw, (optional)pre_grasp_offset, (optional)lift_height]
    MotionOutcome execute_grasp_sequence(const std::vector<double>& data, const std::shared_ptr<GoalHandleExecuteMotion>& goal_handle)
    {
        MotionOutcome out;
        if (data.size() < 8) {
            out.error_code = "INVALID_GOAL";
            out.error_description = "Grasp needs [3.0, x, y, z, qx, qy, qz, qw, (opt)offset, (opt)lift].";
            return out;
        }

        geometry_msgs::msg::Pose grasp_pose;
        grasp_pose.position.x = data[1] + 0.0225; 
        grasp_pose.position.y = data[2]; 
        grasp_pose.position.z = data[3] + 0.05;
        grasp_pose.orientation.x = data[4]; 
        grasp_pose.orientation.y = data[5];
        grasp_pose.orientation.z = data[6]; 
        grasp_pose.orientation.w = data[7];

        double pre_grasp_offset = (data.size() >= 9) ? data[8] : default_pre_grasp_distance_;
        double lift_height = (data.size() >= 10) ? data[9] : default_lift_distance_;

        auto fb = std::make_shared<ExecuteMotion::Feedback>();

        RCLCPP_INFO(this->get_logger(), "Waiting before approach...");
        rclcpp::sleep_for(std::chrono::seconds(1));  // 0.5 seconds

        // Step 1: Open gripper
        RCLCPP_INFO(this->get_logger(), "Grasp Step 1/5: Opening gripper");
        fb->state = "OPENING_GRIPPER"; 
        fb->progress = 0.20f; 
        goal_handle->publish_feedback(fb);
        
        if (!gripper_move_group_->setNamedTarget(gripper_open_target_)) {
            out.error_code = "GRIPPER_OPEN_FAILED"; 
            return out;
        }
        
        moveit::planning_interface::MoveGroupInterface::Plan gplan;
        if (gripper_move_group_->plan(gplan) != moveit::core::MoveItErrorCode::SUCCESS ||
            gripper_move_group_->execute(gplan) != moveit::core::MoveItErrorCode::SUCCESS) {
            out.error_code = "GRIPPER_OPEN_EXEC_FAILED"; 
            return out;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting before approach...");
        rclcpp::sleep_for(std::chrono::seconds(1));  // 0.5 seconds

        
        // Step 2: Move to pre-grasp
        RCLCPP_INFO(this->get_logger(), "Grasp Step 2/5: Moving to pre-grasp");
        fb->state = "MOVING_TO_PRE_GRASP"; 
        fb->progress = 0.40f; 
        goal_handle->publish_feedback(fb);
        
        geometry_msgs::msg::Pose pre_grasp = compute_pre_grasp_pose(grasp_pose, pre_grasp_offset);
        configure_for_pose_planning();

        log_pose("GRASP POSE: ", grasp_pose);
        log_pose("PRE GRASP: ", pre_grasp);
        
        // CRITICAL FIX: Set start state to current state before planning
        arm_move_group_->setStartStateToCurrentState();
        
        arm_move_group_->setPoseTarget(pre_grasp);
        moveit::planning_interface::MoveGroupInterface::Plan pplan;
        auto pcode = arm_move_group_->plan(pplan);
        
        if (pcode != moveit::core::MoveItErrorCode::SUCCESS) {
            configure_for_joint_planning();
            arm_move_group_->setStartStateToCurrentState();  // Set again for fallback
            arm_move_group_->setPoseTarget(pre_grasp);
            pcode = arm_move_group_->plan(pplan);
            if (pcode != moveit::core::MoveItErrorCode::SUCCESS) {
                out.error_code = "PRE_GRASP_PLAN_FAILED"; 
                return out;
            }
        }
        
        if (arm_move_group_->execute(pplan) != moveit::core::MoveItErrorCode::SUCCESS) {
            out.error_code = "PRE_GRASP_EXEC_FAILED"; 
            return out;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting before approach...");
        rclcpp::sleep_for(std::chrono::seconds(1));  // 0.5 seconds

        // Step 3: Approach using simple pose planning
        RCLCPP_INFO(this->get_logger(), "Grasp Step 3/5: Approaching grasp pose");
        fb->state = "APPROACHING_GRASP"; 
        fb->progress = 0.60f; 
        goal_handle->publish_feedback(fb);

        log_pose("APPROACH GRASP POSE: ", grasp_pose);

        // CRITICAL FIX: Set start state to current state before planning
        arm_move_group_->setStartStateToCurrentState();

        // Use Pilz LIN planner for straight-line motion (faster than cartesian)
        arm_move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        arm_move_group_->setPlannerId("LIN");
        arm_move_group_->setPlanningTime(5.0);
        arm_move_group_->setMaxVelocityScalingFactor(0.05);  // Slow and controlled
        arm_move_group_->setMaxAccelerationScalingFactor(0.05);
        arm_move_group_->setPoseTarget(grasp_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        auto approach_code = arm_move_group_->plan(approach_plan);
        
        // Fallback to OMPL if LIN fails
        if (approach_code != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "LIN planner failed, trying OMPL fallback");
            configure_for_joint_planning();
            arm_move_group_->setStartStateToCurrentState();  // Set again for OMPL
            arm_move_group_->setMaxVelocityScalingFactor(0.05);
            arm_move_group_->setMaxAccelerationScalingFactor(0.05);
            arm_move_group_->setPoseTarget(grasp_pose);
            approach_code = arm_move_group_->plan(approach_plan);
            
            if (approach_code != moveit::core::MoveItErrorCode::SUCCESS) {
                out.error_code = "GRASP_APPROACH_PLAN_FAILED"; 
                return out;
            }
        }
        
        if (arm_move_group_->execute(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            out.error_code = "GRASP_APPROACH_EXEC_FAILED"; 
            return out;
        }
 
        RCLCPP_INFO(this->get_logger(), "Waiting before approach...");
        rclcpp::sleep_for(std::chrono::seconds(1));  // 0.5 seconds
        // Step 4: Close gripper
        RCLCPP_INFO(this->get_logger(), "Grasp Step 4/5: Closing gripper");
        fb->state = "CLOSING_GRIPPER"; 
        fb->progress = 0.80f; 
        goal_handle->publish_feedback(fb);
        
        configure_for_gripper_closing();
        
        if (!gripper_move_group_->setNamedTarget(gripper_close_target_)) {
            out.error_code = "GRIPPER_CLOSE_FAILED"; 
            return out;
        }
        
        moveit::planning_interface::MoveGroupInterface::Plan cplan;
        if (gripper_move_group_->plan(cplan) != moveit::core::MoveItErrorCode::SUCCESS ||
            gripper_move_group_->execute(cplan) != moveit::core::MoveItErrorCode::SUCCESS) {
            out.error_code = "GRIPPER_CLOSE_EXEC_FAILED"; 
            return out;
        }
        
        RCLCPP_INFO(this->get_logger(), "Waiting before approach...");
        rclcpp::sleep_for(std::chrono::seconds(1));  // 0.5 seconds
        // Step 5: Lift using simple pose planning (SIMPLIFIED - NO CARTESIAN)
        RCLCPP_INFO(this->get_logger(), "Grasp Step 5/5: Lifting object");
        fb->state = "LIFTING_OBJECT"; 
        fb->progress = 0.90f; 
        goal_handle->publish_feedback(fb);
        
        geometry_msgs::msg::Pose lift_pose = compute_lift_pose(grasp_pose, lift_height);
        
        // CRITICAL FIX: Set start state to current state before planning
        arm_move_group_->setStartStateToCurrentState();
        
        // Use LIN planner for straight upward motion
        arm_move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        arm_move_group_->setPlannerId("LIN");
        arm_move_group_->setPlanningTime(5.0);
        arm_move_group_->setMaxVelocityScalingFactor(0.05);
        arm_move_group_->setMaxAccelerationScalingFactor(0.05);
        arm_move_group_->setPoseTarget(lift_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
        auto lift_code = arm_move_group_->plan(lift_plan);
        
        // Fallback to OMPL if LIN fails
        if (lift_code != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "LIN planner failed for lift, trying OMPL fallback");
            configure_for_joint_planning();
            arm_move_group_->setStartStateToCurrentState();  // Set again for fallback
            arm_move_group_->setMaxVelocityScalingFactor(0.05);
            arm_move_group_->setMaxAccelerationScalingFactor(0.05);
            arm_move_group_->setPoseTarget(lift_pose);
            lift_code = arm_move_group_->plan(lift_plan);
            
            if (lift_code != moveit::core::MoveItErrorCode::SUCCESS) {
                out.error_code = "LIFT_PLAN_FAILED"; 
                return out;
            }
        }
        
        if (arm_move_group_->execute(lift_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            out.error_code = "LIFT_EXEC_FAILED"; 
            return out;
        }
        
        out.success = true;
        out.error_code = "SUCCESS";
        out.error_description = "Grasp sequence completed.";
        return out;

    }
    

    // NEW: Command type 4 - Place sequence
    // Format: [4.0, x, y, z, qx, qy, qz, qw, (optional)retreat_distance]
    MotionOutcome execute_place_sequence(const std::vector<double>& data, const std::shared_ptr<GoalHandleExecuteMotion>& goal_handle)
    {
        MotionOutcome out;
        if (data.size() < 8) {
            out.error_code = "INVALID_GOAL";
            out.error_description = "Place needs [4.0, x, y, z, qx, qy, qz, qw, (opt)retreat].";
            return out;
        }

        geometry_msgs::msg::Pose place_pose;
        place_pose.position.x = data[1]; 
        place_pose.position.y = data[2]; 
        place_pose.position.z = data[3];
        place_pose.orientation.x = data[4]; 
        place_pose.orientation.y = data[5];
        place_pose.orientation.z = data[6]; 
        place_pose.orientation.w = data[7];

        double retreat_dist = (data.size() >= 9) ? data[8] : default_lift_distance_;
        auto fb = std::make_shared<ExecuteMotion::Feedback>();

        // Step 1: Move to place pose
        RCLCPP_INFO(this->get_logger(), "Place Step 1/3: Moving to place pose");
        fb->state = "MOVING_TO_PLACE"; 
        fb->progress = 0.33f; 
        goal_handle->publish_feedback(fb);
        
        configure_for_pose_planning();
        
        // CRITICAL FIX: Set start state to current state before planning
        arm_move_group_->setStartStateToCurrentState();
        
        arm_move_group_->setPoseTarget(place_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan pplan;
        auto pcode = arm_move_group_->plan(pplan);
        
        if (pcode != moveit::core::MoveItErrorCode::SUCCESS) {
            configure_for_joint_planning();
            arm_move_group_->setStartStateToCurrentState();  // Refresh for fallback
            arm_move_group_->setPoseTarget(place_pose);
            pcode = arm_move_group_->plan(pplan);
            if (pcode != moveit::core::MoveItErrorCode::SUCCESS) {
                out.error_code = "PLACE_PLAN_FAILED"; 
                return out;
            }
        }
        
        if (arm_move_group_->execute(pplan) != moveit::core::MoveItErrorCode::SUCCESS) {
            out.error_code = "PLACE_EXEC_FAILED"; 
            return out;
        }

        // Step 2: Open gripper
        RCLCPP_INFO(this->get_logger(), "Place Step 2/3: Opening gripper");
        fb->state = "RELEASING_OBJECT"; 
        fb->progress = 0.66f; 
        goal_handle->publish_feedback(fb);
        
        if (!gripper_move_group_->setNamedTarget(gripper_open_target_)) {
            out.error_code = "GRIPPER_OPEN_FAILED"; 
            return out;
        }
        
        moveit::planning_interface::MoveGroupInterface::Plan oplan;
        if (gripper_move_group_->plan(oplan) != moveit::core::MoveItErrorCode::SUCCESS ||
            gripper_move_group_->execute(oplan) != moveit::core::MoveItErrorCode::SUCCESS) {
            out.error_code = "GRIPPER_OPEN_EXEC_FAILED"; 
            return out;
        }

        // Step 3: Retreat using simple pose planning (SIMPLIFIED - NO CARTESIAN)
        RCLCPP_INFO(this->get_logger(), "Place Step 3/3: Retreating");
        fb->state = "RETREATING"; 
        fb->progress = 0.90f; 
        goal_handle->publish_feedback(fb);
        
        geometry_msgs::msg::Pose retreat_pose = place_pose;
        retreat_pose.position.z += retreat_dist;
        
        // CRITICAL FIX: Set start state to current state before planning
        arm_move_group_->setStartStateToCurrentState();
        
        // Use LIN planner for straight retreat motion
        arm_move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        arm_move_group_->setPlannerId("LIN");
        arm_move_group_->setPlanningTime(5.0);
        arm_move_group_->setMaxVelocityScalingFactor(0.05);
        arm_move_group_->setMaxAccelerationScalingFactor(0.05);
        arm_move_group_->setPoseTarget(retreat_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
        auto retreat_code = arm_move_group_->plan(retreat_plan);
        
        // Fallback to OMPL if LIN fails
        if (retreat_code != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "LIN planner failed for retreat, trying OMPL fallback");
            configure_for_joint_planning();
            arm_move_group_->setStartStateToCurrentState();  // Refresh for fallback
            arm_move_group_->setMaxVelocityScalingFactor(0.05);
            arm_move_group_->setMaxAccelerationScalingFactor(0.05);
            arm_move_group_->setPoseTarget(retreat_pose);
            retreat_code = arm_move_group_->plan(retreat_plan);
            
            if (retreat_code != moveit::core::MoveItErrorCode::SUCCESS) {
                out.error_code = "RETREAT_PLAN_FAILED"; 
                return out;
            }
        }
        
        if (arm_move_group_->execute(retreat_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            out.error_code = "RETREAT_EXEC_FAILED"; 
            return out;
        }

        out.success = true;
        out.error_code = "SUCCESS";
        out.error_description = "Place sequence completed.";
        return out;
    }



    // ------------------------------
    // Members
    // ------------------------------
    std::mutex exec_mutex_;
    std::string arm_group_name_, gripper_group_name_;
    std::string gripper_open_target_, gripper_close_target_, gripper_joint_name_;
    double gripper_open_value_{0.0}, gripper_close_value_{1.0};
    double default_pre_grasp_distance_{0.15}, default_lift_distance_{0.15};
    double grasp_approach_velocity_{0.05}, grasp_close_velocity_{0.3}, grasp_lift_velocity_{0.05};
    std::string joint_planner_pipeline_{"ompl"}, joint_planner_id_{"RRTConnect"};
    double joint_planning_time_{5.0};
    std::string pose_planner_pipeline_{"pilz_industrial_motion_planner"}, pose_planner_id_{"LIN"};
    double pose_planning_time_{3.0};
    bool pose_use_ompl_fallback_{true};
    int joint_dof_expected_{-1};
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
    rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotionActionServer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}