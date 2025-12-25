#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

        // --------------------------
        // Parameters (override via YAML / launch)
        // --------------------------
        arm_group_name_ = this->declare_parameter<std::string>("arm_group_name", "manipulator");
        gripper_group_name_ = this->declare_parameter<std::string>("gripper_group_name", "gripper");
        gripper_open_target_ = this->declare_parameter<std::string>("gripper_open_target", "Open");
        gripper_close_target_ = this->declare_parameter<std::string>("gripper_close_target", "Close");

        // Optional fallback: control gripper via a joint target (if named targets aren't defined).
        // If gripper_joint_name is empty, fallback is disabled.
        gripper_joint_name_ = this->declare_parameter<std::string>("gripper_joint_name", "");
        gripper_open_value_ = this->declare_parameter<double>("gripper_open_value", 0.0);
        gripper_close_value_ = this->declare_parameter<double>("gripper_close_value", 1.0);

        // Optional sanity check for joint commands.
        // Set to -1 to disable. For a 7-DOF arm, set to 7.
        joint_dof_expected_ = this->declare_parameter<int>("joint_dof_expected", -1);

        // DO NOT USE shared_from_this() HERE → causes std::bad_weak_ptr
        // We delay initializing MoveGroupInterface until first use.

        action_server_ = rclcpp_action::create_server<ExecuteMotion>(
            this,
            "execute_motion",
            std::bind(&MotionActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MotionActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }

private:
    struct MotionOutcome
    {
        bool success{false};
        std::string error_code{"FAILURE"};
        std::string error_description{"Motion failed."};
    };

    // --------------------------
    // Lazy init: Arm MoveGroup
    // --------------------------
    void ensure_arm_move_group()
    {
        if (arm_move_group_) {
            return;
        }

        // SAFE method: aliasing shared_ptr to this node
        auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

        arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_ptr, arm_group_name_
        );

        arm_move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        arm_move_group_->setPlannerId("PTP");
        arm_move_group_->setPlanningTime(5.0);
        arm_move_group_->setMaxVelocityScalingFactor(0.10);
        arm_move_group_->setMaxAccelerationScalingFactor(0.10);

        RCLCPP_INFO(this->get_logger(), "Arm MoveGroupInterface initialized (group: %s).", arm_group_name_.c_str());
    }

    // --------------------------
    // Lazy init: Gripper MoveGroup
    // --------------------------
    void ensure_gripper_move_group()
    {
        if (gripper_move_group_) {
            return;
        }

        auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

        gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_ptr, gripper_group_name_
        );

        gripper_move_group_->setPlanningTime(2.0);
        gripper_move_group_->setMaxVelocityScalingFactor(1.0);
        gripper_move_group_->setMaxAccelerationScalingFactor(1.0);

        RCLCPP_INFO(this->get_logger(), "Gripper MoveGroupInterface initialized (group: %s).", gripper_group_name_.c_str());
    }

    // --------------------------
    // Action callbacks
    // --------------------------
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

    // ------------------------------
    // EXECUTION LOGIC
    // ------------------------------
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

        default:
            outcome.success = false;
            outcome.error_code = "INVALID_COMMAND";
            outcome.error_description = "Unknown command type. Use 0 (joint), 1 (pose), 2 (gripper).";
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

    // ------------------------------
    // MOVEIT EXECUTION HELPERS
    // ------------------------------
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

        // IMPORTANT FIX: take all joint values after the flag
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

        arm_move_group_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_code = arm_move_group_->plan(plan);
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
            out.success = false;
            out.error_code = "PLAN_FAILED";
            out.error_description = "MoveIt planning failed for pose target.";
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

    // ------------------------------
    // Members
    // ------------------------------
    std::mutex exec_mutex_;

    std::string arm_group_name_;
    std::string gripper_group_name_;
    std::string gripper_open_target_;
    std::string gripper_close_target_;

    std::string gripper_joint_name_;
    double gripper_open_value_{0.0};
    double gripper_close_value_{1.0};

    int joint_dof_expected_{-1};

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;

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
