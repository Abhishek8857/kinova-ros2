# include <rclcpp/rclcpp.hpp>
# include <moveit/move_group_interface/move_group_interface.h>
# include <trajectory_msgs/msg/joint_trajectory_point.hpp>
# include <moveit/planning_interface/planning_interface.h>
#include <control_msgs/action/gripper_command.hpp>
# include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
# include <yaml-cpp/yaml.h>
# include <fstream>
# include <string>
# include <filesystem>
# include <algorithm>
# include <vector>
# include <thread>

namespace fs = std::filesystem;

moveit_msgs::msg::RobotTrajectory loadTrajectoryFromFile(const std::string &filepath)
{
    YAML::Node root = YAML::LoadFile(filepath);

    moveit_msgs::msg::RobotTrajectory trajectory_msg;

    auto joint_names_node = root["joint_order"];
    auto points_node = root["points"];

    for (const auto &name : joint_names_node)
        trajectory_msg.joint_trajectory.joint_names.push_back(name.as<std::string>());

    double t0 = points_node[0]["time_from_start"].as<double>();

    for (const auto &pt : points_node)
    {
        trajectory_msgs::msg::JointTrajectoryPoint point;

        for (const auto &val : pt["positions"])
            point.positions.push_back(val.as<double>());

        if (pt["velocities"])
        {
            for (const auto &val : pt["velocities"])
                point.velocities.push_back(val.as<double>());
        }

        // fallback if velocities are empty
        if (point.velocities.empty())
            point.velocities.resize(point.positions.size(), 0.0);

        if (pt["time_from_start"])
        {
            double rel_time = pt["time_from_start"].as<double>() - t0;
            point.time_from_start = rclcpp::Duration::from_seconds(rel_time);
        }

        trajectory_msg.joint_trajectory.points.push_back(point);
    }

    return trajectory_msg;
}

void splitTrajectoryByGroup(
    const moveit_msgs::msg::RobotTrajectory& full_trajectory,
    const std::vector<std::string>& manipulator_joints,
    const std::vector<std::string>& gripper_joints,
    moveit_msgs::msg::RobotTrajectory& manipulator_trajectory,
    moveit_msgs::msg::RobotTrajectory& gripper_trajectory)
{
    const auto& jt = full_trajectory.joint_trajectory;

    std::map<std::string, size_t> name_to_index;
    for (size_t i = 0; i < jt.joint_names.size(); ++i)
        name_to_index[jt.joint_names[i]] = i;

    // Set joint names
    manipulator_trajectory.joint_trajectory.joint_names = manipulator_joints;
    gripper_trajectory.joint_trajectory.joint_names = gripper_joints;

    for (const auto& pt : jt.points)
    {
        trajectory_msgs::msg::JointTrajectoryPoint mpt, gpt;
        mpt.time_from_start = pt.time_from_start;
        gpt.time_from_start = pt.time_from_start;

        for (const auto& name : manipulator_joints)
        {
            size_t idx = name_to_index[name];
            mpt.positions.push_back(pt.positions[idx]);
            if (!pt.velocities.empty()) mpt.velocities.push_back(pt.velocities[idx]);
        }

        for (const auto& name : gripper_joints)
        {
            size_t idx = name_to_index[name];
            gpt.positions.push_back(pt.positions[idx]);
            if (!pt.velocities.empty()) gpt.velocities.push_back(pt.velocities[idx]);
        }

        manipulator_trajectory.joint_trajectory.points.push_back(mpt);
        gripper_trajectory.joint_trajectory.points.push_back(gpt);
    }
}

void saveTrajectoryToFile(const moveit_msgs::msg::RobotTrajectory& traj, const std::string& filepath)
{

    // Remove the file if it already exists
    if (fs::exists(filepath))
    {
        fs::remove(filepath);
    }

    YAML::Node root;
    const auto& jt = traj.joint_trajectory;

    root["joint_order"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& name : jt.joint_names)
        root["joint_order"].push_back(name);

    root["points"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& pt : jt.points)
    {
        YAML::Node point;
        point["positions"] = YAML::Node(YAML::NodeType::Sequence);
        for (double p : pt.positions)
            point["positions"].push_back(p);

        point["velocities"] = YAML::Node(YAML::NodeType::Sequence);
        for (double v : pt.velocities)
            point["velocities"].push_back(v);

        point["time_from_start"] = rclcpp::Duration(pt.time_from_start).seconds();
        root["points"].push_back(point);
    }

    std::ofstream fout(filepath);
    fout << root;
    fout.close();
}


void printTrajectory(const moveit_msgs::msg::RobotTrajectory& traj)
{
    const auto& jt = traj.joint_trajectory;
    RCLCPP_INFO(rclcpp::get_logger("trajectory_printer"), "Joint names:");
    for (const auto& name : jt.joint_names)
        RCLCPP_INFO(rclcpp::get_logger("trajectory_printer"), "  %s", name.c_str());

    RCLCPP_INFO(rclcpp::get_logger("trajectory_printer"), "Trajectory points: %zu", jt.points.size());
    for (size_t i = 0; i < jt.points.size(); ++i)
    {
        const auto& pt = jt.points[i];
        std::ostringstream pos, vel;
        for (size_t j = 0; j < pt.positions.size(); ++j)
        {
            pos << pt.positions[j] << (j < pt.positions.size() - 1 ? ", " : "");
        }
        for (size_t j = 0; j < pt.velocities.size(); ++j)
        {
            vel << pt.velocities[j] << (j < pt.velocities.size() - 1 ? ", " : "");
        }

        RCLCPP_INFO(rclcpp::get_logger("trajectory_printer"),
                    "Point %zu:\n  positions: [%s]\n  velocities: [%s]\n  time_from_start: %.3f",
                    i, pos.str().c_str(), vel.str().c_str(), rclcpp::Duration(pt.time_from_start).seconds()
);
    }
}


class TrajectoryReplayAllNode : public rclcpp::Node
{
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    TrajectoryReplayAllNode()
        : Node("trajectory_replay_all_node")
    {
        gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "/robotiq_gripper_controller/gripper_cmd");

        if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Gripper action server not available.");
        }
    }

    void run()
    {
        manipulator_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");

        const std::string folder_path = "/kinova-ros2/trajectories/";
        std::vector<fs::directory_entry> yaml_files;

        for (const auto& entry : fs::directory_iterator(folder_path))
        {
            if (entry.path().extension() == ".yaml")
            {
                std::string filename = entry.path().filename().string();
                if (filename.find("debug_") == std::string::npos)  // Skip debug YAMLs
                {
                    yaml_files.push_back(entry);
                }
            }
        }

        std::sort(
            yaml_files.begin(),
            yaml_files.end(),
            [](const fs::directory_entry& a, const fs::directory_entry& b)
            {
                return a.last_write_time() < b.last_write_time();
            });

        if (yaml_files.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No YAML files found in directory: %s", folder_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Found %zu trajectory files. Starting replay...", yaml_files.size());

        for (const auto& file : yaml_files)
        {
            std::string filepath = file.path().string();
            RCLCPP_INFO(this->get_logger(), "Loading %s", filepath.c_str());

            try
            {
                auto full_trajectory = loadTrajectoryFromFile(filepath);
                // printTrajectory(full_trajectory);

                moveit_msgs::msg::RobotTrajectory manip_traj, grip_traj;
                splitTrajectoryByGroup(
                    full_trajectory,
                    {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"},
                    {"robotiq_85_left_knuckle_joint"},
                    manip_traj,
                    grip_traj);

                // Save to YAML
                saveTrajectoryToFile(manip_traj, "/kinova-ros2/trajectories/debug_manip.yaml");
                saveTrajectoryToFile(grip_traj, "/kinova-ros2/trajectories/debug_gripper.yaml");

                moveit::planning_interface::MoveGroupInterface::Plan manip_plan, grip_plan;
                manip_plan.trajectory_ = manip_traj;
                grip_plan.trajectory_ = grip_traj;
                bool manip_success = false, gripper_success = false;

                std::thread manip_thread([&]() {
                    RCLCPP_INFO(this->get_logger(), "Executing manipulator...");
                    manip_success = (manipulator_group_->execute(manip_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                });

                std::thread gripper_thread([&]() {
                    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                        RCLCPP_ERROR(this->get_logger(), "Gripper action server not available.");
                        gripper_success = false;
                        return;
                    }

                    double gripper_pos = 0.0;
                    if (!grip_traj.joint_trajectory.points.empty()) {
                        gripper_pos = grip_traj.joint_trajectory.points.back().positions[0];
                    }
                    
                    if (gripper_pos < 0.1)
                    {
                        RCLCPP_INFO(this->get_logger(), "Gripper command interpreted as: OPEN");
                    }
                    else if (gripper_pos > 0.6)
                    {
                        RCLCPP_INFO(this->get_logger(), "Gripper command interpreted as: CLOSE");
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Gripper command position (%.3f) is ambiguous.", gripper_pos);
                    }

                    auto goal_msg = GripperCommand::Goal();
                    goal_msg.command.position = gripper_pos;
                    goal_msg.command.max_effort = 0.0;

                    RCLCPP_INFO(this->get_logger(), "Sending gripper goal: position=%.3f", gripper_pos);

                    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
                    send_goal_options.result_callback = [this](const GoalHandleGripperCommand::WrappedResult &result) {
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                        {
                            RCLCPP_INFO(this->get_logger(), "Gripper action succeeded");
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "Gripper action failed");
                        }
                    };

                    auto goal_handle_future = gripper_action_client_->async_send_goal(goal_msg, send_goal_options);

                    // Wait for result (optional)
                    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
                        rclcpp::FutureReturnCode::SUCCESS)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to send gripper goal");
                        gripper_success = false;
                        return;
                    }

                    // Optionally wait for result here (blocking)
                    auto goal_handle = goal_handle_future.get();
                    if (!goal_handle)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                        gripper_success = false;
                        return;
                    }

                    auto result_future = gripper_action_client_->async_get_result(goal_handle);
                    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
                        rclcpp::FutureReturnCode::SUCCESS)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to get gripper result");
                        gripper_success = false;
                        return;
                    }

                    auto result = result_future.get();
                    gripper_success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
                });
                    
                manip_thread.join();
                gripper_thread.join();

                if (!manip_success || !gripper_success)
                {
                    RCLCPP_ERROR(this->get_logger(), "Execution failed for: %s", filepath.c_str());
                }

                rclcpp::sleep_for(std::chrono::seconds(1));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error loading/executing file %s: %s", filepath.c_str(), e.what());
            }
        }


        RCLCPP_INFO(this->get_logger(), "Finished executing all saved trajectories");
     }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> manipulator_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryReplayAllNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}