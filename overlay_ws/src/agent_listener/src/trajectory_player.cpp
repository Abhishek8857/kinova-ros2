# include <rclcpp/rclcpp.hpp>
# include <moveit/move_group_interface/move_group_interface.h>
# include <moveit/planning_interface/planning_interface.h>
# include <moveit_msgs/msg/robot_trajectory.hpp>
# include <trajectory_msgs/msg/joint_trajectory_point.hpp>

# include <yaml-cpp/yaml.h>
# include <fstream>
# include <string>
# include <filesystem>
# include <algorithm>
# include <vector>

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
    TrajectoryReplayAllNode()
        : Node("trajectory_replay_all_node")
    {
        // Constructor only sets up the node
    }

    void run()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");

        const std::string folder_path = "/kinova-ros2/trajectories/";
        std::vector<fs::directory_entry> yaml_files;

        for (const auto& entry : fs::directory_iterator(folder_path))
        {
            if (entry.path().extension() == ".yaml")
            {
                yaml_files.push_back(entry);
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
                auto trajectory = loadTrajectoryFromFile(filepath);
                printTrajectory(trajectory);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;

                RCLCPP_INFO(this->get_logger(), "Executing...");
                bool success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (!success)
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
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryReplayAllNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
