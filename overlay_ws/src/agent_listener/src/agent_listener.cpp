# include <memory>
# include <rclcpp/rclcpp.hpp>
# include <geometry_msgs/msg/point.hpp>
# include <moveit/move_group_interface/move_group_interface.h>
# include <std_msgs/msg/float64_multi_array.hpp>
# include <std_msgs/msg/bool.hpp>
# include <thread>
# include <chrono>
# include <yaml-cpp/yaml.h>
# include <fstream>
# include <iomanip>
# include <filesystem>


using std::placeholders::_1;

class AgentSubscriber : public rclcpp::Node
{
  public:
    // Initialise the Node
    AgentSubscriber () : Node ("agent_subscriber")
    {
      // Subscribe to recieve Coordinates
      RCLCPP_INFO(this->get_logger(), "Agent subscriber node initialised. Waiting for coordinates...");
      subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "published_coordinates", 10, std::bind(&AgentSubscriber::agent_callback, this, _1));
    }

  private:  
    void agent_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
    {
      // DEBUG: Print out the recieved Coordinates
      RCLCPP_INFO(this->get_logger(), "Recieved Cartesian Coordinates..");
      for (size_t i {0}; i < msg->data.size(); i++)
      {
        RCLCPP_INFO(this->get_logger(), " -[%zu]: %f", i, msg->data[i]);
      }

      // Lazy initialization of MoveGroupInterface
      if (!move_group_manipulator && !move_group_gripper)
      {
        move_group_manipulator = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");
        move_group_gripper = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
      } 

      // Check if we need to execute Joint Target or Pose Target
      if (msg->data[0] == 0.0)
      {
        // Parse the Coordinates
        std::map<std::string, double> joint_values = 
        {
          {"joint_1", msg->data[1]},
          {"joint_2", msg->data[2]},
          {"joint_3", msg->data[3]},
          {"joint_4", msg->data[4]},
          {"joint_5", msg->data[5]},
          {"joint_6", msg->data[6]},
          {"joint_7", msg->data[7]}
        };

        // Plan and execute
        RCLCPP_INFO(this->get_logger(), "Planning the target pose ...");
        move_group_manipulator->setJointValueTarget(joint_values);

        // Verify if the motion was successful
        // bool success = (move_group_manipulator->move() == moveit::core::MoveItErrorCode::SUCCESS);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_manipulator->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
          move_group_manipulator->execute(plan);
          std::string file_name = "trajectory_" + getTimestamp() + ".yaml";
          saveTrajectoryToFile(plan.trajectory_, file_name); 
        }
        succeed(success);
      }
      else if (msg->data[0] == 1.0)
      {
        // Parse the Coordinates
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = msg->data[1];
        target_pose.position.y = msg->data[2];
        target_pose.position.z = msg->data[3];
  
        // Parse the orientation
        target_pose.orientation.w = msg->data[4];
        target_pose.orientation.x = msg->data[5];
        target_pose.orientation.y = msg->data[6];
        target_pose.orientation.z = msg->data[7];

        // Plan and execute
        RCLCPP_INFO(this->get_logger(), "Planning the target pose ...");
        move_group_manipulator->setPoseTarget(target_pose);

        // Verify if the motion was successful
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group_manipulator->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
          move_group_manipulator->execute(plan);     
          std::string file_name = "trajectory_" + getTimestamp() + ".yaml";
          saveTrajectoryToFile(plan.trajectory_, file_name);    
        }
        succeed(success);
      }
      else if (msg->data[0] == 2.0)
      {
        // Parse the coordinates
        std::map<std::string, double> gripper_values = 
        {
          {"robotiq_85_left_knuckle_joint", msg->data[1]},
          {"robotiq_85_right_knuckle_joint", msg->data[2]}
        };

        // Planning the Gripper motion
        RCLCPP_INFO(this->get_logger(), "Planning the Gripper motion");
        move_group_gripper->setJointValueTarget(gripper_values);

        // Verify if the motion was successful
        // bool success = (move_group_gripper->move() == moveit::core::MoveItErrorCode::SUCCESS);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group_gripper->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
          move_group_gripper->execute(plan);
          std::string file_name = "trajectory_" + getTimestamp() + ".yaml";
          saveTrajectoryToFile(plan.trajectory_, file_name);
        }

        succeed(success);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Error. Mismatched Coordinates..");
        return;
      }
    }


    void succeed (bool status)
    {
      if (status)
      {
        RCLCPP_INFO(this->get_logger(), "Motion executed successfully.");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute motion.");
      }
    }
  
  std::string getTimestamp()
    {
      auto now = std::chrono::system_clock::now();
      auto now_c = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
      return ss.str();
    }

    void saveTrajectoryToFile(const moveit_msgs::msg::RobotTrajectory& traj, const std::string& filename)
    {
      std::string folder_path = "/kinova-ros2/trajectories/";
      static bool first_call = true;
      std::filesystem::path dir(folder_path);
      
      // Create Directory if it is not present
      if (first_call)
      {
        if(std::filesystem::exists(dir))
        {
          for (const auto& entry : std::filesystem::directory_iterator(dir))
          {
            std::error_code ec;
            std::filesystem::remove_all(entry.path(), ec);
            if (ec)
            {
              RCLCPP_WARN(this->get_logger(), "Could not delete %s: %s",
                          entry.path().c_str(), ec.message().c_str());
            }
          }
          RCLCPP_INFO(this->get_logger(), "Cleared contents of folder: %s", folder_path.c_str());
        }
        else
        {
          std::filesystem::create_directories(dir);
          RCLCPP_INFO(this->get_logger(), "Created directory: %s", folder_path.c_str());
        }
        first_call = false;
      }
     

      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "joint_names" << YAML::Value << traj.joint_trajectory.joint_names;
      out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;

      for (const auto& pt : traj.joint_trajectory.points)
      {
        out << YAML::BeginMap;
        out << YAML::Key << "positions" << YAML::Value << YAML::Flow << pt.positions;
        out << YAML::Key << "velocities" << YAML::Value << YAML::Flow << pt.velocities;
        out << YAML::Key << "accelerations" << YAML::Value << YAML::Flow << pt.accelerations;
        out << YAML::Key << "time_from_start" << YAML::Value << (pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9);
        out << YAML::EndMap;
      }

      out << YAML::EndSeq;
      out << YAML::EndMap;

      std::ofstream fout(folder_path + "/" + filename);
      fout << out.c_str();
      RCLCPP_INFO(this->get_logger(), "Trajectory saved to %s", filename.c_str());
    }

  
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription;
  
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_manipulator; // Lazy-initialized
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper; // Lazy-initialized
 
};

int main (int argc, char *argv []) 
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgentSubscriber>());
  rclcpp::shutdown();
  return 0;
}

