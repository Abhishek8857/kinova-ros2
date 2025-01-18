# include <memory>
# include <rclcpp/rclcpp.hpp>
# include <geometry_msgs/msg/point.hpp>
# include <moveit/move_group_interface/move_group_interface.h>
# include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

class AgentSubscriber : public rclcpp::Node
{
  public:
    // Initialise the Node
    AgentSubscriber () : Node ("agent_subscriber")
    {
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
        bool success = (move_group_manipulator->move() == moveit::core::MoveItErrorCode::SUCCESS);
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
        bool success = (move_group_manipulator->move() == moveit::core::MoveItErrorCode::SUCCESS);
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
        bool success = (move_group_gripper->move() == moveit::core::MoveItErrorCode::SUCCESS);
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
