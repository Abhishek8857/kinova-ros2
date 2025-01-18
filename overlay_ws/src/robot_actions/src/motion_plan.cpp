// # include <memory>
// # include <rclcpp/rclcpp.hpp>
// # include <geometry_msgs/msg/point.hpp>
// # include <moveit/move_group_interface/move_group_interface.h>


// int main (int argc, char *argv[])
// {

//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);

//   auto const node = std::make_shared<rclcpp::Node>
//     (
//         "moveit_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//     );

//     // Create ROS Logger
//     auto const logger = rclcpp::get_logger("moveit_node");

//     // Create the Moveit MoveGroup Interface
//     using moveit::planning_interface::MoveGroupInterface;
//     auto move_group_interface = MoveGroupInterface(node, "manipulator");

//     // Set a target Pose
//     auto const target_pose = []
//     {
//         geometry_msgs::msg::Pose msg;
//         msg.orientation.w = 1.0;
//         msg.position.x = 0.28;
//         msg.position.y = -0.2;
//         msg.position.z = 0.5;
//         return msg;
//     } ();

//     move_group_interface.setPoseTarget(target_pose);

//     // Create a plan to that target pose
//     auto const [success, plan] = [&move_group_interface]
//     {
//         moveit::planning_interface::MoveGroupInterface::Plan msg;
//         auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//         return std::make_pair(ok, msg);
//     } ();

//     // Execute the plan
//     if(success) 
//     {
//         move_group_interface.execute(plan);
//     }
//     else
//     {
//         RCLCPP_ERROR(logger, "Planning failed!");
//     }

//   rclcpp::shutdown();
//   return 0;
// }


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
