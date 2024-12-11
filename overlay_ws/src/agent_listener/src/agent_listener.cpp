# include <rclcpp/rclcpp.hpp>
# include <geometry_msgs/msg/point.hpp>
# include <moveit/move_group_interface/move_group_interface.h>


int main (int argc, char *argv []) 
{
  // Initialise ROS and create the node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "kinova_motion_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create ROS Logger
  auto const logger = rclcpp::get_logger("kinova_motion_planner");

  // Create MoveIt group interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface {MoveGroupInterface (node, "manipulator")};

  // Define the joint values for the target pose
  std::map<std::string, double> joint_values = {
      {"joint_1", 0.0},
      {"joint_2", -0.9443},
      {"joint_3", -3.15353},
      {"joint_4", -2.1302},
      {"joint_5", 0.00588},
      {"joint_6", -1.2077},
      {"joint_7", 1.55037}
  };

  // Set joint value target
  move_group_interface.setJointValueTarget(joint_values);

  // Create a plan to that target pose
  auto const [success, plan] =
    [&move_group_interface]
    {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok {static_cast<bool>(move_group_interface.plan(msg))};
      return std::make_pair(ok, msg);
    }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning Failed");
  }


  rclcpp::shutdown();
  return 0;
}
