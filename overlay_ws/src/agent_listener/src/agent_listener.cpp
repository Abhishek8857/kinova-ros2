# include <memory>
# include <rclcpp/rclcpp.hpp>
# include <geometry_msgs/msg/point.hpp>
# include <moveit/move_group_interface/move_group_interface.h>
# include <moveit/planning_scene_interface/planning_scene_interface.h>
# include <moveit_msgs/msg/collision_object.hpp>
# include <shape_msgs/msg/solid_primitive.hpp>
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
    void collision_objects ()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_manipulator->getPlanningFrame();
        collision_object.id = "ground_plane";

        // Define the ground plane as a large box
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 2.50;
        primitive.dimensions[primitive.BOX_Y] = 2.50;
        primitive.dimensions[primitive.BOX_Z] = 0.01;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.005;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        // Add to planning scene
        planning_scene_interface_.applyCollisionObject(collision_object);
        RCLCPP_INFO(this->get_logger(), "Ground plane collision object added to planning scene");

        // Give some time for the collision object to be processed
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }


    void ensureCollisionObjectsExist()
    {
      if (!collision_objects_added_)
      {
        collision_objects();
        collision_objects_added_ = true;
      }
    }

    void setDownwardOrientationConstraint()
    {
      // Set orientation constraint to keep the end effector pointing down
      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.link_name = move_group_manipulator->getEndEffectorLink();
      ocm.header.frame_id = move_group_manipulator->getPlanningFrame();

      // Downward orientation (180° rotation around X-axis)
      ocm.orientation.w = 0.0;
      ocm.orientation.x = 1.0;
      ocm.orientation.y = 0.0;
      ocm.orientation.z = 0.0;
      
      ocm.absolute_x_axis_tolerance = 0.2;  // Allow some deviation (radians)
      ocm.absolute_y_axis_tolerance = 0.2;
      ocm.absolute_z_axis_tolerance = 0.2;
      ocm.weight = 1.0;

      moveit_msgs::msg::Constraints constraints;
      constraints.orientation_constraints.push_back(ocm);
      
      move_group_manipulator->setPathConstraints(constraints);
      RCLCPP_INFO(this->get_logger(), "Downward orientation constraint set");

    }

    geometry_msgs::msg::Quaternion getDownwardOrientation()
    {
      // Try different common downward orientations for your robot
      geometry_msgs::msg::Quaternion orientation;
      
      // Option 1: Standard downward (Z-axis pointing down) - 180° around X
      // orientation.w = 0.0;
      // orientation.x = 1.0;
      // orientation.y = 0.0;
      // orientation.z = 0.0;
      
      // Option 2: 180° rotation around Y-axis
      // orientation.w = 0.0;
      // orientation.x = 0.0;
      // orientation.y = 1.0;
      // orientation.z = 0.0;
      
      // Option 3: -90° around Y-axis (common for many robot arms)
      orientation.w = 0.7071;  // cos(-90°/2)
      orientation.x = 0.0;
      orientation.y = -0.7071; // sin(-90°/2)
      orientation.z = 0.0;
      
      // Option 4: 90° around Y-axis
      // orientation.w = 0.7071;
      // orientation.x = 0.0;
      // orientation.y = 0.7071;
      // orientation.z = 0.0;
      
      // Option 5: -90° around X-axis
      // orientation.w = 0.7071;
      // orientation.x = -0.7071;
      // orientation.y = 0.0;
      // orientation.z = 0.0;
      
      // Option 6: 90° around X-axis
      // orientation.w = 0.7071;
      // orientation.x = 0.7071;
      // orientation.y = 0.0;
      // orientation.z = 0.0;
      
      return orientation;
    }


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

        // Add ground plane collision object after initialization
        ensureCollisionObjectsExist();

        // Configure planning parameters for better collision avoidance
        move_group_manipulator->setPlanningTime(10.0);
        move_group_manipulator->setNumPlanningAttempts(10);
        move_group_manipulator->setMaxVelocityScalingFactor(0.5);
        move_group_manipulator->setMaxAccelerationScalingFactor(0.5);
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

        // Check if the orientation is provided
        bool has_orientation = (msg->data[4] != 0 || msg->data[5] != 0 || msg->data[6] != 0 || msg->data[7] != 0);

        // Parse the orientation
        if(has_orientation)
        {
          target_pose.orientation.w = msg->data[4];
          target_pose.orientation.x = msg->data[5];
          target_pose.orientation.y = msg->data[6];
          target_pose.orientation.z = msg->data[7];
          RCLCPP_INFO(this->get_logger(), "Using provided orientation (w=%.3f, x=%.3f, y=%.3f, z=%.3f)", 
                      msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
        }
        else
        {
          // Use automatic downward orientation
          target_pose.orientation = getDownwardOrientation();
          RCLCPP_INFO(this->get_logger(), "Using automatic downward orientation (w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
                      target_pose.orientation.w, target_pose.orientation.x, 
                      target_pose.orientation.y, target_pose.orientation.z);// Use automatic downward orientation
        }

        // Plan and execute
        RCLCPP_INFO(this->get_logger(), "Planning the target pose ...");
        move_group_manipulator->setPoseTarget(target_pose);

        // Verify if the motion was successful
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group_manipulator->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
          move_group_manipulator->execute(plan);
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


  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_manipulator; // Lazy-initialized
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper; // Lazy-initialized
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  bool collision_objects_added_ = false;
  bool use_orientation_constraint_ = false; 


};

int main (int argc, char *argv [])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgentSubscriber>());
  rclcpp::shutdown();
  return 0;
}