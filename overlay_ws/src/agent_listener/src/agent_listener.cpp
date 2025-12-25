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
# include <nlohmann/json.hpp>

using std::placeholders::_1;
using json = nlohmann::json;

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
      
      // Create logs directory if it doesn't exist
      std::filesystem::create_directories("motion_logs");
      
      // Delete old log files in the directory
      for (const auto& entry : std::filesystem::directory_iterator("motion_logs"))
      {
        if (entry.is_regular_file() && entry.path().extension() == ".json")
        {
          std::filesystem::remove(entry.path());
          RCLCPP_INFO(this->get_logger(), "Deleted old log file: %s", entry.path().c_str());
        }
      }
      
      // Generate log filename once at startup
      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      std::stringstream filename;
      filename << "motion_logs/motion_log_" 
               << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") 
               << ".json";
      log_filename_ = filename.str();
      
      // Initialize the JSON array in the file
      std::ofstream file(log_filename_);
      if (file.is_open())
      {
        file << "[]" << std::endl;  // Start with empty array
        file.close();
      }
      
      RCLCPP_INFO(this->get_logger(), "Logging to file: %s", log_filename_.c_str());
    }
    
    ~AgentSubscriber()
    {
      // Delete the log file on shutdown
      if (std::filesystem::exists(log_filename_))
      {
        std::filesystem::remove(log_filename_);
        RCLCPP_INFO(this->get_logger(), "Log file deleted: %s", log_filename_.c_str());
      }
    }

  private:
    std::string getCurrentTimestamp()
    {
      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
      
      std::stringstream ss;
      ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
      ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
      return ss.str();
    }

    std::string getDetailedErrorDescription(const moveit::core::MoveItErrorCode& error_code)
    {
      std::string description;
      
      switch(error_code.val)
      {
        case moveit::core::MoveItErrorCode::SUCCESS:
          description = "Operation completed successfully";
          break;
        case moveit::core::MoveItErrorCode::FAILURE:
          description = "Generic failure - operation did not succeed for unspecified reasons";
          break;
        case moveit::core::MoveItErrorCode::PLANNING_FAILED:
          description = "Motion planner failed to find a valid path to the target. This could be due to: "
                       "unreachable target pose, joint limits, collision constraints, or insufficient planning time";
          break;
        case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
          description = "Generated motion plan is invalid or unsafe. May contain trajectory errors, "
                       "violate velocity/acceleration limits, or have discontinuities";
          break;
        case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
          description = "Planning scene changed during execution (new obstacles detected or collision objects moved), "
                       "making the current plan unsafe";
          break;
        case moveit::core::MoveItErrorCode::CONTROL_FAILED:
          description = "Robot controller failed to execute the trajectory. Possible causes: "
                       "hardware communication error, joint servo errors, or emergency stop triggered";
          break;
        case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
          description = "Failed to acquire required sensor data (camera, force sensor, etc.) needed for planning";
          break;
        case moveit::core::MoveItErrorCode::TIMED_OUT:
          description = "Operation exceeded the allocated time limit. Try increasing planning time or "
                       "simplifying the motion request";
          break;
        case moveit::core::MoveItErrorCode::PREEMPTED:
          description = "Operation was cancelled or preempted by a new request before completion";
          break;
        case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
          description = "Robot's current state is in collision with environment or self. "
                       "Check for obstacles touching the robot or invalid joint configurations";
          break;
        case moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
          description = "Current robot state violates the path constraints specified for motion planning";
          break;
        case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
          description = "Target pose would result in collision with environment or self-collision. "
                       "The goal position may be inside an obstacle or cause joint interference";
          break;
        case moveit::core::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
          description = "Target goal violates path constraints (orientation, position bounds, etc.)";
          break;
        case moveit::core::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
          description = "Goal constraints are violated or unreachable with current robot configuration";
          break;
        case moveit::core::MoveItErrorCode::INVALID_GROUP_NAME:
          description = "Specified planning group name doesn't exist in robot configuration";
          break;
        case moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
          description = "Goal constraints are malformed or contain invalid values";
          break;
        case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE:
          description = "Robot state is invalid (NaN values, out of bounds joints, etc.)";
          break;
        case moveit::core::MoveItErrorCode::INVALID_LINK_NAME:
          description = "Specified link name doesn't exist in robot model";
          break;
        case moveit::core::MoveItErrorCode::INVALID_OBJECT_NAME:
          description = "Referenced collision object name not found in planning scene";
          break;
        case moveit::core::MoveItErrorCode::FRAME_TRANSFORM_FAILURE:
          description = "Failed to transform between coordinate frames. Check TF tree and frame names";
          break;
        case moveit::core::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE:
          description = "Collision checking service is not available or not responding";
          break;
        case moveit::core::MoveItErrorCode::ROBOT_STATE_STALE:
          description = "Robot state information is outdated. May indicate communication issues with robot";
          break;
        case moveit::core::MoveItErrorCode::SENSOR_INFO_STALE:
          description = "Sensor information is outdated or not being updated";
          break;
        case moveit::core::MoveItErrorCode::COMMUNICATION_FAILURE:
          description = "Communication failure with robot controller or action server. "
                       "Check network connection and verify action server is running";
          break;
        case moveit::core::MoveItErrorCode::NO_IK_SOLUTION:
          description = "Inverse kinematics solver could not find a joint configuration for the target pose. "
                       "Target may be out of reach, in a singularity, or violate joint limits";
          break;
        default:
          description = "Unknown error code: " + std::to_string(error_code.val);
          break;
      }
      
      return description;
    }

    void appendLogEntry(const json& log_data)
    {
      // Read existing logs from file
      json existing_logs = json::array();
      
      std::ifstream read_file(log_filename_);
      if (read_file.is_open())
      {
        try
        {
          read_file >> existing_logs;
        }
        catch (const json::exception& e)
        {
          RCLCPP_WARN(this->get_logger(), "Could not parse existing logs, starting fresh: %s", e.what());
          existing_logs = json::array();
        }
        read_file.close();
      }
      
      // Append new log entry
      existing_logs.push_back(log_data);
      
      // Write back to file immediately
      std::ofstream write_file(log_filename_);
      if (write_file.is_open())
      {
        write_file << std::setw(2) << existing_logs << std::endl;
        write_file.close();
        RCLCPP_INFO(this->get_logger(), "Log appended in real-time. Total entries: %zu", existing_logs.size());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to log file: %s", log_filename_.c_str());
      }
    }

    void collision_objects ()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_manipulator->getPlanningFrame();
        collision_object.id = "ground_plane";

        // Define the ground plane as a large box
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 2.0;
        primitive.dimensions[primitive.BOX_Y] = 2.0;
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
      
      // Option 3: -90° around Y-axis (common for many robot arms)
      orientation.w = 0.7071;  // cos(-90°/2)
      orientation.x = 0.0;
      orientation.y = -0.7071; // sin(-90°/2)
      orientation.z = 0.0;
      
      return orientation;
    }


    void agent_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      // Initialize log structure
      json motion_log;
      motion_log["timestamp_received"] = getCurrentTimestamp();
      motion_log["node_name"] = this->get_name();
      
      // DEBUG: Print out the recieved Coordinates
      RCLCPP_INFO(this->get_logger(), "Recieved Cartesian Coordinates..");
      
      // Log received coordinates
      json coordinates = json::array();
      for (size_t i {0}; i < msg->data.size(); i++)
      {
        RCLCPP_INFO(this->get_logger(), " -[%zu]: %f", i, msg->data[i]);
        coordinates.push_back(msg->data[i]);
      }
      motion_log["received_coordinates"] = coordinates;

      // Lazy initialization of MoveGroupInterface
      if (!move_group_manipulator && !move_group_gripper)
      {
        motion_log["moveit_initialized"] = true;
        motion_log["timestamp_moveit_init"] = getCurrentTimestamp();
        
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
      else
      {
        motion_log["moveit_initialized"] = false;
      }

      // Check if we need to execute Joint Target or Pose Target
      if (msg->data[0] == 0.0)
      {
        motion_log["motion_type"] = "joint_target";
        
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
        
        motion_log["joint_values"] = joint_values;

        // Plan and execute
        RCLCPP_INFO(this->get_logger(), "Planning the target pose ...");
        motion_log["timestamp_planning_start"] = getCurrentTimestamp();
        
        move_group_manipulator->setJointValueTarget(joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_start = std::chrono::high_resolution_clock::now();
        auto plan_result = move_group_manipulator->plan(plan);
        bool success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
        auto plan_end = std::chrono::high_resolution_clock::now();
        
        motion_log["planning_success"] = success;
        motion_log["planning_error_code"] = moveit::core::error_code_to_string(plan_result);
        motion_log["planning_error_description"] = getDetailedErrorDescription(plan_result);
        motion_log["planning_time_seconds"] = 
          std::chrono::duration<double>(plan_end - plan_start).count();
        motion_log["timestamp_planning_end"] = getCurrentTimestamp();

        if (success)
        {
          motion_log["timestamp_execution_start"] = getCurrentTimestamp();
          auto exec_start = std::chrono::high_resolution_clock::now();
          
          auto exec_result = move_group_manipulator->execute(plan);
          bool exec_success = (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
          
          auto exec_end = std::chrono::high_resolution_clock::now();
          motion_log["execution_success"] = exec_success;
          motion_log["execution_error_code"] = moveit::core::error_code_to_string(exec_result);
          motion_log["execution_error_description"] = getDetailedErrorDescription(exec_result);
          motion_log["execution_time_seconds"] = 
            std::chrono::duration<double>(exec_end - exec_start).count();
          motion_log["timestamp_execution_end"] = getCurrentTimestamp();
          
          success = exec_success;
        }
        else
        {
          motion_log["execution_skipped"] = "Planning failed";
        }
        
        succeed(success);
        motion_log["final_status"] = success ? "SUCCESS" : "FAILED";
      }
      else if (msg->data[0] == 1.0)
      {
        motion_log["motion_type"] = "pose_target";
        
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
          motion_log["orientation_type"] = "provided";
        }
        else
        {
          // Use automatic downward orientation
          target_pose.orientation = getDownwardOrientation();
          RCLCPP_INFO(this->get_logger(), "Using automatic downward orientation (w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
                      target_pose.orientation.w, target_pose.orientation.x, 
                      target_pose.orientation.y, target_pose.orientation.z);
          motion_log["orientation_type"] = "automatic_downward";
        }
        
        // Log target pose
        motion_log["target_pose"] = {
          {"position", {
            {"x", target_pose.position.x},
            {"y", target_pose.position.y},
            {"z", target_pose.position.z}
          }},
          {"orientation", {
            {"w", target_pose.orientation.w},
            {"x", target_pose.orientation.x},
            {"y", target_pose.orientation.y},
            {"z", target_pose.orientation.z}
          }}
        };

        // Plan and execute
        RCLCPP_INFO(this->get_logger(), "Planning the target pose ...");
        motion_log["timestamp_planning_start"] = getCurrentTimestamp();
        
        move_group_manipulator->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_start = std::chrono::high_resolution_clock::now();
        auto plan_result = move_group_manipulator->plan(plan);
        bool success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
        auto plan_end = std::chrono::high_resolution_clock::now();
        
        motion_log["planning_success"] = success;
        motion_log["planning_error_code"] = moveit::core::error_code_to_string(plan_result);
        motion_log["planning_error_description"] = getDetailedErrorDescription(plan_result);
        motion_log["planning_time_seconds"] = 
          std::chrono::duration<double>(plan_end - plan_start).count();
        motion_log["timestamp_planning_end"] = getCurrentTimestamp();
        
        if (success)
        {
          motion_log["timestamp_execution_start"] = getCurrentTimestamp();
          auto exec_start = std::chrono::high_resolution_clock::now();
          
          auto exec_result = move_group_manipulator->execute(plan);
          bool exec_success = (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
          
          auto exec_end = std::chrono::high_resolution_clock::now();
          motion_log["execution_success"] = exec_success;
          motion_log["execution_error_code"] = moveit::core::error_code_to_string(exec_result);
          motion_log["execution_error_description"] = getDetailedErrorDescription(exec_result);
          motion_log["execution_time_seconds"] = 
            std::chrono::duration<double>(exec_end - exec_start).count();
          motion_log["timestamp_execution_end"] = getCurrentTimestamp();
          
          success = exec_success;
        }
        else
        {
          motion_log["execution_skipped"] = "Planning failed";
        }
        
        succeed(success);
        motion_log["final_status"] = success ? "SUCCESS" : "FAILED";
      }
      else if (msg->data[0] == 2.0)
      {
        motion_log["motion_type"] = "gripper";
        
        // Parse the coordinates
        std::map<std::string, double> gripper_values =
        {
          {"robotiq_85_left_knuckle_joint", msg->data[1]},
          {"robotiq_85_right_knuckle_joint", msg->data[2]}
        };
        
        motion_log["gripper_values"] = gripper_values;

        // Planning the Gripper motion
        RCLCPP_INFO(this->get_logger(), "Planning the Gripper motion");
        motion_log["timestamp_planning_start"] = getCurrentTimestamp();
        
        move_group_gripper->setJointValueTarget(gripper_values);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_start = std::chrono::high_resolution_clock::now();
        auto plan_result = move_group_gripper->plan(plan);
        bool success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
        auto plan_end = std::chrono::high_resolution_clock::now();
        
        motion_log["planning_success"] = success;
        motion_log["planning_error_code"] = moveit::core::error_code_to_string(plan_result);
        motion_log["planning_error_description"] = getDetailedErrorDescription(plan_result);
        motion_log["planning_time_seconds"] = 
          std::chrono::duration<double>(plan_end - plan_start).count();
        motion_log["timestamp_planning_end"] = getCurrentTimestamp();
        
        if (success)
        {
          motion_log["timestamp_execution_start"] = getCurrentTimestamp();
          auto exec_start = std::chrono::high_resolution_clock::now();
          
          auto exec_result = move_group_gripper->execute(plan);
          bool exec_success = (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
          
          auto exec_end = std::chrono::high_resolution_clock::now();
          motion_log["execution_success"] = exec_success;
          motion_log["execution_error_code"] = moveit::core::error_code_to_string(exec_result);
          motion_log["execution_error_description"] = getDetailedErrorDescription(exec_result);
          motion_log["execution_time_seconds"] = 
            std::chrono::duration<double>(exec_end - exec_start).count();
          motion_log["timestamp_execution_end"] = getCurrentTimestamp();
          
          success = exec_success;
        }
        else
        {
          motion_log["execution_skipped"] = "Planning failed";
        }

        succeed(success);
        motion_log["final_status"] = success ? "SUCCESS" : "FAILED";
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Error. Mismatched Coordinates..");
        motion_log["motion_type"] = "error";
        motion_log["error"] = "Mismatched coordinates";
        motion_log["final_status"] = "ERROR";
        
        appendLogEntry(motion_log);
        return;
      }
      
      // Save the complete log
      motion_log["timestamp_completed"] = getCurrentTimestamp();
      appendLogEntry(motion_log);
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
  
  // JSON logging members
  std::string log_filename_;


};

int main (int argc, char *argv [])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgentSubscriber>());
  rclcpp::shutdown();
  return 0;
}