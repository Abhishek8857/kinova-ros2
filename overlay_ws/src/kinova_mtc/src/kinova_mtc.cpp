#include <fstream>
#include <iostream>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_subscriber_;
  geometry_msgs::msg::PoseStamped current_grasp_pose_;

  void graspPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  grasp_pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/grasp_pose", 10, std::bind(&MTCTaskNode::graspPoseCallback, this, std::placeholders::_1));

}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::graspPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_grasp_pose_ = *msg;
  setupPlanningScene();
  doTask();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.02, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = current_grasp_pose_.pose.position.x;
  pose.position.y = current_grasp_pose_.pose.position.y;
  pose.position.z = current_grasp_pose_.pose.position.z;
  pose.orientation.x = current_grasp_pose_.pose.orientation.x;
  pose.orientation.y = current_grasp_pose_.pose.orientation.y;
  pose.orientation.z = current_grasp_pose_.pose.orientation.z;
  pose.orientation.w = current_grasp_pose_.pose.orientation.w;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);

  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "world";
  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions = { 1.5, 1.5, 0.01 };

  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.0;
  table_pose.position.y = 0.0;
  table_pose.position.z = -0.04;
  table_pose.orientation.x = 0.0;
  table_pose.orientation.y = 0.0;
  table_pose.orientation.z = 0.0;
  table_pose.orientation.w = 1.0;
  table.pose = table_pose;

  psi.applyCollisionObject(table);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed");
    return;
  }

  try
  {
    task_.plan(5);
  }
  catch(mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    // Log error message (print the error code)
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    RCLCPP_ERROR_STREAM(LOGGER, result.val);
    return;
  }

  // task_.reset();
  // task_.introspection().reset();

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "end_effector_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  const double max_velocity_scaling_factor = 0.2;
  const double max_acceleration_scaling_factor = 0.2;

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  sampling_planner->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  interpolation_planner->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  cartesian_planner->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
  cartesian_planner->setStepSize(.01);

  auto stage = std::make_unique<mtc::stages::MoveTo>("start at observation", interpolation_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
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
  stage->setGoal(joint_values);
  task.add(std::move(stage));

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("Open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, interpolation_planner } });
  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("Open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI * 2);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(0.0 , Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0.0 , Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0.0 , Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.15;
      
      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      // clang-format on
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("Close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, interpolation_planner } });
                                                  // { hand_group_name, sampling_planner } });
    // clang-format on
    stage_move_to_place->setTimeout(10.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    // stage_move_to_place->restrictDirection(mtc::stages::MoveTo::FORWARD);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "base_link";
      target_pose_msg.pose.position.x = 0.08;
      target_pose_msg.pose.position.y = 0.35;
      target_pose_msg.pose.position.z = 0.25;
      target_pose_msg.pose.orientation.x = 1.0;
      target_pose_msg.pose.orientation.y = 0.0;
      target_pose_msg.pose.orientation.z = 0.0;
      target_pose_msg.pose.orientation.w = 0.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("Open");
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.2);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return to observation", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
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
    stage->setGoal(joint_values);
    task.add(std::move(stage));
  }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // mtc_task_node->setupPlanningScene();
  // mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
