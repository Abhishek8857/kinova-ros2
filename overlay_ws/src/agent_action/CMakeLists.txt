cmake_minimum_required(VERSION 3.8)
project(agent_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(agent_action_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

ament_export_dependencies(rosidl_default_runtime)


add_executable(execute_motion_server
  src/execute_motion_server.cpp
)

target_include_directories(execute_motion_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(execute_motion_server
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  agent_action_interface
)

target_link_libraries(execute_motion_server
  moveit_core::moveit_robot_trajectory
  moveit_core::moveit_trajectory_processing
)


# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(execute_motion_server ${cpp_typesupport_target})

install(TARGETS
  execute_motion_server
  DESTINATION lib/${PROJECT_NAME}
)

# Client
add_executable(execute_motion_client
  src/execute_motion_client.cpp
)

target_include_directories(execute_motion_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(execute_motion_client
  rclcpp
  rclcpp_action
  agent_action_interface
)

# Link generated action typesupport
# rosidl_get_typesupport_target(
#   cpp_typesupport_target_client
#   ${PROJECT_NAME}
#   "rosidl_typesupport_cpp"
# )
# target_link_libraries(execute_motion_client ${cpp_typesupport_target_client})

# Install client
install(TARGETS
  execute_motion_client
  DESTINATION lib/${PROJECT_NAME}
)

# Python version
install(PROGRAMS
  scripts/execute_motion_client.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
