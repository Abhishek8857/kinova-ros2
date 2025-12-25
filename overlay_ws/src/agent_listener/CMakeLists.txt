cmake_minimum_required(VERSION 3.8)
project(agent_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
add_executable(agent_listener src/agent_listener.cpp)
add_executable(trajectory_player src/trajectory_player.cpp)
add_executable(trajectory_recorder src/trajectory_recorder.cpp)


ament_target_dependencies(
  agent_listener
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
)

ament_target_dependencies (trajectory_player rclcpp moveit_ros_planning_interface control_msgs)
ament_target_dependencies(trajectory_recorder rclcpp sensor_msgs yaml-cpp moveit_ros_planning_interface)

target_link_libraries(agent_listener yaml-cpp nlohmann_json::nlohmann_json)
target_link_libraries(trajectory_player yaml-cpp)
target_link_libraries(trajectory_recorder yaml-cpp)



target_include_directories(agent_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(agent_listener PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(trajectory_player PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trajectory_player PUBLIC c_std_99 cxx_std_17)

target_include_directories(trajectory_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trajectory_recorder PUBLIC c_std_99 cxx_std_17)  

install(TARGETS agent_listener trajectory_player trajectory_recorder  
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
