cmake_minimum_required(VERSION 3.22)
project(kinova_python)

find_package(ament_cmake REQUIRED)

ament_package()

install(PROGRAMS
  scripts/kinova_python.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/kinova_grasp.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
	)