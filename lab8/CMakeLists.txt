cmake_minimum_required(VERSION 3.5)
project(lab8)

find_package(ament_cmake REQUIRED)

# Install launch and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/orchestrator.py
  scripts/explore_action_server.py
  scripts/aruco_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
