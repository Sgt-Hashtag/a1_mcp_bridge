cmake_minimum_required(VERSION 3.0.2)
project(a1_mcp_bridge)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  rospy
)

add_message_files(
  FILES
  RobotStateReport.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install the Python MCP script
catkin_install_python(PROGRAMS
  scripts/mcp_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

