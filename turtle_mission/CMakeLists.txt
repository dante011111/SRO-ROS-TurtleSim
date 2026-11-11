cmake_minimum_required(VERSION 3.0.2)
project(turtle_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs turtlesim message_generation)

add_message_files(FILES
  Waypoint.msg
  MissionStatus.msg
  WallSensor.msg
  Score.msg)
add_service_files(FILES StartMission.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs turtlesim message_runtime)

# Installs every script in scripts/, so you never edit this list again
file(GLOB PY_SCRIPTS scripts/*.py)
catkin_install_python(PROGRAMS ${PY_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
