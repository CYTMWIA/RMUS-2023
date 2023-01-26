cmake_minimum_required(VERSION 3.0.2)
project(rmus_solution)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
  Square.msg
  SquareArray.msg
)

add_service_files(
  FILES
  setgoal.srv
  switch.srv
  graspsignal.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime sensor_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)