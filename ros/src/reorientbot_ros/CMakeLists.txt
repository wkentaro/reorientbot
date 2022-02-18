cmake_minimum_required(VERSION 2.8.3)
project(mercury_ros)

find_package(
  catkin REQUIRED
  COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
)

add_message_files(
  FILES
  ObjectClass.msg
  ObjectClassArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
