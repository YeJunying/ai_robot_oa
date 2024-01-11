cmake_minimum_required(VERSION 3.1)

project(ai_robot_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  # nav_msgs
  # sensor_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  DIRECTORY
    msg
  FILES
    RecoveryStatus.msg
    OAVelocity.msg
)

add_service_files(
  FILES
  # GetPath.srv
    SetRobotPose.srv
    SetTargetPosition.srv
)

add_action_files(
  DIRECTORY
    action
  FILES
    MoveBase.action
)

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
  # nav_msgs
  # sensor_msgs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  # nav_msgs
  # sensor_msgs
  actionlib_msgs
)
