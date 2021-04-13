cmake_minimum_required(VERSION 3.0.2)
project(platypous_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Convert.srv
  RobotPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
