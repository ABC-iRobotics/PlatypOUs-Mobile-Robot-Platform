cmake_minimum_required(VERSION 3.0.2)
project(auto_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_compile_options(-std=c++11)
add_compile_options(-Wall -Wextra -pedantic)

add_message_files(
  FILES
  Info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(auto_controller_node src/auto_controller_node.cpp)

target_link_libraries(auto_controller_node ${catkin_LIBRARIES})
