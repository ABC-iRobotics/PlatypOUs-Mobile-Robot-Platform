cmake_minimum_required(VERSION 3.0.2)
project(map_to_image)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  cv_bridge
  sensor_msgs
  image_transport
  message_generation
)

find_package( OpenCV REQUIRED )
add_compile_options(-Wall -Wextra -pedantic)

add_service_files(
  FILES
  Convert.srv
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
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(map_to_image_node src/map_to_image_node.cpp)

target_link_libraries(map_to_image_node ${catkin_LIBRARIES})
