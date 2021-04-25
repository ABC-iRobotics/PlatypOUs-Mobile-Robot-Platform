cmake_minimum_required(VERSION 3.0.2)
project(platypous_status)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  cv_bridge
  sensor_msgs
  image_transport
  platypous_msgs
)

find_package( OpenCV REQUIRED )
add_compile_options(-Wall -Wextra -pedantic)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(map_to_image src/map_to_image.cpp)
add_executable(status_publisher src/status_publisher.cpp)

target_link_libraries(map_to_image ${catkin_LIBRARIES})
target_link_libraries(status_publisher ${catkin_LIBRARIES})
