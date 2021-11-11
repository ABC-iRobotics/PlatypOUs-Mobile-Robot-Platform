cmake_minimum_required(VERSION 3.0.2)
project(platypous_driver)

find_package(catkin REQUIRED
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#catkin_install_python(PROGRAMS
#  scripts/odrive_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

add_executable(led_driver_node src/led_driver_node.cpp src/serialib/serialib.cpp)
target_link_libraries(led_driver_node ${catkin_LIBRARIES})
