cmake_minimum_required(VERSION 2.8.3)
project(odom_listener)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES odom_listener
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_listener src/odom_listener.cpp)

target_link_libraries(odom_listener
   ${catkin_LIBRARIES}
 )
