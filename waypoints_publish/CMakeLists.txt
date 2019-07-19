cmake_minimum_required(VERSION 2.8.3)
project(waypoints_publish)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  tf
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoints_publish
  CATKIN_DEPENDS actionlib move_base_msgs roscpp tf
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)


add_executable(waypoints_publish src/waypoints_publish.cpp)

target_link_libraries(waypoints_publish
   ${catkin_LIBRARIES}
   ${yaml-cpp_LIBRARIES}
 )
