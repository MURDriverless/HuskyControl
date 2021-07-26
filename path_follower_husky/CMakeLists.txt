cmake_minimum_required(VERSION 3.0.2)
project(path_follower_husky)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  mur_common
)

add_definitions(-std=c++14)
include_directories(src ${catkin_INCLUDE_DIRS})
catkin_package()

add_executable(path_follower_husky src/follower_husky.cpp)

target_link_libraries(path_follower_husky ${catkin_LIBRARIES})





