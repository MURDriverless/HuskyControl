cmake_minimum_required(VERSION 3.0.2)
project(path_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  mur_common
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_publisher src/path_publisher.cpp)

target_link_libraries(path_publisher ${catkin_LIBRARIES})
