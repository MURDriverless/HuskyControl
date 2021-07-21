cmake_minimum_required(VERSION 3.0.2)
project(husky_terminal_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(husky_terminal_control 
	src/main.cpp
  src/terminalcontrol.cpp
  src/terminalcontrol.h
  )

target_link_libraries(husky_terminal_control ${catkin_LIBRARIES})
