cmake_minimum_required(VERSION 2.8.3)
project(body_axis_control_keyboard)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(body_axis_control_keyboard_node src/body_axis_control_keyboard_node.cpp)

target_link_libraries(body_axis_control_keyboard_node
   ${catkin_LIBRARIES}
 )

