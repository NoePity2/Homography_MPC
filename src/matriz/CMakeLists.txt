cmake_minimum_required(VERSION 3.0.2)
project(matriz)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
  LIBRARIES matriz
  CATKIN_DEPENDS roscpp rospy std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(matriz_node src/matriz_node.cpp)
target_link_libraries(matriz_node ${catkin_LIBRARIES})

#add_executable(matriz_listener_node src/matriz_listener_node.cpp)
#target_link_libraries(matriz_listener_node ${catkin_LIBRARIES})
