cmake_minimum_required(VERSION 3.0.2)
project(visual_predictive)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  nav_msgs
  rospy
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(VISP REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs tf geometry_msgs nav_msgs cv_bridge image_transport visp_ros
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/visual_predictive/
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/visual_predictive_node.cpp src/utils.cpp src/vision_utils.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${VISP_LIBRARIES}
 )
 
 add_executable(video_publisher src/video_publisher.cpp)
target_link_libraries(video_publisher ${catkin_LIBRARIES})
target_link_libraries(video_publisher ${OpenCV_LIBRARIES})

add_executable(video_listener src/video_listener.cpp)
target_link_libraries(video_listener ${catkin_LIBRARIES})
target_link_libraries(video_listener ${OpenCV_LIBRARIES})

add_executable(save_image_listener src/save_image_listener.cpp)
target_link_libraries(save_image_listener ${catkin_LIBRARIES})
target_link_libraries(save_image_listener ${OpenCV_LIBRARIES})

add_executable(real_homography_node src/real_homography_node.cpp src/utils.cpp src/vision_utils.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(real_homography_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${VISP_LIBRARIES}
 )





