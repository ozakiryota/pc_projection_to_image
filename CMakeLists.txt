cmake_minimum_required(VERSION 2.8.3)
project(pc_projection_to_image)

add_compile_options(-std=c++17 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  image_transport
  image_geometry
  cv_bridge
  rosbag
)
find_package(PCL)
find_package(OpenCV)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pc_projection_to_image src/pc_projection_to_image.cpp)
target_link_libraries(pc_projection_to_image
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(pc_projection_to_image_offline src/pc_projection_to_image_offline.cpp)
target_link_libraries(pc_projection_to_image_offline
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
