cmake_minimum_required(VERSION 2.8.3)
project(local_nav)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  rospy
  pcl_ros
)

#find_package(OpenCV REQUIRED)

##############
## Reconfig ##
##############

#add dynamic reconfigure api
#generate_dynamic_reconfigure_options(
#cfg/Control.cfg
# cfg/RoadObstacle.cfg
# cfg/StopSign.cfg
#)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES actor_obstacle
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(cloud2grid src/pointcloud2Occupancy.cpp)
add_dependencies(cloud2grid ${PROJECT_NAME}_gencfg)
target_link_libraries(cloud2grid ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})

add_executable(grid2detect src/Occupancy2ObjectDetect.cpp)
add_dependencies(grid2detect ${PROJECT_NAME}_gencfg)
target_link_libraries(grid2detect ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})

