cmake_minimum_required(VERSION 2.8.3)
project(LocalNav)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Get the information about this package's buildtime dependencies
find_package(catkin
    REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    pcl_ros
    grid_map_ros
    grid_map_pcl
    pcl_conversions
    )

# Declare that this catkin package's runtime dependencies
catkin_package(
    CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    sensor_msgs
    grid_map_ros
    pcl_ros
    grid_map_pcl
    pcl_conversions
    DEPENDS
    PCL
)

find_package(PCL REQUIRED COMPONENTS common surface kdtree)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(gridmap2costmap src/gridmap2costmap.cpp)
add_executable(pointcloud2gridmap src/pointcloud2gridmap.cpp)

add_dependencies(gridmap2costmap ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pointcloud2gridmap ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gridmap2costmap ${catkin_LIBRARIES})
target_link_libraries(pointcloud2gridmap ${catkin_LIBRARIES} pcl_common pcl_surface pcl_kdtree pcl_ml)
