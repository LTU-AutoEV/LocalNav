#include <ros/ros.h>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_msgs/GridMap.h"
#include "grid_map_ros/grid_map_ros.hpp"

#include <nav_msgs/OccupancyGrid.h>

#include <iostream>

/* GridMap2CostMap
 *
 * Convert grid map ro cost map
 *
 */
class GridMap2CostMap
{
public:
    GridMap2CostMap();

private:
    // Callbacks
    void gmCallback(const grid_map_msgs::GridMap& scan);

    // ROS objects
    ros::NodeHandle nh_;
    ros::Subscriber gm_sub_;
    ros::Publisher pub_;
};

// Constructor
//   Set up publisher and subscriber
GridMap2CostMap::GridMap2CostMap()
    :nh_("~")
{

    gm_sub_  = nh_.subscribe("/pointcloud2gridmap/grid_map", 10, &GridMap2CostMap::gmCallback, this);
    pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 10);
}

// Callback function for the points
void GridMap2CostMap::gmCallback(const grid_map_msgs::GridMap& msg)
{
    grid_map::GridMap gm;
    grid_map::GridMapRosConverter::fromMessage(msg, gm);

    // Convert to OccupancyGrid msg.
    nav_msgs::OccupancyGrid occupancyGrid;
    grid_map::GridMapRosConverter::toOccupancyGrid(gm, "elevation", 0.1, 1.0, occupancyGrid);

    pub_.publish(occupancyGrid);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gridmap2costmap");
  GridMap2CostMap GridMap2CostMap;

  ros::spin();
}

