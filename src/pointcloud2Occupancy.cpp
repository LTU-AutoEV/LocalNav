#include <iostream>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include "/velodyne/pointcloud2.h"
#include <cmath>

constexpr int hieght = 3;
constexpr int length = 4;
constexpr int width = 3;

/*PCloud2GMap
 *
 * Converts the PointCloud2 data from the velodyne Lidar
 *  into a grid_map
 *
 * Publish:     grid_map on "~/grid_map"
 *
 * Subscrivbe:  PointCloud2 on "/velodyne_points"
 */
class PCloud2GMap {

public:
    PCloud2GMap();

private:
    // Callback
    void CloudCallback(const pcl::PointCloud<pcl::XYZ>::Ptr &cloud);

    ros::NodeHandle nh_;
    ros::Publisher  pub_;
    ros::Subscriber sub_;
};

// constructor
//  set up publisher and subscriber
PCloud2GMap::PCloud2GMap()
{
    nh_ = ros::NodeHandle("~");

    // publish on /grid_map
    pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_map", 100);
    // subscribe to the pointcloud2 data comming from the lidar
    sub_ = nh_.subscribe("/velodyne/pointcloud2", 100, &PCloud2GMap::CloudCallback, this);
}

void
PCloud2GMap::CloudCallback(const pcl::PointCloud<pcl::XYZ>::Ptr &cloud)
{

    nav_msgs::OccupancyGrid grid;

    grid.info.resolution = cloud.field.resolution;
    grid.info.height = cloud->height;
    grid.info.width = cloud->width;
    grid.info.map_load_time = ros::Time::now();

    //TODO find ground plane
    pcl::rotate()
    //TODO rotate data points
    
    //truncate data points not within car hieght

    for (auto& p : cloud.points){
        //is zero correct?
        if (p.z < hieght && p.z > 0){
            grid.data[p.x  + p.x * p.y] = 100;
        }
    }

    pub_.publish(grid);
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud2gridmap");
    PCloud2GMap PCloud2GMap;

    ros::spin();
}
