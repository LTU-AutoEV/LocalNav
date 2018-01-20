#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <grid_map_msgs/GridMap.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <grid_map_pcl/grid_map_pcl.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

/*PCloud2GMap 
 *
 * Converts the PointCloud2 data from the velodyne Lidar
 *  into a grid_map
 *
 * Publish:     grid_map on "~/grid_map"
 *
 * Subscrivbe:  PointCloud2 on "/velodyne_points"
 */
class PCloud2GMap{

public:
    PCloud2GMap();

private:
    //Callback
    void CloudCallback(const sensor_msgs::PointCloud2& cloud); 

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

//constructor
//  set up publisher and subscriber
PCloud2GMap::PCloud2GMap(){
    nh_ = ros::NodeHandle("~");

    //publish on /grid_map
    pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 100);
    //subscribe to the pointcloud2 data comming from the lidar
    sub_ = nh_.subscribe("/velodyne_points", 100, &PCloud2GMap::CloudCallback, this);
}

void PCloud2GMap::CloudCallback(const sensor_msgs::PointCloud2& cloud){
   pcl::PolygonMesh poly_mesh;
   pcl_conversions::toPCL(cloud, poly_mesh.cloud);

   grid_map::GridMap map;
   grid_map::GridMapPclConverter::initializeFromPolygonMesh(poly_mesh, 0.05, map);

    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map, msg);
    pub_.publish(msg);

} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PointCloud2GridMap");
  PCloud2GMap PCloud2GMap;

  ros::spin();
}
