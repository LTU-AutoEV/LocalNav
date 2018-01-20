#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <grid_map_msgs/GridMap.h>

class PCloud2GMap{

public:
    PCloud2GMap();

private:
    void CloudCallback(const sensor_msgs::PointCloud2& cloud); 

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

PCloud2GMap::PCloud2GMap(){
    nh_ = ros::NodeHandle("~");

    pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 100);
    sub_ = nh_.subscribe("/velodyne_points", 100, &PCloud2GMap::CloudCallback, this);
}

void PCloud2GMap::CloudCallback(const sensor_msgs::PointCloud2& cloud){

} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PointCloud2GridMap");
  PCloud2GMap PCloud2GMap;

  ros::spin();
}
