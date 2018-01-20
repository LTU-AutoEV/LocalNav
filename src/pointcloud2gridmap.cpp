#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include "grid_map_core/GridMap.hpp"

class PCloud2GMap{
public:
PCloud2GMap();

private:
void CloudCallback(const sensor_msgs::PointCloud2& cloud); 

ros::NodeHandle nh_;
ros::Publisher pub_:
ros::Subscriber sub_;
};

PCloud2GMap::PCloud2GMap(){

}

void PCloud2GMap::CloudCallback(const sensor_msgs::PointCloud2& cloud){

} 

int main()
{


}
