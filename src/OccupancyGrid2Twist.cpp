#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>

#define TWIST_OUTPUT "/rb_drive/rb_drive/twist_cmd"

class OGrid2Twist{
public:
    OGrid2Twist();

private:
    void OGridCallback(const nav_msgs::OccupancyGrid& grid);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

OGrid2Twist::OGrid2Twist(){
    nh_ = ros::NodeHandle("~");

    pub_ = nh_.advertise<geometry_msgs::Twist>(TWIST_OUTPUT,100);
    sub_ = nh_.subscribe("Occupancy_", 100, &OGrid2Twist::OGridCallback, this                                                                                                                               );
}

void OGrid2Twist::OGridCallback(const nav_msgs::OccupancyGrid& grid){
    //????????
}

void main(int argc, char** argv){
  ros::init(argc, argv, "Twist");
  OGrid2Twist OGrid2Twist;

  ros::spin();
}