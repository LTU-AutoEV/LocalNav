#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>

#define TWIST_OUTPUT "/rb_drive/rb_drive/twist_cmd"

class OGrid2Twist{
public:
    OGrid2Twist();

private:
    void OGridCallback(const nav_msgs::OccupancyGrid& grid);

    bool check_forward();
    bool check_right();
    bool check_left();

    uint32_t pos_x;
    uint32_t pos_y;

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
   
   if ((grid.info.width == 0) || (grid.info.height == 0)){
       return;
   }

   pos_x = grid.info.width / 2;
   pos_y = grid.info.height / 2;

   if (check_forward()){

   } else if (check_right()){

   } else if (check_left()){

   } else {
       
   }

}

void main(int argc, char** argv){
  ros::init(argc, argv, "Twist");
  OGrid2Twist OGrid2Twist;

  ros::spin();
}