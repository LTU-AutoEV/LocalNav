#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>

/* Occupancy2Object
 *
 * Convert grid map ro cost map
 *
 */
class Occupancy2Object
{
public:
    Occupancy2Object();

private:
    // Callbacks
    void gmCallback(const nav_msgs::OccupancyGrid& grid);

    // ROS objects
    ros::NodeHandle nh_;
    ros::Subscriber gm_sub_;
    ros::Publisher pub_;
};

// Constructor
//   Set up publisher and subscriber
Occupancy2Object::Occupancy2Object()
    :nh_("~")
{

    gm_sub_  = nh_.subscribe("~/gridmap", 10, &Occupancy2Object::gmCallback, this);
    pub_ = nh_.advertise<geometry_msgs::Point>("~/obstacle_loc", 10);
}

// Callback function for the points
void Occupancy2Object::gmCallback(const nav_msgs::OccupancyGrid& grid)
{
    //If an object is with this field stop the car
    //values in meters
    const float search_witdth = 2.0f;

    int mid_y = grid.info.height/2;
    int mid_x = grid.info.width/2;

    float res = grid.info.resolution;

    //search reagion centered on the car
    int search_width_in_cells = std::ceil(search_witdth * res);
    
    //make sure the search region is not bigger than the search space
    if (search_width_in_cells > grid.info.width){
        search_width_in_cells = grid.info.width;
    }


    geometry_msgs::Point closest;
    closest.x = INT_MAX; //default for no obstacle
    closest.y = 0;

    for (auto i = 0; i <= mid_y; ++i){
        for (auto j = 0; j < search_width_in_cells; ++j){
            int offset = mid_x - search_width_in_cells/2;
            if (grid.data[(offset + j) + (mid_y - i)*grid.info.width] > 0){
                //x and y are swapped
                closest.x = i * res;
                closest.y = j * res;
            }
        }

        if (closest.x != INT_MAX){
            break;
        }
    }

    pub_.publish(closest);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Occupancy2Object");
  Occupancy2Object Occupancy2Object;

  ros::spin();
}

