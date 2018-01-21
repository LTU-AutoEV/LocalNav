#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>

#define TWIST_OUTPUT "/rb_drive/rb_drive/twist_cmd"
#define DISTANCE_TO_CHECK 1 

class OGrid2Twist{
public:
    OGrid2Twist();

private:
    void OGridCallback(const nav_msgs::OccupancyGrid& grid);

    bool check_forward(const nav_msgs::OccupancyGrid& grid);
    bool check_right(const nav_msgs::OccupancyGrid& grid);
    bool check_left(const nav_msgs::OccupancyGrid& grid);

    uint32_t pos_x;
    uint32_t pos_y;
    float cells_per_meter;

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
    geometry_msgs::Twist twist;

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

   if ((grid.info.width != 0) && (grid.info.height != 0)){

    pos_x = grid.info.width / 2;
    pos_y = grid.info.height / 2;
    cells_per_meter = 1 / grid.info.resolution;

    if (check_forward(grid)){
        twist.linear.x = 2;
        twist.angular.z = 0;
    } else if (check_right(grid)){
        twist.linear.x = 2;
        twist.angular.z = -10;
    } else if (check_left(grid)){
        twist.linear.x = 2;
        twist.angular.z = 10;
    }
   }
   pub_.publish(twist);
}

bool OGrid2Twist::check_forward(const nav_msgs::OccupancyGrid& grid){
    const uint32_t scan_radius = DISTANCE_TO_CHECK / 2;

    uint32_t start_x = (scan_radius > pos_x) ? 0 : pos_x - scan_radius;
    uint32_t end_x = (scan_radius > pos_x) ? 0 : pos_x + scan_radius;
    uint32_t start_y = (scan_radius > pos_y) ? 0 : pos_y - scan_radius;
    uint32_t end_y = (scan_radius > pos_y) ? 0 : pos_y + scan_radius;

    for (auto x = start_x; x < end_x; ++x){
        for (auto y = start_y; y < end_y; ++y){
            if (grid.data[x + (grid.info.width * y)] != 0){
                return false;
            }
        }
    }

    return true;
}

bool OGrid2Twist::check_forward(const nav_msgs::OccupancyGrid& grid){
    const uint32_t scan_radius = DISTANCE_TO_CHECK / 2;

    uint32_t start_x = (scan_radius > pos_x) ? 0 : pos_x - scan_radius;
    uint32_t end_x = (scan_radius > pos_x) ? grid.info.width - 1 : pos_x + scan_radius;
    uint32_t start_y = pos_y;
    uint32_t end_y = (DISTANCE_TO_CHECK > pos_y) ? grid.info.height - 1 : pos_y + scan_radius;

    for (auto x = start_x; x <= end_x; ++x){
        for (auto y = start_y; y <= end_y; ++y){
            if (grid.data[x + (grid.info.width * y)] != 0){
                return false;
            }
        }
    }

    return true;
}

bool OGrid2Twist::check_left(const nav_msgs::OccupancyGrid& grid){
    const uint32_t scan_radius = DISTANCE_TO_CHECK / 2;

    uint32_t start_x = (DISTANCE_TO_CHECK > pos_x) ? 0 : pos_x - scan_radius;
    uint32_t end_x = pos_x;
    uint32_t start_y = (scan_radius > pos_y) ? 0 : pos_y - scan_radius;
    uint32_t end_y = (scan_radius > pos_y) ? grid.info.height - 1 : pos_y + scan_radius;

    for (auto x = start_x; x <= end_x; ++x){
        for (auto y = start_y; y <= end_y; ++y){
            if (grid.data[x + (grid.info.width * y)] != 0){
                return false;
            }
        }
    }

    return true;
}

bool OGrid2Twist::check_right(const nav_msgs::OccupancyGrid& grid){
    const uint32_t scan_radius = DISTANCE_TO_CHECK / 2;

    uint32_t start_x = (DISTANCE_TO_CHECK > pos_x) ? grid.info.width - 1 : pos_x + scan_radius;
    uint32_t end_x = pos_x;
    uint32_t start_y = (scan_radius > pos_y) ? 0 : pos_y - scan_radius;
    uint32_t end_y = (scan_radius > pos_y) ? grid.info.height - 1 : pos_y + scan_radius;

    for (auto x = start_x; x <= end_x; ++x){
        for (auto y = start_y; y <= end_y; ++y){
            if (grid.data[x + (grid.info.width * y)] != 0){
                return false;
            }
        }
    }

    return true;
}

void main(int argc, char** argv){
  ros::init(argc, argv, "Twist");
  OGrid2Twist OGrid2Twist;

  ros::spin();
}

