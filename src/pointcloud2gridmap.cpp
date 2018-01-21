#include <iostream>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_pcl/grid_map_pcl.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <cmath>

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
    void CloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &);

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
    pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 100);
    // subscribe to the pointcloud2 data comming from the lidar
    sub_ = nh_.subscribe("/velodyne_points", 100, &PCloud2GMap::CloudCallback, this);
}

void
PCloud2GMap::CloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

  
  
    // Normal estimation*
  //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  //pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZ>);
  //tree3->setInputCloud (cloud);
  //n.setInputCloud (cloud);
  //n.setSearchMethod (tree3);
  //n.setKSearch (2);
  //n.compute (*normals);
  ///[> normals should not contain the point normals + surface curvatures

  //// Concatenate the XYZ and normal fields*
  //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  //pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals
 


  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals(true);

  // Set parameters
  mls.setInputCloud(cloud);
  mls.setPolynomialFit(false);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.04);

  // Reconstruct
  mls.process(*mls_points);


    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    //tree2->setInputCloud(cloud_with_normals);
    tree2->setInputCloud(mls_points);
    
    pcl::PolygonMesh triangles;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setSearchRadius(.04);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI/4);
    gp3.setMinimumAngle(M_PI/18);
    gp3.setMaximumAngle(2*M_PI/3);
    gp3.setNormalConsistency(false);
    //gp3.setInputCloud(cloud_with_normals);
    gp3.setInputCloud(mls_points);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    ROS_ERROR_STREAM(triangles.cloud.data[0]);
    for (auto d : triangles.polygons)
        ROS_ERROR_STREAM(d);

    grid_map::GridMap map;
    grid_map::GridMapPclConverter::initializeFromPolygonMesh(triangles, 0.01, map);
    grid_map::GridMapPclConverter::addLayerFromPolygonMesh(triangles, "elevation", map);
    map.setFrameId("my_frame");

    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map, msg);
    pub_.publish(msg);
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud2gridmap");
    PCloud2GMap PCloud2GMap;

    ros::spin();
}
