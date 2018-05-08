# Documentation of the Lidar Code

## Overview

The Lidar code requires the pcl includes to complile and the velodyne driver package to complile. 

The lidar code can be broken into 3 sections: Conversion, Transformation, and Searching.

# Convertion

The first step in the lidar code is to convert from the point cloud message to the point cloud object.

This involves two convertions: one from the message to the pcl pointcloud and another from the pcl pointcloud to the ROS pointcloud.

~~~ C++
    using actor_cloud_t = pcl::PointCloud<pcl::PointXYZ>;
    using actor_cloud_ptr_t = actor_cloud_t::Ptr;
~~~

~~~ c++
    pcl::PCLPointCloud2 pcl_pc2;
    //convert from msg to pcl
    pcl_conversions::toPCL(*msg,pcl_pc2);

    //create input cloud
    actor_cloud_ptr_t cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    //convert from pcl to ros
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
~~~

Note: actor_cloud_ptr_t is an alias for pcl::PointCloud< pcl::PointXYZ>::Ptr;

## Transformation

This step transforms the datapoints from the lidars referece to the referece of the ground.
This is done because the lidar does not sit perfectly flat on the top of the vehicle.

Note: it appears the Lidar is able to tell its orientation ( accelerometer maybe? ), so this partion of the code is currently not being used.  

The transformation starts by segmenting the data and calculating the ground plane.

~~~ C++
    //find ground plane (http://pointclouds.org/documentation/tutorials/planar_segmentation.php)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (in);
    seg.segment(*inliers, *coefficients);
~~~

From there, we can create the transformation and angle for the rotation.

~~~ C++
    Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector;

    //The normal vector for the base plane (floor plane)
    //generated from the segmentation
    floor_plane_normal_vector[0] = coefficients->values[0];
    floor_plane_normal_vector[1] = coefficients->values[1];
    floor_plane_normal_vector[2] = coefficients->values[2];

    //NORMAL of the XY plane <0,0,1> (aka the Z axis)
    xy_plane_normal_vector[0] = 0.0;
    xy_plane_normal_vector[1] = 0.0;
    xy_plane_normal_vector[2] = 1.0;

    
    Eigen::Vector3f rotation_vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
    float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));
~~~

With the tranformation and angle known, they can now be applied to the pointcloud object.

~~~ C++
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate (Eigen::AngleAxisf (theta, rotation_vector));
    pcl::transformPointCloud (*in, *out, transform_2);
~~~

## Searching

The filal part of the code searched a rectaular prism region for points.
If points are found inside this region the closest point is published, otherwise the point <INT_MAX, 0, 0> is published.

~~~ C++
    geometry_msgs::PointStamped closest_point;

    closest_point.header = msg->header;
    closest_point.point.x = INT_MAX;

    double dist = INT_MAX;

    for (auto p : *cloud){
        if (WithinThreshold(p.x, p.y, p.z)){
            double dist2 =  sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
            if (dist > dist2){
                dist = dist2;
                closest_point.point.x = p.x;
                closest_point.point.y = p.y;
                closest_point.point.z = p.z;
            }
        }
    }

    pub_.publish(closest_point);
~~~