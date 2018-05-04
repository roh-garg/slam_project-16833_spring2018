#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <ros/ros.h>
#include <iostream>
#include <deque>
#include <cmath>
#include <climits>
#include <fstream>
#include <sys/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>

// global variable definitions
// std::stringstream filePath;
int counter_ = 0;
ros::Publisher PclPub;
pcl::PCDWriter pc_writer;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr apply_transformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                                                                       const geometry_msgs::PoseStampedConstPtr& tf_pose) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_return(new pcl::PointCloud<pcl::PointXYZRGB>);

  Eigen::Matrix3f R_world_to_cam;
  Eigen::Matrix3f R_cam_to_world;
  Eigen::Vector3f point_T;
  pcl::PointXYZRGB point_xyzrgb;

  Eigen::Quaternionf pose_q(tf_pose->pose.orientation.w,
                            tf_pose->pose.orientation.x,
                            tf_pose->pose.orientation.y,
                            tf_pose->pose.orientation.z);
  R_world_to_cam = pose_q.normalized().toRotationMatrix();

  for (std::size_t search_index = 0; search_index < cloud_in->size(); search_index++) {
    Eigen::Vector3f point(cloud_in->points[search_index].x,
          cloud_in->points[search_index].y,
          cloud_in->points[search_index].z);

    // inverting the rotation matrix
    R_cam_to_world = R_world_to_cam.transpose();

    // rotating the point
    point_T = R_cam_to_world * point;

    // changing origin from robot frame to world frame
    point_T(0) += tf_pose->pose.position.x; 
    point_T(1) += tf_pose->pose.position.y;
    point_T(2) += tf_pose->pose.position.z;

    point_xyzrgb.x = point_T(0);
    point_xyzrgb.y = point_T(1);
    point_xyzrgb.z = point_T(2);
    point_xyzrgb.r = cloud_in->points[search_index].r;
    point_xyzrgb.g = cloud_in->points[search_index].g;
    point_xyzrgb.b = cloud_in->points[search_index].b;

    cloud_to_return->points.push_back(point_xyzrgb);
  }
  cloud_to_return->width = (uint32_t) cloud_to_return->points.size(); cloud_to_return->height = 1;

  return cloud_to_return;
}

#endif