/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <dirent.h>
#include <typeinfo>
#include <string.h>

#include "std_msgs/Float64MultiArray.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <geometry_msgs/PoseStamped.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

#define PclSubscriberTopic "points2"
#define PoseSubscriberTopic "points2"
#define PclPublisherTopic "fused_points"
#define DROP_SIZE 0

#define __DEBUG


//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


class subandpub
{	


	uint point_cloud_counter=0;	
	PointCloud::Ptr fusedPC;
    PointCloud::Ptr source;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr fusedPC = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  	Eigen::Matrix4f GlobalTransform;
	public:
	ros::NodeHandle handle;

	ros::Subscriber PclSub;
	ros::Subscriber PoseSub;
	
    ros::Publisher PclPub;
	
	subandpub()
    {
  		GlobalTransform = Eigen::Matrix4f::Identity ();
        fusedPC = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        source = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

		point_cloud_counter = 0;
	
        message_filters::Subscriber<sensor_msgs::PointCloud2> PclSub(handle, PclSubscriberTopic, DROP_SIZE); ROS_INFO("Listening to Pointcloud ...");
        message_filters::Subscriber<geometry_msgs::PoseStamped> PoseSub(handle, PoseSubscriberTopic, DROP_SIZE); ROS_INFO("Listening to Pose Data ...\n");
        
        // ApproximateTime
        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(100)
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), PclSub, PoseSub);
        sync.registerCallback(boost::bind(&subandpub::CallbackFunction, this, _1, _2));
        
        PclPub = handle.advertise<sensor_msgs::PointCloud2>(PclPublisherTopic, 1);
	
    }
	void CallbackFunction (const sensor_msgs::PointCloud2ConstPtr& pcl_input, const geometry_msgs::PoseStampedConstPtr& pose_input)
    {   

    	//siddhanj: need to convert ROS message to PCL point cloud
    	//calling that point cloud inPointCloud

    	//siddhanj: I formally register a protest against this code. There is no Point!
    	//But Pulkit's lab, Pulkit's rulezzzzzz
    	//cloud_register(input);
	}
};

int main (int argc, char** argv)
{
        ros::init(argc, argv, "sub_pcl");
        subandpub object;
        ros::spin();

        return (0);

}
