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

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

#define SubscriberTopic "points2"
#define PublisherTopic "fused_points"
#define PublisherTopic1 "obstacle_points1"
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
	ros::Subscriber sub;
	ros::Publisher pub;
	ros::Publisher pub1;
	
	subandpub()
    {
  		GlobalTransform = Eigen::Matrix4f::Identity ();
        fusedPC = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        source = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr fusedPC = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		point_cloud_counter = 0;
		sub = handle.subscribe<sensor_msgs::PointCloud2>(SubscriberTopic, DROP_SIZE, &subandpub::callback_function, this);
        	pub = handle.advertise<sensor_msgs::PointCloud2>(PublisherTopic, 1);
      		pub1 = handle.advertise<sensor_msgs::PointCloud2>(PublisherTopic1, 1);
	}
	void callback_function (const sensor_msgs::PointCloud2ConstPtr& input)
    {   

    	//siddhanj: need to convert ROS message to PCL point cloud
    	//calling that point cloud inPointCloud

    	//siddhanj: I formally register a protest against this code. There is no Point!
    	//But Pulkit's lab, Pulkit's rulezzzzzz
    	cloud_register(input);
	}
	void cloud_register (const sensor_msgs::PointCloud2ConstPtr& input)
	{
		PointCloud::Ptr result (new PointCloud),  target(new PointCloud);

		pcl::PCLPointCloud2 inPCLPointCloud2;
		PointCloud::Ptr inPointCloud (new PointCloud);
		pcl_conversions::toPCL(*input,inPCLPointCloud2);
		pcl::fromPCLPointCloud2(inPCLPointCloud2,*target);

		if (point_cloud_counter==0)
		{
			pcl::fromPCLPointCloud2(inPCLPointCloud2,*source);
			point_cloud_counter++;
			return;
		}
		
        Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity ();

	    
	    PointCloud::Ptr temp (new PointCloud);
	    
	    pairAlign (source, target, temp, pairTransform, true);

	    //transform current pair into the global transform
	    pcl::transformPointCloud (*temp, *result, GlobalTransform);
	    *fusedPC += *result;
	    //update the global transform
	    GlobalTransform = GlobalTransform * pairTransform;

/*        
	    pcl::VoxelGrid<PointT> grid;
	    grid.setLeafSize (0.000001, 0.000001, 0.000001);
	    grid.setInputCloud (fusedPC);
	    grid.filter (*fusedPC);	
  */      
	    point_cloud_counter++;

	    pcl::fromPCLPointCloud2(inPCLPointCloud2,*source);
	    //siddhant: Writing PCs to file right now. Will eventually publish them as a message.
        if(point_cloud_counter%10==0)
        {
	        std::stringstream ss;
    	    ss <<"../pcd_files/"<<point_cloud_counter << ".pcd";
    	    pcl::io::savePCDFile (ss.str (), *fusedPC, true);
        }
	}
	////////////////////////////////////////////////////////////////////////////////
	/** \brief Align a pair of PointCloud datasets and return the result
	  * \param cloud_src the source PointCloud
	  * \param cloud_tgt the target PointCloud
	  * \param output the resultant aligned source PointCloud
	  * \param final_transform the resultant transform between source and target
	  */
	void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
	{
	  //
	  // Downsample for consistency and speed
	  // \note enable this for large datasets
	  PointCloud::Ptr src (new PointCloud);
	  PointCloud::Ptr tgt (new PointCloud);
	  pcl::VoxelGrid<PointT> grid;
	  if (downsample)
	  {
	    grid.setLeafSize (0.01, 0.01, 0.01);
	    grid.setInputCloud (cloud_src);
	    grid.filter (*src);

	    grid.setInputCloud (cloud_tgt);
	    grid.filter (*tgt);
	  }
	  else
	  {
	    src = cloud_src;
	    tgt = cloud_tgt;
	  }


	  // Compute surface normals and curvature
	  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
	  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

	  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	  norm_est.setSearchMethod (tree);
	  norm_est.setKSearch (16);
	  
	  norm_est.setInputCloud (src);
	  norm_est.compute (*points_with_normals_src);
	  pcl::copyPointCloud (*src, *points_with_normals_src);

	  norm_est.setInputCloud (tgt);
	  norm_est.compute (*points_with_normals_tgt);
	  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

	  //
	  // Instantiate our custom point representation (defined above) ...
	  MyPointRepresentation point_representation;
	  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	  point_representation.setRescaleValues (alpha);

	  //
	  // Align
	  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	  reg.setTransformationEpsilon (1e-6);
	  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
	  // Note: adjust this based on the size of your datasets
	  reg.setMaxCorrespondenceDistance (1);  
	  // Set the point representation
	  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	  reg.setInputSource (points_with_normals_src);
	  reg.setInputTarget (points_with_normals_tgt);



	  //
	  // Run the same optimization in a loop and visualize the results
	  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	  reg.setMaximumIterations (2);
	  for (int i = 0; i < 30; ++i)
	  {
		    PCL_INFO ("Iteration Nr. %d.\n", i);

		    // save cloud for visualization purpose
		    points_with_normals_src = reg_result;

		    // Estimate
		    reg.setInputSource (points_with_normals_src);
		    reg.align (*reg_result);

				//accumulate transformation between each Iteration
		    Ti = reg.getFinalTransformation () * Ti;

				//if the difference between this transformation and the previous one
				//is smaller than the threshold, refine the process by reducing
				//the maximal correspondence distance
		    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
		      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
		    
		    prev = reg.getLastIncrementalTransformation ();
	  }

		//
	  // Get the transformation from target to source
	  targetToSource = Ti.inverse();

	  //
	  // Transform target back in source frame
	  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

	  //p->removePointCloud ("source");
	  //p->removePointCloud ("target");

	  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
	  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);

	  *output += *cloud_src;
	  
	  final_transform = targetToSource;

	  if(typeid(cloud_src) != typeid(output))
	  {
	    cout << "source type id: " << typeid(cloud_src).name() << endl;
	    cout << "output type id: " << typeid(output).name() << endl;
	  }
	}
};

int main (int argc, char** argv)
{
        ros::init(argc, argv, "sub_pcl");
        subandpub object;
        ros::spin();

        return (0);

}
