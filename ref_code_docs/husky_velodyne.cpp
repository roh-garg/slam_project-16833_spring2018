#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
// #include <vector>

char** argv;
int global_counter = 0;
int drop_size = 1;
class subandpub
{
	public:
  	ros::NodeHandle nh;
  	ros::Subscriber sub;
  	ros::Publisher pub;
    ros::Publisher pub1;
  	subandpub()
  	{
    
    	sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", drop_size, &subandpub::cloud_callback, this);
    	pub = nh.advertise<std_msgs::Float64MultiArray>("obstacle_points", 1);
      pub1 = nh.advertise<sensor_msgs::PointCloud2>("obstacle_clouds", 1);
  	}
  	void drop_packets()
  	{ 
    	// std::cerr << "Droping packets!!" << std::endl;
    	return;
  	}
  	void cloud_process (const sensor_msgs::PointCloud2ConstPtr& input)
  	{
    	pcl::PCLPointCloud2 pcl_pc2;
	    pcl_conversions::toPCL(*input,pcl_pc2);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_old(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(pcl_pc2,*cloud_old);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_old (new pcl::PointCloud<pcl::PointXYZ>);
	    cloud_old->points.resize (cloud_old->width * cloud_old->height);
 
		// if (strcmp(argv[1], "-r") == 0){
		//   pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		//   // build the filter
		//   outrem.setInputCloud(cloud_old);
		//   outrem.setRadiusSearch(0.8);
		//   outrem.setMinNeighborsInRadius (2);
		//   // apply filter
		//   outrem.filter (*cloud_filtered_old);
		// }
		// else if (strcmp(argv[1], "-c") == 0){
		// build the condition
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
		pcl::ConditionAnd<pcl::PointXYZ> ());
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, 0.0)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 3.0)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -3.0)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, 3.0)));
		// build the filter
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setCondition (range_cond);
		condrem.setInputCloud (cloud_old);
		condrem.setKeepOrganized(true);
		// apply filter
		condrem.filter (*cloud_filtered_old);
		// }
		// else{
		//   std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
		//   exit(0);
		// }

		/*  pcl::PCDWriter writer;
		std::stringstream ss;
		ss << "/home/administrator/catkin_ws/obstacles/filtered_points_1" << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered_old, false); */

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
		cloud = cloud_filtered_old;

		// Create the filtering object: downsample the dataset using a leaf size of 1cm
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		vg.setInputCloud (cloud);
		vg.setLeafSize (0.01f, 0.01f, 0.01f);
		vg.filter (*cloud_filtered);
		//  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.02);

		int i=0, nr_points = (int) cloud_filtered->points.size ();
		while (cloud_filtered->points.size () > 0.3 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud_filtered);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
			extract.setNegative (false);

			// Get the points associated with the planar surface
			extract.filter (*cloud_plane);
			//    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

			// Remove the planar inliers, extract the rest
			extract.setNegative (true);
			extract.filter (*cloud_f);
			*cloud_filtered = *cloud_f;
		}
    pub1.publish(cloud_filtered);
	    // Creating the KdTree object for the search method of the extraction
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	    tree->setInputCloud (cloud_filtered);

	    std::vector<pcl::PointIndices> cluster_indices;
	    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	    ec.setClusterTolerance (0.05); // 2cm
	    // NOTE: changed below value
	    ec.setMinClusterSize (100);
	    ec.setMaxClusterSize (25000);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (cloud_filtered);
	    ec.extract (cluster_indices);

	    //int j = 0;
	    float max_x,min_x,max_y,min_y;
	    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	    {   
	      	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	      	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
	      	cloud_cluster->width = cloud_cluster->points.size ();
	      	cloud_cluster->height = 1;
	      	cloud_cluster->is_dense = true;

		    max_x = INT_MIN;
		    min_x = INT_MAX;
		    max_y = INT_MIN;
		    min_y = INT_MAX;
        for (size_t i = 0; i < cloud_cluster->points.size(); ++i)
        {	// XXXX ///
				  min_x = std::min(min_x,float(cloud->points[i].x));
				      //std::cout << cloud->points[i].x << "    ";
				  max_x = std::max(max_x,float(cloud->points[i].x));
				// YYY ///		
				  min_y = std::min(min_y,float(cloud->points[i].y));
				//std::cout << cloud->points[i].y << '\n';
				  max_y = std::max(max_y,float(cloud->points[i].y));
	    	}

/*      if(min_x > INT_MAX/100 )
      {
        for (size_t i = 0; i < cloud_cluster->points.size(); ++i)
        {	// XXXX ///
				  ROS_INFO("%lf  %lf\n",(cloud->points[i].x),(cloud->points[i].y));

    //      std::cout << cloud->points[i].x << "  " << cloud->points[i].y<< '\n';
				  //max_y = std::max(max_y,float(cloud->points[i].y));
	    	}
          
      }
*/			//std::cout << "end" << '\n';
      
      std_msgs::Float64MultiArray msg;
      std::vector<float> data_vector;

      if(int(min_x) < INT_MAX/10 && int(max_x) > INT_MIN/10)
      //if(1)
      {
			    std::cout << "Max and Min of X in cluster "  << " are " << max_x << " and " << min_x << '\n';
          std::cout << "Max and Min of Y in cluster "  << " are " << max_y << " and " << min_y << '\n';
          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
          //    vector<int> data = {min_x, max_x, min_y, max_y};
          data_vector.push_back(min_x);
          data_vector.push_back(max_x);
          data_vector.push_back(min_y);
          data_vector.push_back(max_y);
        
          // set up dimensions
          msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
          msg.layout.dim[0].size = data_vector.size();
          msg.layout.dim[0].stride = 1;
          msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

          // copy in the data
          msg.data.clear();
          msg.data.insert(msg.data.end(), data_vector.begin(), data_vector.end());


/*          std_msgs::String msg;
          std::stringstream ss;
          ss << "X"<<min_x<<" "<<max_x<<"Y"<<min_y<<" "<<max_y;
          msg.data = ss.str();
          ROS_INFO("%s", msg.data.c_str());
*/  
          // ROS_INFO("%d,%d,%d,%d", )
          pub.publish(msg);
          ROS_INFO("Published Successfully!!!");
          std::cout << std::endl;
	    
      }
      data_vector.clear();
      //std::cout << "on the way\n";
			/*std::stringstream ss;
			ss << "/home/administrator/catkin_ws/obstacles/new_cluster_" << j << ".pcd";
			writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
			j++;*/
  		}

  	}
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_old (new pcl::PointCloud<pcl::PointXYZ>);
	void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
	{   //global_counter++;
		//if(global_counter%drop_size==0)
		if(1)
    {
		  //global_counter=1;
		  cloud_process(input);
		}
		else
		{
		  drop_packets();
		}
	}
	
};
int main (int argc, char** argv)
{ 
	// if (argc != 2)
	// {
	//   std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
	//   exit(0);
	// }
	ros::init(argc, argv, "sub_pcl");
	subandpub object;
	//sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, cloud_callback);

	// sub =  

	ros::spin();

	return (0);
}


