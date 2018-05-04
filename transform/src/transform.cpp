#include "../include/transform.hpp"


void callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const geometry_msgs::PoseStampedConstPtr& pose_msg)
{ 
  // reading in and converting points2 msg to pcl pointer  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_wrt_camera(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *pc_wrt_camera);
  std::cout << "[ INFO ] Read point cloud with size " << pc_wrt_camera->size() << std::endl;
  
  // creating a new pcl pointer wrt world
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_wrt_world(new pcl::PointCloud<pcl::PointXYZRGB>);

  // applying transformation using the VIO robot pose information
  pc_wrt_world = apply_transformation(pc_wrt_camera, pose_msg);
  std::cout << "[ INFO ] Transforming point cloud " << std::endl;

  // assigning a frame id and subsequently publishing
  pc_wrt_world->header.frame_id = "/world";
  PclPub.publish(pc_wrt_world);

  // PCD file writing code below:
  std::stringstream filePath;
  filePath << "transf_pcl_" << counter_;
  counter_++;  
  pc_writer.writeBinaryCompressed(filePath.str(), *pc_wrt_world); //Binary Compressed
  // pc_writer.writeBinary(filePath.str(), *pc_wrt_world);  //Binary format
  // pc_writer.writeASCII(filePath.str(), *pc_wrt_world);  //ASCII format
  std::cout << "[ INFO ] Saved point cloud to build folder with filename :" << filePath.str() << std::endl;
  filePath.flush();
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "transform_pcl");
  ros::NodeHandle nh;

  PclPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/transformed/points2", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/points2", 5); ROS_INFO("Listening to topic /points2 ...");
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "/svo/pose_cam/0", 5); ROS_INFO("Listening to topic /svo/pose_cam/0 ...\n");
  
  // ApproximateTime Filter takes a queue size as its constructor argument, hence MySyncPolicy(100)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, pose_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  

  ros::spin();

  return (0);
}
