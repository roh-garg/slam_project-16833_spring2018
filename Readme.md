# Introduction
The code builds a map using pose information from SVO odometry and does fusion and finer pointcloud alignment using ICP.

# Installation
1. Prerequisite Libraries
	* [point cloud library with version 1.5](http://pointclouds.org/).
	* [pcl_ros](http://wiki.ros.org/pcl)
	* [stereo_img_proc](http://wiki.ros.org/stereo_image_proc)
	* imu_tools
		* The imu_tools package is same as the default one available for ROS. Except that it is modified to subscribe to /epson_g364/imu topic.

2. Build 
* There are two folders: icp and transform. Please build them separately.
```
cd /path/to/icp/ OR cd /path/to/transform
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make
```

# Run Instruction
## transform part
1. roslaunch launch/stereo_img_proc.launch (will launch `stereo_image_proc` and `imu_filter_madgwick` nodes with a custom parameter configuration). Although, madgwik filter is no longer needed as SVO takes in raw imu data. It's still there in the launch file as an option. 
2. roslaunch svo launch file (eg. rgrg_stereo.launch)
3. Play rosbag. The rosbag requires the following topics:  
	`/narrow_stereo/left/image_raw`  
	`/narrow_stereo/left/camera_info`  
	`/narrow_stereo/right/image_raw`  
	`/narrow_stereo/right/camera_info`  
	`/epson_g364/imu`
4. cd /path/to/transform/build
5. ./transform
6. transform needs /points2 and /epson_g364/imu to register the point clouds. It publishes them to /transform/points2 as well as writes each pointcloud to a pcd within the build folder.

##ICP part
* Use the cpp to fuse all the transformed pcd files to a single pcd.

##LZ evaluation part
* Use the landing zone evaluation code to post process the fused map pcd file.
  

##More on the Madgwik Filter (although it is not being used right now)
* Put the folder imu_tools inside ros_ws/src
```
mv imu_tools ~/ros_ws/src/
cd ~/ros_ws
catkin_make
```

* If you want to run the filter separately (and not as part of the launch file bundled in this codebase)
```
rosrun imu_madgwik_filter imu_filter_node
```
* Please note: Do not loop the rosbag while keeping the filter running. 
* Everytime you run a rosbag, the filter needs to be started afresh. It has got to do with how the filter initializes and converges.


#SVO Tips:
You can copy the custom files in the custom_svo_files folder into the respective folders of your built svo workspaces.

##Folder paths

1. calib/
	* /path/to/svo_install_overlay_ws/src/rpg_svo_example/svo_ros
	* This is the folder that is supposed to contain the calibration file of the cameras/imu used to collect data.
	* Custom Files:
		* rgrg_stereo.yaml --> the file you will find by default in calib/ is from the 29April2018_calib_1280x1024 folder.
		* Use that for the 29April2018 data.

2. scripts/
	* /path/to/svo_install_overlay_ws/src/rpg_svo_example/svo_ros
	* This is the folder that converts the kalibr format calibration files into a format that SVO uses. Use the py script to do the conversion and copy the output to the calib folder. It's already been done in this case.


3. launch/
	* /path/to/svo_install_overlay_ws/src/rpg_svo_example/svo_ros
	* This is the folder that has the launch file used to run the algorithm. 
	* Custom files:
		* rgrg_stereo.launch


## How to run SVO
1. Follow the installation instructions in the install.pdf you will find after extracting svo_binaries_1604_kinetic.zip.
2. After you have setup the two workspaces, use the custom folders as listed above to use the calibration and launch files.

3. After you have put the files in their respective folders.
4. Start the terminal:
```
cd /path/to/svo_install_overlay_ws/src/rpg_svo_example/svo_ros/launch
roslaunch rgrg_stereo.launch
```

5. This should launch RVIZ and an SVO console that displays the number of tracked features. Then play the rosbag from the data folder.
``` 
rosparam set /use_sim_time true
rosbag play 5_full.bag -r 0.5 -u 9 --clock
```


###Some rosbag commands that might be helpful
1. Recording relevant data published by SVO and the raw bag file:
```
rosbag record /narrow_stereo/left/image_raw /epson_g364/imu /points2 /svo/pose_cam/0 /svo/pose_imu /svo/points /svo/loop_closures /initialpose /tf /tf_static
```

2. playing a rosbag from a pcd file at a rate of 1Hz publishing to the odom frame:
```
rosrun pcl_ros pcd_to_pointcloud iter30_80.pcd 1.0 _frame_id:=/odom
```
* ref: http://wiki.ros.org/pcl_ros#pointcloud_to_pcd 


# Contact
Rohit Garg
rg1@andrew.cmu.edu