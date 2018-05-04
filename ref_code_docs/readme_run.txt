General Directions:
0. Start with running roscore on a separate terminal

1. Set simulation time  
$ rosparam set /use_sim_time true

Check if it has been set by:
$ rosparam get /use_sim_time

**********some rosbag commands****************

simple playback
$ rosbag play /path/to/rosbag/1.bag --clock

looping
$ rosbag play /path/to/rosbag/1.bag --clock -l

reduced play rate (0.5) + looping
$ rosbag play /path/to/rosbag/1.bag -r 0.5 -l --clock

start time + until (duration in sec)
$ rosbag play /path/to/rosbag/1.bag -s 5 -u 10 --clock


***********Launch File*************************

Use this launch file to produce pointcloud and filter the imu values
$ roslaunch stereo_img_proc.launch
output pointcloud topic /points2

Use 
$ rostopic list
to check all the published topics


***********icp_module*************************

Go to catkin_ws/build/ folder and run ../icp_module
Make sure you have a folder in catkin_ws named pcd_files to capture the resultant!!
