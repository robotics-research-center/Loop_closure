##Description##

1. ROS package for extracting data which is compatible to run with slam6d.
2. *Output*: The pcd_sync node outputs following to `~/Desktop/data` directory: 
	1. scanXXX.pose
	2. scanXXX.3d
3. *Running:* `rosrun pcd_extract pcd_sync in_cloud:=/camera/depth/color/points in_odom:=/raw_odom`