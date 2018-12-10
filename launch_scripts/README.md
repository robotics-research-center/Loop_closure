## Description ##

1. **realsense_launch_record.sh**: Launch realsense and record following topics to a bagfile:
	1. /camera/aligned_depth_to_color/image_raw 
	2. /camera/color/image_raw/compressed
	3. /camera/color/camera_info 
	4. /tf_static

2. **export_rgb.launch**: Playback bag file and export raw images to `~/.ros`

3. **rtabmap.ini**: Contains changeable parameters for RTABMAP.
	1. *Changed Parameters:*  
		1. 	param name="Reg/Force3Dof" value="true"  
		2.	param name="Optimizer/Slam2D" value="true"  
		3.	param name="Optimizer/Strategy" value="0"  
		4.	param name="Optimizer/VarianceIgnored" value="true"