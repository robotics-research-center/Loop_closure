#!/bin/bash
echo "Hello!!! Launching roscore"
roscore &
echo "Launching realsense"
sleep 5
roslaunch realsense2_camera rs_camera.launch align_depth:=true & 
echo "Recording to bag file"
sleep 8
rosbag record -O /home/udit/data1 /camera/aligned_depth_to_color/image_raw /camera/color/image_raw/compressed /camera/color/camera_info /tf_static 