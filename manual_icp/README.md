## SIFT_ICP ##

1. **sift_icp.cpp** : 
	1. *Corresponding matching pixels* are calculated using sift. 
	2. Then all image pixels are converted into 3d Points of a PointCloud. While *Corresponding matching pixels* are converted to *Corresponding matching 3D points* for ICP. 

2. *Download 180 degree image pairs data:* from [here](https://drive.google.com/open?id=1xrk_atOlpIyw5gHuMrkI5OInpQ6xIvML)

3. **180 Degree image pairs:**
	1. 70 111
	2. 79 119
	3. 85 119
	4. 119 157
	5. 119 161

4. *Coordinate convention used in RTABMAP/g2o/ROS and PCL/ICP*: [REP 103](http://www.ros.org/reps/rep-0103.html)
	1. Forward: x (Red)
	2. Left: y (Green)
	3. Upward: z (Blue)