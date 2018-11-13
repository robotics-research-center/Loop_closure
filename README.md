## Description: ##

1. **manual_icp/feature_detection.cpp** : Gives the R-T and the loop closing edge for g2o file, RGB and Depth images are used to calculate SIFT features and the correspondences. Then the depth image is converted into PointCloud which is given to the ICP along with correspondences.  
	1. *Usage:* ./feature_detection rgb1.png rgb2.png depth1.png depth2.png  
	2. *Download data:* from [here](https://drive.google.com/drive/folders/1AmbcwgK6rQJtM-TxdzpmFXIA18UK4Pl3?usp=sharing)

2. **Reading Resources** :
	1. [ICP](http://pointclouds.org/documentation/tutorials/iterative_closest_point.php)
	2. [Concatenation of 2 cloud](http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration)
	3. [Depth2pointcloud](https://github.com/ZJULiXiaoyang/depth2pointCloud)
	4. [OpenCV feature detection](https://docs.opencv.org/4.0.0-beta/d5/d6f/tutorial_feature_flann_matcher.html)
	5. [Manual correspondences function](http://docs.pointclouds.org/trunk/classpcl_1_1registration_1_1_transformation_estimation_s_v_d.html#ac2e675e113bd1762962c36618456bee3)
	6. [Pixel coordinates from key point](https://stackoverflow.com/questions/30716610/how-to-get-pixel-coordinates-from-feature-matching-in-opencv-python)

3. **cloud_assemble/cloud_assemble.cpp** : Concatanates the PointClouds on the basis of optimized g2o file. Load the first PointCloud and add it to the global map as it is, then rest PointClouds are individually transformed according to the optimized edge constraints in "*EDGE_SE3:QUAT*" and added to global map to form a single pcd file.  
	1. *Usage:* ./cloud_assemble optimized.g2o /path/to/clouds  
	2. *Download data:* from the above mentioned database download link  

4. **rtabmap_configuration/rtabmap.ini** : Contains the changeable parameters in RTABMap.  
	1. *Changed Parameters:*   
		1. 	param name="Reg/Force3Dof" value="true"
		2.	param name="Optimizer/Slam2D" value="true"
		3.	param name="Optimizer/Strategy" value="0"
		4.	param name="Optimizer/VarianceIgnored" value="0"