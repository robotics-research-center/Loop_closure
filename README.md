## Description: ##

1. **feature_detection.cpp** : Gives the R-T and the loop closing edge for g2o file, RGB and Depth images are used to calculate SIFT features and the correspondences. Then the depth image is converted into PointCloud which is given to the ICP along with correspondences.  
	*Usage:* ./feature_detection rgb1.png rgb2.png depth1.png depth2.png  
	*Download data:* from [here](https://drive.google.com/drive/folders/1AmbcwgK6rQJtM-TxdzpmFXIA18UK4Pl3?usp=sharing)