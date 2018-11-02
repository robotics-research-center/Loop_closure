#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <Eigen/Dense>

using namespace cv;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void display_image(const Mat& image){
	namedWindow("opencv_viewer", WINDOW_AUTOSIZE);
	imshow("opencv_viewer", image);
	waitKey(2500);
	destroyWindow("opencv_viewer");
}

void matched_pairs(const Mat& img1, const Mat& img2, vector<DMatch>& good_matches, Mat& out_image, vector<KeyPoint>& keypoints1, vector<KeyPoint>& keypoints2){
	const int count_features = 550;
    Ptr<xfeatures2d::SIFT> feature_detect = xfeatures2d::SIFT::create(count_features);
    Mat descriptors1, descriptors2;
    feature_detect->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
    feature_detect->detectAndCompute(img2, noArray(), keypoints2, descriptors2);
    
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    vector<vector<DMatch>> knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

    const float threshold_ratio = 0.7f;
    for(size_t i=0; i<knn_matches.size(); ++i){
    	if(knn_matches[i][0].distance < threshold_ratio * knn_matches[i][1].distance)
    		good_matches.push_back(knn_matches[i][0]);
    }
    drawMatches(img1, keypoints1, img2, keypoints2, good_matches, out_image);
}

void display_cloud(const PointCloudT::Ptr cloud){
	pcl::visualization::PCLVisualizer viewer("Cloud_viewer");
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
	viewer.addPointCloud<PointT>(cloud, rgb, "cloud_points");
	while(! viewer.wasStopped())
		viewer.spinOnce();
}

PointCloudT::Ptr images2cloud(const Mat& rgb_image, const Mat& depth_image,const vector<pair<int, int>> &coordinates, vector<int>& out_cloud_indexes){
	const float f = 570.3, cx = 320.0, cy = 240.0;
	PointCloudT::Ptr cloud(new PointCloudT());
	cloud->width = rgb_image.cols;
	cloud->height = rgb_image.rows;
	cloud->is_dense = false;
	float bad_point = std::numeric_limits<float>::quiet_NaN();
	int image_index = 0;
	int bad_image_index = 0;
	for(int y=0; y<rgb_image.rows; ++y){
		for(int x=0; x<rgb_image.cols; ++x){
			pcl::PointXYZRGB point;
			if(depth_image.at<unsigned short>(y, x) == 0){
				//point.x = bad_point;
				//point.y = bad_point;
				//point.z = bad_point;
				++bad_image_index;
			}
			else{
				point.z = depth_image.at<unsigned short>(y, x)/1000.0;
				point.x = (x - cx) * point.z / f;
				point.y = (y - cy) * point.z / f;
				point.r = rgb_image.at<cv::Vec3b>(y, x)[2];
				point.g = rgb_image.at<cv::Vec3b>(y, x)[1];
				point.b = rgb_image.at<cv::Vec3b>(y, x)[0];
				std::pair<int, int> element = std::make_pair(x, y);
				if(std::find(coordinates.begin(), coordinates.end(), element) != coordinates.end()){
					//cout << "Matched cloud point index: " << image_index << endl;
					out_cloud_indexes.push_back(image_index);
				}
				++image_index;
				cloud->points.push_back(point);
			}
		}
	}
	fprintf(stdout, "bad_image_index: %d\t image_index: %d\t rows: %d\t cols: %d\n", bad_image_index, image_index, rgb_image.rows, rgb_image.cols);
	return cloud;
}

void get_matched_coordinates(const vector<DMatch>& matches, const vector<KeyPoint>& kps1, const vector<KeyPoint>& kps2, 
							vector<pair<int, int>>& coordinates_img1, vector<pair<int, int>>& coordinates_img2){
	for(vector<DMatch>::const_iterator iter=matches.begin(); iter!=matches.end(); ++iter){
		/*fprintf(stdout, "[%d %d] \t[%d %d]\n", int(kps1[iter->queryIdx].pt.x), int(kps1[iter->queryIdx].pt.y), 
				int(kps2[iter->trainIdx].pt.x), int(kps2[iter->trainIdx].pt.y));*/
		coordinates_img1.push_back(make_pair(int(kps1[iter->queryIdx].pt.x), int(kps1[iter->queryIdx].pt.y)));
		coordinates_img2.push_back(make_pair(int(kps2[iter->trainIdx].pt.x), int(kps2[iter->trainIdx].pt.y)));
	}
}

Eigen::Matrix4f simple_icp(PointCloudT::Ptr source, PointCloudT::Ptr target, pcl::Correspondences& out_correspondences){
	Eigen::Matrix4f homogeneous;
	if(out_correspondences.size() == 0){
		cout << "Doing align\n";
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setInputSource(source);
		icp.setInputTarget(target);
		PointCloudT transformed;
		icp.align(transformed);
		homogeneous = icp.getFinalTransformation();
		out_correspondences = (*icp.correspondences_);
		fprintf(stdout, "Has converged?: %d\t Score: %g\n", icp.hasConverged(), icp.getFitnessScore());
	}
	else{
		cout << "Doing estimateRigidTransformation\n";
		pcl::registration::TransformationEstimationSVD<PointT, PointT> transform_estimation;
		transform_estimation.estimateRigidTransformation(*source, *target, out_correspondences, homogeneous);
	}
	transformPointCloud(*target, *target, homogeneous.inverse());
	/*for(size_t i=0; i<out_correspondences.size(); ++i){
		cout << out_correspondences[i] << endl;
	}*/
	return homogeneous;
}

void simple_visualize(const PointCloudT::Ptr cloud1, const PointCloudT::Ptr cloud2, const pcl::Correspondences& correspond={}){
	pcl::visualization::PCLVisualizer viewer("ICP");
	viewer.addCorrespondences<PointT>(cloud1, cloud2, correspond);
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb1(cloud1);
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb2(cloud2);
	viewer.addPointCloud(cloud1, rgb1, "cloud1");
	viewer.addPointCloud(cloud2, rgb2, "cloud2");
	//viewer.setBackgroundColor(255, 255, 255, 0);
	while(! viewer.wasStopped())
		viewer.spinOnce();
}

void fill_correspondences(pcl::Correspondences& out_correspondences, const vector<int>& cloud_indexes1, 
						const vector<int>& cloud_indexes2){
	for(int i=0; i<cloud_indexes1.size(); ++i){
		out_correspondences.push_back(pcl::Correspondence(cloud_indexes1[i], cloud_indexes2[i], 0.0));
	}
}

void homogeneous_to_quaternion(const Eigen::Matrix4f &homogeneous){
	Eigen::Matrix3f rotate;
	for(int i = 0; i<3 ; i++)
		for(int j = 0; j<3 ; j++ )
			rotate(i,j) = homogeneous(i,j);
	Eigen::Quaternionf q(rotate);
	
	fprintf(stdout, "\nTranslation \n%f %f %f\n", homogeneous(0,3), homogeneous(1,3), homogeneous(2,3));
	Eigen::Matrix<float, 4, 1> coeffs = q.coeffs();	
	fprintf(stdout, "Quaternion:\n%f %f %f %f\n", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
	fprintf(stdout, "g2o edge:\n%f %f 0 0 0 %f %f\n", homogeneous(0,3), homogeneous(1,3), coeffs[2], coeffs[3]);
}

int main( int argc, char* argv[] ){
    if(argc != 5){
    	fprintf(stdout, "Usage: ./feature_detection rgb1.png rgb2.png depth1.png depth2.png\n");
    	return 1;
    }
    Mat rgb1 = imread(argv[1], IMREAD_COLOR );
    Mat rgb2 = imread(argv[2], IMREAD_COLOR );
    Mat depth1 = imread(argv[3], IMREAD_ANYDEPTH);
    Mat depth2 = imread(argv[4], IMREAD_ANYDEPTH);
    if(rgb1.empty() || rgb2.empty()){
    	fprintf(stdout, "Unable to open images\n");
    	return 1;
    }
    vector<DMatch> out_matches;
    vector<KeyPoint> out_kp1, out_kp2;
    Mat out_image;
    vector<pair<int, int>> out_coord1, out_coord2;
    vector<int> cloud_indexes1, cloud_indexes2;
    matched_pairs(rgb1, rgb2, out_matches, out_image, out_kp1, out_kp2);
    display_image(out_image);
    get_matched_coordinates(out_matches, out_kp1, out_kp2, out_coord1, out_coord2);
    PointCloudT::Ptr source = images2cloud(rgb1, depth1, out_coord1, cloud_indexes1);
    PointCloudT::Ptr target = images2cloud(rgb2, depth2, out_coord2, cloud_indexes2);
    pcl::Correspondences out_correspondences;
    //fill_correspondences(out_correspondences, cloud_indexes1, cloud_indexes2);
    Eigen::Matrix4f homogeneous = simple_icp(source, target, out_correspondences);
    homogeneous_to_quaternion(homogeneous);
    simple_visualize(source, target, out_correspondences);
    return 0;
}