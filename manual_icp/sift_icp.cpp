#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/registration_visualizer.h>

using namespace cv;
using namespace std;

#define PI 3.1415
#define rad2deg(num)(num * 180/PI)

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ImageOperations{
private:
	Mat &rgb1, &rgb2, &depth1, &depth2;
	vector<KeyPoint> keypoints1, keypoints2;
	vector<DMatch> matches;
	Mat matched_image;
	vector<pair<int, int>>& kps1_coord;
	vector<pair<int, int>>& kps2_coord;	

private:
	void display_corresponding_image(void){
		namedWindow("opencv_viewer", WINDOW_AUTOSIZE);
		imshow("opencv_viewer", matched_image);
		waitKey(0);
		destroyWindow("opencv_viewer");
	}

	void match_keypoints(void){
		const int count_features = 500;
		Ptr<xfeatures2d::SIFT> feature_detect = xfeatures2d::SIFT::create(count_features);
		Mat descriptors1, descriptors2;

		feature_detect->detectAndCompute(rgb1, noArray(), keypoints1, descriptors1);
		feature_detect->detectAndCompute(rgb2, noArray(), keypoints2, descriptors2);
		
		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
		vector<vector<DMatch>> knn_matches;
		matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
		
		const float threshold_ratio = 0.7f;
		for(size_t i=0; i<knn_matches.size(); ++i){
			if(knn_matches[i][0].distance < threshold_ratio * knn_matches[i][1].distance)
				matches.push_back(knn_matches[i][0]);
		}
		drawMatches(rgb1, keypoints1, rgb2, keypoints2, matches, matched_image);
	}

	void kps_to_pixel_coordinates(void){
		for(vector<DMatch>::const_iterator iter=matches.begin(); iter!=matches.end(); ++iter){
			kps1_coord.push_back(make_pair(int(keypoints1[iter->queryIdx].pt.x), int(keypoints1[iter->queryIdx].pt.y)));
			kps2_coord.push_back(make_pair(int(keypoints2[iter->trainIdx].pt.x), int(keypoints2[iter->trainIdx].pt.y)));
		}
	}

public:
	ImageOperations(Mat& arg_rgb1, Mat& arg_rgb2, Mat& arg_depth1, Mat& arg_depth2,
					vector<pair<int, int>>& arg_kps1_coord, vector<pair<int, int>>& arg_kps2_coord):
					rgb1{arg_rgb1},
					rgb2{arg_rgb2},
					depth1{arg_depth1},
					depth2{arg_depth2},
					kps1_coord{arg_kps1_coord},
					kps2_coord{arg_kps2_coord}{};

	void start_processing(void){
		match_keypoints();
		kps_to_pixel_coordinates();
		display_corresponding_image();
	}
};

class CloudOperations{
private:
	Mat &rgb1, &rgb2, &depth1, &depth2;
	vector<pair<int, int>>& kps1_coord;
	vector<pair<int, int>>& kps2_coord;
	vector<int> cloud_indexes1, cloud_indexes2;
	PointCloudT::Ptr source, target;
	pcl::Correspondences correspondences;
	Eigen::Matrix4f homogeneous;
	vector<int> cloud1_keypoints;
	vector<int> cloud2_keypoints;

private:
	PointCloudT::Ptr images2cloud(const Mat& rgb_image, const Mat& depth_image, const vector<pair<int, int>> &coordinates, 
										vector<int>& out_cloud_indexes, vector<int>& cloud_keypoints){
		const float f = 570.3, cx = 320.0, cy = 240.0;
		PointCloudT::Ptr cloud(new PointCloudT());
		cloud->is_dense = false;
		float bad_point = std::numeric_limits<float>::quiet_NaN();
		int image_index = 0;
		int bad_image_index = 0;
		for(int y=0; y<rgb_image.rows; ++y){
			for(int x=0; x<rgb_image.cols; ++x){
				pcl::PointXYZRGB point;
				if(depth_image.at<unsigned short>(y, x) == 0){
					++bad_image_index;
				}
				else{
					point.z = depth_image.at<unsigned short>(y, x)/1000.0;
					point.x = (x - cx) * point.z / f;
					point.y = (y - cy) * point.z / f;
					
					float temp_z = point.z; 
					float temp_x = point.x;
					float temp_y = point.y;
					point.x = temp_z;
					point.z = -temp_y;
					point.y = -temp_x;

					point.r = rgb_image.at<cv::Vec3b>(y, x)[2];
					point.g = rgb_image.at<cv::Vec3b>(y, x)[1];
					point.b = rgb_image.at<cv::Vec3b>(y, x)[0];
					std::pair<int, int> element = std::make_pair(x, y);
					auto iter = std::find(coordinates.begin(), coordinates.end(), element);
					if(iter != coordinates.end()){
						out_cloud_indexes.push_back(image_index);
						int element_index = distance(coordinates.begin(), iter);
						cloud_keypoints.push_back(element_index);
					}
					++image_index;
					cloud->points.push_back(point);
				}
			}
		}
		cloud->width = cloud->points.size();
		cloud->height = 1;
		return cloud;
	}

	void print_cloud_keypoints(const vector<int>& cloud_indexes, const vector<int>& cloud_keypoints){
		fprintf(stdout, "size cloud_indexes: %lu\tsize cloud_keypoints: %lu\n", cloud_indexes.size(), cloud_keypoints.size());
		for(auto element : cloud_keypoints){
			cout << element << "\t" << endl;
		}
	}

	void fill_correspondences(void){
		for(int index1=0; index1<cloud1_keypoints.size(); ++index1){
			auto iter = std::find(cloud2_keypoints.begin(), cloud2_keypoints.end(), cloud1_keypoints[index1]);
			if(iter != cloud2_keypoints.end()){
				int index2 = distance(cloud2_keypoints.begin(), iter);
				correspondences.push_back(pcl::Correspondence(cloud_indexes1[index1], cloud_indexes2[index2], 0.0));
			}
		}
	}

	void print_correspondences(void){
		cout << "Size of correspondences: " << correspondences.size() << endl;
		for(size_t i=0; i<correspondences.size(); ++i){
			cout << correspondences[i] << endl;
		}
	}

	void translate_cloud(const Eigen::Matrix4f& transform){
		transformPointCloud(*target, *target, transform);
	}

	void translate_cloud(const Eigen::Affine3f& transform){
		transformPointCloud(*target, *target, transform);
	}

	void simple_icp(void){
		if(correspondences.size() == 0){
			cout << "Doing align\n";
			pcl::IterativeClosestPoint<PointT, PointT> icp;
			icp.setInputSource(source);
			icp.setInputTarget(target);
			PointCloudT transformed;
			icp.align(transformed);
			homogeneous = icp.getFinalTransformation();
			correspondences = (*icp.correspondences_);
			fprintf(stdout, "Has converged?: %d\t Score: %g\n", icp.hasConverged(), icp.getFitnessScore());
		}
		else{
			cout << "Doing estimateRigidTransformation\n";
			pcl::registration::TransformationEstimationSVD<PointT, PointT> transform_estimation;
			transform_estimation.estimateRigidTransformation(*source, *target, correspondences, homogeneous);
		}
		translate_cloud(homogeneous.inverse());
	}

	void display_homogeneous_to_quaternion(void){
		Eigen::Matrix3f rotate;
		for(int i = 0; i<3 ; i++)
			for(int j = 0; j<3 ; j++ )
				rotate(i,j) = homogeneous(i,j);
		Eigen::Quaternionf q(rotate);
		
		cout << "Homogeneous matrix: \n" << homogeneous << endl;
		fprintf(stdout, "\nTranslation \n%f %f %f\n", homogeneous(0,3), homogeneous(1,3), homogeneous(2,3));
		Eigen::Matrix<float, 4, 1> coeffs = q.coeffs();
		fprintf(stdout, "Quaternion:\n%f %f %f %f\n", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
		fprintf(stdout, "g2o edge:\n%f %f 0 0 0 %f %f\n", homogeneous(0,3), homogeneous(1,3), coeffs[2], coeffs[3]);
		auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
		fprintf(stdout, "Quaternion to euler in degree: %g %g %g\n",rad2deg(euler[0]), rad2deg(euler[1]), rad2deg(euler[2]) );
	}

	Eigen::Affine3f get_custom_translation(void){
		Eigen::Affine3f translate = Eigen::Affine3f::Identity();
		translate.translation() << 0.0, 0.0, -1;
		translate.rotate(Eigen::AngleAxisf(PI, Eigen::Vector3f::UnitY()));
		return translate;
	}

	void simple_visualize(void){
		pcl::visualization::PCLVisualizer viewer("ICP");
		viewer.addCoordinateSystem(1.0);
		viewer.addCorrespondences<PointT>(source, target, correspondences);
		pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb1(source);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> red(target, 230, 20, 20);
		viewer.addPointCloud(source, rgb1, "source");
		viewer.addPointCloud(target, red, "target");
		while(! viewer.wasStopped())
			viewer.spinOnce();
	}

	void save_to_pcd(string output_path){
		pcl::io::savePCDFileASCII(output_path, *source + *target);
		fprintf(stdout, "Assemlbed Point Cloud saved to: %s\n", output_path.c_str());
	}

public:
	CloudOperations(Mat& arg_rgb1, Mat& arg_rgb2, Mat& arg_depth1, Mat& arg_depth2, 
					vector<pair<int, int>>& arg_kps1_coord, vector<pair<int, int>>& arg_kps2_coord):
					kps1_coord{arg_kps1_coord},
					kps2_coord{arg_kps2_coord},
					rgb1{arg_rgb1},
					rgb2{arg_rgb2},
					depth1{arg_depth1},
					depth2{arg_depth2},
					source{new PointCloudT},
					target{new PointCloudT}{};
	
	void start_processing(void){
		fprintf(stdout, "Size of kps1_coord: %lu\nSize of kps2_coord: %lu\n", kps1_coord.size(), kps2_coord.size());
		source = images2cloud(rgb1, depth1, kps1_coord, cloud_indexes1, cloud1_keypoints);
		target = images2cloud(rgb2, depth2, kps2_coord, cloud_indexes2, cloud2_keypoints);
		// fill_correspondences();
		// Eigen::Affine3f translate = get_custom_translation();
		// translate_cloud(translate);
		simple_visualize();
		// print_cloud_keypoints(cloud_indexes1, cloud1_keypoints);
		// print_cloud_keypoints(cloud_indexes2, cloud2_keypoints);
		// print_correspondences();
		simple_icp();
		display_homogeneous_to_quaternion();
		simple_visualize();
		// save_to_pcd("/home/udit/Desktop/assembled.pcd");
	}
};


int main(int argc, char* argv[] ){
	if(argc != 5){
		fprintf(stdout, "Usage: ./sift_icp rgb1.png rgb2.png depth1.png depth2.png\n");
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
	
	vector<pair<int, int>> kps1_coord;
	vector<pair<int, int>> kps2_coord;   
	
	ImageOperations image_processor{rgb1, rgb2, depth1, depth2, kps1_coord, kps2_coord};
	image_processor.start_processing();

	CloudOperations cloud_processor{rgb1, rgb2, depth1, depth2, kps1_coord, kps2_coord};
	cloud_processor.start_processing();

	return 0;
}