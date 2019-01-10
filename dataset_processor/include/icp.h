#ifndef ICP_H
#define ICP_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/registration_visualizer.h>
#include <cmath>
#include "graph.h"

using namespace cv;
using namespace std;

#define PI 3.1415
#define rad2deg(num)(num * 180/PI)

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CloudOperations{
private:
	Mat &rgb1, &rgb2, &depth1, &depth2;
	PointCloudT::Ptr source, target;
	pcl::Correspondences correspondences;
	Eigen::Matrix4f homogeneous;
	const Edge& edge;
	
private:
	PointCloudT::Ptr images2cloud(const Mat& rgb_image, const Mat& depth_image){
		const float f = 525, cx = 319.5, cy = 239.5;
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
	
					++image_index;
					cloud->points.push_back(point);
				}
			}
		}
		cloud->width = cloud->points.size();
		cloud->height = 1;
		return cloud;
	}

	void translate_cloud(const Eigen::Matrix4f& transform){
		transformPointCloud(*target, *target, transform);
	}

	void simple_icp(const Eigen::Matrix4f& guess){
		cout << "Doing align\n";
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setInputSource(source);
		icp.setInputTarget(target);
		PointCloudT transformed;
		icp.align(transformed, guess);
		homogeneous = icp.getFinalTransformation();
		correspondences = (*icp.correspondences_);
		fprintf(stdout, "Has converged?: %d\t Score: %g\n", icp.hasConverged(), icp.getFitnessScore());

		translate_cloud(homogeneous.inverse());
	}

	void get_delta_theta_z(float& z_angle){
		const Eigen::Matrix4f &mat = homogeneous;
		float y_angle = atan2(-mat(2, 0), sqrt(pow(mat(0, 0), 2) + pow(mat(1, 0), 2)));
		z_angle = atan2(mat(1, 0)/cos(y_angle), mat(0, 0)/cos(y_angle));
		float x_angle = atan2(mat(2, 1)/cos(y_angle), mat(2, 2)/cos(y_angle));
	}

	void get_edge_parameters(void){
		const float delta_x = homogeneous(0,3); 
		const float delta_y = homogeneous(1,3);
		float delta_theta=0.0;
		get_delta_theta_z(delta_theta);

		Edge final_edge{delta_x, delta_y, delta_theta, edge.from_id, edge.to_id};
		cout << final_edge;  
		// fprintf(stdout, "Edge parameters:\ndelta_x: %f delta_y: %f delta_theta: %f(deg)\n",
		// 		delta_x, delta_y, rad2deg(delta_theta));
	}
	
	void display_homogeneous_to_quaternion(void){
		Eigen::Matrix3f rotate;
		for(int i = 0; i<3 ; i++)
			for(int j = 0; j<3 ; j++ )
				rotate(i,j) = homogeneous(i,j);
		Eigen::Quaternionf q(rotate);
		
		cout << "Homogeneous matrix: \n" << homogeneous << endl;
		// fprintf(stdout, "\nTranslation \n%f %f %f\n", homogeneous(0,3), homogeneous(1,3), homogeneous(2,3));
		// Eigen::Matrix<float, 4, 1> coeffs = q.coeffs();
		// fprintf(stdout, "Quaternion:\n%f %f %f %f\n", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
		// fprintf(stdout, "g2o edge:\n%f %f 0 0 0 %f %f\n", homogeneous(0,3), homogeneous(1,3), coeffs[2], coeffs[3]);
		// auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
		// fprintf(stdout, "Quaternion to euler in degree: %g %g %g\n",rad2deg(euler[0]), rad2deg(euler[1]), rad2deg(euler[2]) );
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

	void convert_edge_to_guess_matrix(Eigen::Matrix4f& guess){
		guess(0, 3) = edge.delta_x;
		guess(1, 3) = edge.delta_y;
		
		guess(0, 0) = cos(edge.delta_theta);
		guess(0, 1) = -sin(edge.delta_theta);
		guess(1, 0) = sin(edge.delta_theta);
		guess(1, 1) = cos(edge.delta_theta);
	}

public:
	CloudOperations(Mat& arg_rgb1, Mat& arg_rgb2, Mat& arg_depth1, Mat& arg_depth2,
					const Edge& arg_edge):
					rgb1{arg_rgb1},
					rgb2{arg_rgb2},
					depth1{arg_depth1},
					depth2{arg_depth2},
					edge{arg_edge},
					source{new PointCloudT},
					target{new PointCloudT}{};
	
	void start_processing(void){
		source = images2cloud(rgb1, depth1);
		target = images2cloud(rgb2, depth2);
		
		simple_visualize();

		Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
		convert_edge_to_guess_matrix(guess);
		
		simple_icp(guess);
		display_homogeneous_to_quaternion();
		get_edge_parameters();
		simple_visualize();
	}

};

#endif