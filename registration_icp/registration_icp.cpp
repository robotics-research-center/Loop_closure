#include <iostream>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Dense>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


int main(int argc, char const *argv[])
{	

	if (argc != 4)
	{
		std::cout << "Beware of the fact that cloud1 is getting transformed into " << argv[3] << '\n';
		if (argv[0])
			std::cout << "Usage: " << argv[0] << " cloud1.pcd cloud2.pcd transformed_cloud1.pcd" << '\n';
		else
			std::cout << "Usage: <executable> cloud1.pcd cloud2.pcd transformed_cloud1.pcd" << '\n';
		

		exit(1);
	}

	PointCloudT::Ptr cloud1 (new PointCloudT);
	PointCloudT::Ptr cloud2 (new PointCloudT);
	PointCloudT::Ptr cloud_filtered (new PointCloudT);


	if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud1) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}

	std::cout   << "Loaded "
				<< cloud1->width * cloud1->height
				<< " data points from test_pcd.pcd i.e. " << argv[1]
				<< std::endl;


	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (cloud1);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	pcl::io::savePCDFileASCII("filtered_cloud_1.pcd", *cloud_filtered);
	std::cout << "Cloud filtered_cloud_1 " << '\n';


	if (pcl::io::loadPCDFile<PointT> (argv[2], *cloud2) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd2.pcd \n");
		return (-1);
	}

	std::cout   << "Loaded "
				<< cloud2->width * cloud2->height
				<< " data points from test_pcd2.pcd i.e. " << argv[2]
				<< std::endl;


	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(cloud1);
	icp.setInputTarget(cloud2);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (0.05);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (50);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (1);	


	PointCloudT transformed;
	icp.align(transformed);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	pcl::io::savePCDFileASCII (argv[3], transformed);
	std::cout << "Writing transformed cloud 1 to " << argv[3] << '\n';



	return 0;
}