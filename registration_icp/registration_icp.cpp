#include <iostream>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
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

	if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud1) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}

	std::cout   << "Loaded "
				<< cloud1->width * cloud1->height
				<< " data points from test_pcd.pcd "
				<< std::endl;

	PointCloudT::Ptr cloud2 (new PointCloudT);

	if (pcl::io::loadPCDFile<PointT> (argv[2], *cloud2) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd2.pcd \n");
		return (-1);
	}

	std::cout   << "Loaded "
				<< cloud2->width * cloud2->height
				<< " data points from test_pcd2.pcd "
				<< std::endl;


	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(cloud1);
	icp.setInputTarget(cloud2);

	PointCloudT transformed;
	icp.align(transformed);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	pcl::io::savePCDFileASCII (argv[3], transformed);
	std::cout << "Writing transformed cloud 1 to " << argv[3] << '\n';



	return 0;
}