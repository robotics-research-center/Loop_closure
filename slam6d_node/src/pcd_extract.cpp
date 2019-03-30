#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <fstream>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

class Filter{
private:
	ros::NodeHandle& nh;
	sensor_msgs::PointCloud2 output;
	std::string directory;
	static int timer;
	static int cloud_id;
	static int odom_id;

public:
	Filter(ros::NodeHandle& n):nh{n}, directory{"/home/udit/Desktop/data/"}{};

	void voxelize(const PointCloudT::Ptr cloud, float size){
		pcl::VoxelGrid<pcl::PointXYZ> voxel;
		voxel.setInputCloud(cloud);
		voxel.setLeafSize(size, size, size);
		
		voxel.filter(*cloud);
	}
	
	void write_cloud(PointCloudT::Ptr cloud){
		std::string file_name = directory + "scan" + std::to_string(cloud_id) + ".3d";
		std::ofstream file_write{file_name};
		
		for(int index=0; index<cloud->points.size(); ++index){
			file_write << cloud->points[index].x << " " << cloud->points[index].y 
			<< " " << cloud->points[index].z << std::endl;
		}
		file_write.close();
	}

	void cb_cloud(const sensor_msgs::PointCloud2ConstPtr& input){
		if(timer%2 == 0){
			PointCloudT::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*input, *cloud);
			voxelize(cloud, 0.1);
			write_cloud(cloud);
			pcl::toROSMsg(*cloud, output);
			++cloud_id;
		}
	  	++timer;
	}

	void quat2euler(const nav_msgs::Odometry::ConstPtr& msg, Eigen::Vector3f& euler){
  		float q_x = msg->pose.pose.orientation.x;
  		float q_y = msg->pose.pose.orientation.y;
  		float q_z = msg->pose.pose.orientation.z;
  		float q_w = msg->pose.pose.orientation.w;

  		Eigen::Quaternionf q(q_w, q_x, q_y, q_z);
  		euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
	}

	void cb_odom(const nav_msgs::Odometry::ConstPtr& msg){
		if(timer%2 == 0){	
			std::string file_name = directory + "scan" + std::to_string(odom_id) + ".pose";
	  		std::ofstream file_write{file_name};

	  		file_write << msg->pose.pose.position.x << " " << msg->pose.pose.position.y
	  					<< " " << msg->pose.pose.position.z << std::endl;
	  		
	  		Eigen::Vector3f euler;
	  		quat2euler(msg, euler);  		
	  		file_write << euler[0] << " " << euler[1] << " " << euler[2] << std::endl;
	  		
	  		file_write.close();

	  		++odom_id;
	  	}
	}

	void process(ros::Publisher& pub){
		ros::Rate loop_rate{10};
		while(nh.ok()){
			pub.publish(output);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
};

int Filter::cloud_id = 100;
int Filter::odom_id = 100;
int Filter::timer = 1;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "pcd_extraction");
	
	ros::NodeHandle nh;
	Filter processor{nh};
	ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("in_cloud",
						1, &Filter::cb_cloud, &processor);

	ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("in_odom", 1, 
						&Filter::cb_odom, &processor);
	
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
	processor.process(pub);
	
	return 0;
}