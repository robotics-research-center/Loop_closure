#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>

#include <string>
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

using namespace message_filters;

ros::Publisher pub;

class Filter{
private:
	std::string directory;
	static int cloud_id;
	static int odom_id;

public:
	Filter(const std::string& path): directory{path}{};

	void voxelize(const PointCloudT::Ptr cloud, float size){
		pcl::VoxelGrid<pcl::PointXYZ> voxel;
		voxel.setInputCloud(cloud);
		voxel.setLeafSize(size, size, size);
		
		voxel.filter(*cloud);
	}
	
	void write_cloud(PointCloudT::Ptr cloud){
		std::string file_name{};
		if(cloud_id<10)
			file_name = directory + "scan00" + std::to_string(cloud_id) + ".3d";
		else if(cloud_id<100)
			file_name = directory + "scan0" + std::to_string(cloud_id) + ".3d";
		else
			file_name = directory + "scan" + std::to_string(cloud_id) + ".3d";
		
		std::ofstream file_write{file_name};
		
		for(int index=0; index<cloud->points.size(); ++index){
			file_write << cloud->points[index].x << " " << cloud->points[index].y 
			<< " " << cloud->points[index].z << std::endl;
		}
		file_write.close();
	}

	void cb_cloud(const sensor_msgs::PointCloud2ConstPtr& input){
		PointCloudT::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*input, *cloud);
		voxelize(cloud, 0.05);
		write_cloud(cloud);

		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*cloud, output);
		pub.publish(output);

		++cloud_id;
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
		std::string file_name{};
		if(odom_id<10)
			file_name = directory + "scan00" + std::to_string(odom_id) + ".pose";
		else if(odom_id<100)
			file_name = directory + "scan0" + std::to_string(odom_id) + ".pose";
		else
			file_name = directory + "scan" + std::to_string(odom_id) + ".pose";

		std::ofstream file_write{file_name};

  		file_write << msg->pose.pose.position.x << " " << msg->pose.pose.position.y
  					<< " " << msg->pose.pose.position.z << std::endl;
  		
  		Eigen::Vector3f euler;
  		quat2euler(msg, euler);  		
  		file_write << euler[0] << " " << euler[1] << " " << euler[2] << std::endl;
  		
  		file_write.close();

  		++odom_id;
	}
};

int Filter::cloud_id = 1;
int Filter::odom_id = 1;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud,
	const nav_msgs::Odometry::ConstPtr& pose){
	
	static int timer=1;
	const int frame_drop=4;
	if(timer%frame_drop == 0){
		ROS_INFO("Inside common callback\n");

		Filter processor("/home/udit/Desktop/data/");
		processor.cb_cloud(cloud);
		processor.cb_odom(pose);
	}
	++timer;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "pcd_extraction");
	
	ros::NodeHandle nh;
	
	Subscriber<sensor_msgs::PointCloud2> sub_cloud(nh, "in_cloud", 1);
	Subscriber<nav_msgs::Odometry> sub_odom(nh, "in_odom", 1);
	
	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
		nav_msgs::Odometry> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cloud, sub_odom);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
	
	ros::spin();
	
	return 0;
}