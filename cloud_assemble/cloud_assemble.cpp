#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class g2oParser{
private:
	vector<vector<float>> transforms;
	const char *g2o_file;
	const string keyword = "EDGE_SE3:QUAT";
	const int word_count = 9;

private:
	vector<int> find_spaces(const string &line){
		char space = ' ';
		vector<int> index;
		for(int i=0; i<line.length(); ++i){
			if(line[i]==space){
				index.push_back(i);
			}
		}
		return index;
	}

	vector<float> extract_substring(const string &line, const vector<int> &spaces){
		vector<float> substrings;
		for(int i=0; i<word_count; ++i){
			string word{};
			for(int j=spaces[i]+1; j<spaces[i+1]; ++j){
				word += line[j];
			}
			substrings.push_back(stod(word));
		}
		return substrings;
	}

	void parse_g2o_file(void){
		ifstream file_read(g2o_file);
		string line;
		if(file_read.is_open()){
			while(getline(file_read, line)){
				size_t found = line.find(keyword);
				if(found!=string::npos){
					vector<float> transform = extract_substring(line, find_spaces(line));
					transforms.push_back(transform);
				}
			}
		}
	}

public:
	g2oParser(const char *file):g2o_file{file}{
		parse_g2o_file();
	};

	vector<vector<float>> get_transforms(void){
		return transforms;
	}
	
	void print_transforms(void){
		for(vector<float> transform : transforms){
			for(size_t index=0; index<transform.size(); ++index){
				cout << transform[index] << " ";
			}
			cout << "\n";
		}
	}
};


class cloudOperations{
private:
	const char *directory;
	map<int, bool> cloud_queue;
	PointCloudT::Ptr assembled_map;
	PointCloudT::Ptr new_point_cloud;
	vector<vector<float>>& transforms;
	map<int, int>& image_labels;

private:
	int get_cloud_number(const string &str){
		const int cloud_length = 5;
		int index = cloud_length;
		string result{};
		while(str[index] != '.'){
			result += str[index];
			++index;
		}
		return stoi(result);
	}

	void load_cloud_queue(){	
		vector<int> list;
		string keyword = "cloud";
		DIR* dirp = opendir(directory);
		dirent* dp;
		while((dp = readdir(dirp)) != NULL){
			if(string(dp->d_name).find(keyword) != std::string::npos){
				list.push_back(get_cloud_number(dp->d_name));
			}
		}
		closedir(dirp);
		sort(list.begin(), list.end());
		for(int i=0; i<list.size(); ++i){
			cloud_queue.insert(pair<int, bool>(list[i], false)); 
		}
	}

	string get_cloud_path(int cloud_index){
		int index=0;
		string path{};
		while(directory[index] != '\0'){
			path += directory[index];
			++index;
		}
		string cloud_path = path + "cloud" + to_string(cloud_index) + ".pcd";
		return cloud_path;
	}

	void load_point_cloud(int cloud_index){
		io::loadPCDFile<PointT> (get_cloud_path(cloud_index), *new_point_cloud);
		cout << "PointCloud: " << cloud_index << " is loaded: " 
			<< cloud_queue[cloud_index] << endl;
	}

	void transform_point_cloud(const vector<float> &sub){
		Eigen::Quaternionf q(sub[8], sub[5], sub[6], sub[7]);
		Eigen::Matrix3f rotate = q.toRotationMatrix();
		Eigen::Matrix4f homogeneous = Eigen::Matrix4f::Identity();
		for(int i=0; i<3; ++i)
			for(int j=0; j<3; ++j)
				homogeneous(i, j) = rotate(i, j);
		homogeneous(0, 3) = sub[2];
		homogeneous(1, 3) = sub[3];
		homogeneous(2, 3) = sub[4];
		transformPointCloud(*new_point_cloud, *new_point_cloud, homogeneous.inverse());
	}

	void add_to_map(void){
		*assembled_map += *new_point_cloud;
	}

	void simple_voxelize(float leaf_size){
		pcl::VoxelGrid<PointT> voxel;
		voxel.setInputCloud(new_point_cloud);
		voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
		voxel.filter(*new_point_cloud);
	}

	void red_colorization(void){
		for(size_t index=0; index<new_point_cloud->points.size(); ++index){
			new_point_cloud->points[index].r = 200;
			new_point_cloud->points[index].g = 0;
			new_point_cloud->points[index].b = 0;
		}
	}

	void green_colorization(void){
		for(size_t index=0; index<new_point_cloud->points.size(); ++index){
			new_point_cloud->points[index].r = 0;
			new_point_cloud->points[index].g = 200;
			new_point_cloud->points[index].b = 0;
		}
	}

	void blue_colorization(void){
		for(size_t index=0; index<new_point_cloud->points.size(); ++index){
			new_point_cloud->points[index].r = 0;
			new_point_cloud->points[index].g = 0;
			new_point_cloud->points[index].b = 200;
		}
	}

	void yellow_colorization(void){
		for(size_t index=0; index<new_point_cloud->points.size(); ++index){
			new_point_cloud->points[index].r = 200;
			new_point_cloud->points[index].g = 200;
			new_point_cloud->points[index].b = 0;
		}
	}

	void violet_colorization(void){
		for(size_t index=0; index<new_point_cloud->points.size(); ++index){
			new_point_cloud->points[index].r = 200;
			new_point_cloud->points[index].g = 100;
			new_point_cloud->points[index].b = 200;
		}
	}

	void network_prediction(int cloud_index){
		if(image_labels[cloud_index] == 1){ red_colorization();}
		else if(image_labels[cloud_index] == 2){ green_colorization();}
		else if(image_labels[cloud_index] == 3){ blue_colorization();}
		else if(image_labels[cloud_index] == 4){ yellow_colorization();}
		else if(image_labels[cloud_index] == 5){ violet_colorization();}
	}

	void save_to_pcd(string output_path){
		pcl::io::savePCDFileASCII(output_path, *assembled_map);
	}

public:
	cloudOperations(const char* arg_directory, vector<vector<float>>& arg_transforms,
					map<int, int>& image_predictions) :
					directory{arg_directory},
					transforms{arg_transforms},
					image_labels{image_predictions},
					new_point_cloud{new PointCloudT()},
					assembled_map{new PointCloudT}{
						load_cloud_queue();
					};

	void print_cloud_info(void){
		for(pair<int, bool> cloud : cloud_queue){
			cout << cloud.first << " " << cloud.second << " " << get_cloud_path(cloud.first)<< endl;
		}
	}

	void simple_visualize(void){
		pcl::visualization::PCLVisualizer viewer("Assembled_cloud");
		pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(assembled_map);
		viewer.addPointCloud<PointT>(assembled_map, rgb, "cloud_points");
		while(! viewer.wasStopped())
			viewer.spinOnce();
	}

	void assemble_clouds(void){
		cloud_queue[62] = true;
		load_point_cloud(62);
		add_to_map();
		
		for(size_t i=0; i<transforms.size(); ++i){
			int cloud_to_load = transforms[i][1];
			if(cloud_queue[cloud_to_load] == false){
				cloud_queue[cloud_to_load] = true;
				load_point_cloud(cloud_to_load);
				simple_voxelize(0.01f);
				network_prediction(cloud_to_load);
				//transform_point_cloud(transforms[i]);
				add_to_map();
			}
		}
		simple_visualize();
		save_to_pcd("/home/udit/Desktop/assembled2.pcd");
	}
};


class predictionFilesParser{
private:
	map<int, int>& image_labels;
	const char* images_file;
	const char* predictions_file;
	const string keyword = "/rgb/";

private:
	void parse_files(void){
		ifstream images_read(images_file), labels_read(predictions_file);
		string images_line, labels_line;
		if(images_read.is_open() && labels_read.is_open()){
			while(getline(images_read, images_line) && getline(labels_read, labels_line)){
				size_t found = images_line.find(keyword);
				if(found!=string::npos){
					string image_number="";
					for(int i=found+keyword.size(); images_line[i]!='.'; ++i){
						image_number += images_line[i];
					}
					image_labels.insert(pair<int, int>(stoi(image_number), stoi(labels_line)));
				}
			}
		}
	}

public:
	predictionFilesParser(map<int ,int>& image_predictions, 
							const char* arg_images_file,
							const char* arg_predictions_file) :
	image_labels{image_predictions},
	images_file{arg_images_file},
	predictions_file{arg_predictions_file}{
		parse_files();
	};

	void print_info(void){
		for(pair<int, int> image_label : image_labels){
			fprintf(stdout, "Image number: %d\tbelongs to label: %d\n", image_label.first, image_label.second);	
		}
	}
};


int main(int argc, char const *argv[]){
	if(argc != 5){
		cerr << "Usage: ./cloud_assemble file.g2o /path/to/clouds/ images_file.txt predictions_file.txt\n";
		return 1;
	}

	g2oParser parser(argv[1]);
	vector<vector<float>> transforms = parser.get_transforms();
	//parser.print_transforms();

	map<int, int> image_predictions;
	predictionFilesParser prediction_parser(image_predictions, argv[3], argv[4]); 
	prediction_parser.print_info();

	cloudOperations cloud_parser(argv[2], transforms ,image_predictions);
	cloud_parser.print_cloud_info();
	cloud_parser.assemble_clouds();

	return 0;
}