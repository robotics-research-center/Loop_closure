#include <iostream>
#include <dirent.h>
#include <string>
#include "generate_graph.h"
#include "sift_icp.h"

using namespace std;

class ParseImages{
private:
	const char* rgb_folder;
	const char* depth_folder;
	vector<string>& rgb_images;
	vector<string>& depth_images;

private:
	int get_cloud_number(const string &str){
		int index = 0;
		string result{};
		while(str[index] != '.'){
			result += str[index];
			++index;
		}
		return stoi(result);
	}

	void get_extension(const string& str, string& extension){
		int index = 0; 
		while(str[index] != '.'){
			++index;
		}
		while(index != str.size()){
			extension += str[index];
			++index;
		}
	}
	
	void get_image_path(int image_index, const string& extension, 
							const char* directory, string& image_path){
		int index=0;
		string path{};
		while(directory[index] != '\0'){
			path += directory[index];
			++index;
		}
		
		image_path = path + to_string(image_index) + extension;
	}

	void parse_images(vector<string>& images, const char* folder){
		string extension;
		bool extension_parsed = false;
		vector<int> list;
		DIR* directory = opendir(folder);
		dirent* pointer;
		while((pointer = readdir(directory)) != NULL){
			if(pointer->d_name[0] != '.'){
				list.push_back(get_cloud_number(pointer->d_name));
				if(extension_parsed == false){
					get_extension(pointer->d_name, extension);
					extension_parsed = true;
				}
			}
		}
		closedir(directory);
		sort(list.begin(), list.end());

		for(int i=0; i<list.size(); ++i){
			string image_path;
			get_image_path(list[i], extension, folder, image_path);
			images.push_back(image_path);
		}
	}

public:
	ParseImages(const char* rgb, const char* depth, vector<string>& rgb_list, 
				vector<string>& depth_list):
				rgb_folder{rgb},
				depth_folder{depth},
				rgb_images{rgb_list},
				depth_images{depth_list}{};

	void start_processing(void){
		parse_images(rgb_images, rgb_folder);
		parse_images(depth_images, depth_folder);
	}
	
};

class GenerateEdges{
private:
	vector<Pose> poses;
	vector<Edge> edges;
	const vector<string>& rgb_images;
	const vector<string>& depth_images;
	Mat rgb1, rgb2, depth1, depth2;

private:
	void load_images(const int index1, const int index2){
		rgb1 = imread(rgb_images[index1], IMREAD_COLOR );
		rgb2 = imread(rgb_images[index2], IMREAD_COLOR );
		depth1 = imread(depth_images[index1], IMREAD_ANYDEPTH);
		depth2 = imread(depth_images[index2], IMREAD_ANYDEPTH);
	}

public:
	GenerateEdges(const vector<string>& rgb_list, const vector<string>& depth_list):
					rgb_images{rgb_list},
					depth_images{depth_list}{};
	
	void start_processing(void){
		GenerateGraph graph_generator{poses, edges};
		
		for(int i=0; i<rgb_images.size(); ++i){	
			load_images(i, i+1);

			vector<pair<int, int>> kps1_coord;
			vector<pair<int, int>> kps2_coord;  
			
			ImageOperations image_processor{rgb1, rgb2, depth1, depth2, kps1_coord, kps2_coord};
			image_processor.start_processing();

			float delta_x=0.0, delta_y=0.0, delta_theta=0.0;
			CloudOperations cloud_processor{rgb1, rgb2, depth1, depth2, kps1_coord, kps2_coord, 1};
			cloud_processor.start_processing();
			cloud_processor.get_edge_parameters(delta_x, delta_y, delta_theta);

			graph_generator.add_edge(delta_x, delta_y, delta_theta);
		}

		GraphVisualizer graph_visualizer{poses, edges};
		graph_visualizer.start_processing();
	}

	void synthetic_trajectory(void){
		GenerateGraph graph_generator{poses, edges};
		GraphVisualizer graph_visualizer{poses, edges};

		for(int i=0; i<10; ++i){
			graph_generator.add_edge(2, 0, 0);
		}
		for(int i=0; i<6; ++i){
			graph_generator.add_edge(2, 0, PI/6);
		}
		for(int i=0; i<10; ++i){
			graph_generator.add_edge(2, 0, 0);
		}

		graph_visualizer.start_processing();
	}
};

int main(int argc, char const *argv[]){
	if(argc != 3){
		fprintf(stdout, "Usage: %s /path/to/rgb_images /path/to/depth_images\n",
				argv[0]);
		return 1;
	}

	vector<string> rgb_images;
	vector<string> depth_images;
	ParseImages parser(argv[1], argv[2], rgb_images, depth_images);
	parser.start_processing();

	GenerateEdges generator(rgb_images, depth_images);
	generator.start_processing();
	// generator.synthetic_trajectory();

	return 0;
}