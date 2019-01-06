#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "file_parser.h"
#include "keypoints_gui.h"
#include "sift_icp.h"

using namespace std;

int main(int argc, char const *argv[]){
	if(argc != 4){
		fprintf(stdout, "Usage: %s /path/to/rgb_images /path/to/depth_images trajectories.txt\n",
				argv[0]);
		return 1;
	}

	vector<string> rgb_images;
	vector<string> depth_images;
	ParseImages parser(argv[1], argv[2], rgb_images, depth_images);
	parser.start_processing();
	
	pair<int ,int> going;
	pair<int, int> coming;
	FileParser trajectory_info{argv[3], going, coming};
	trajectory_info.start_processing();

	vector<pair<string, string>> rgb_pairs;
	vector<pair<string, string>> depth_pairs;
	MatchImagePair pair_maker{rgb_images, depth_images, going, coming, 
								rgb_pairs, depth_pairs};
	pair_maker.start_processing();

	for(int i=0; i<2; ++i){
		vector<pair<int, int>> kps1_coord;
		vector<pair<int, int>> kps2_coord;
		gui::GenerateKeypoints keypoint_gui(rgb_pairs[i].first, rgb_pairs[i].second,
										kps1_coord, kps2_coord);
		keypoint_gui.start_processing();

		Mat rgb1 = imread(rgb_pairs[i].first, IMREAD_COLOR );
		Mat rgb2 = imread(rgb_pairs[i].second, IMREAD_COLOR );
		Mat depth1 = imread(depth_pairs[i].first, IMREAD_ANYDEPTH);
		Mat depth2 = imread(depth_pairs[i].second, IMREAD_ANYDEPTH);

		CloudOperations cloud_processor{rgb1, rgb2, depth1, depth2, kps1_coord, kps2_coord, 1};
		cloud_processor.start_processing();
	}

	return 0;
}