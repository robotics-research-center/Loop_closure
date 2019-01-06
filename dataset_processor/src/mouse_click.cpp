#include <iostream>
#include "keypoints_gui.h"
#include "sift_icp.h"

int main(int argc, char const *argv[]){
	if(argc != 5){
		fprintf(stdout, "Usage: %s image1.jpg image2.jpg depth1.png depth2.png\n",
				argv[0]);
		return 1;
	}

	vector<pair<int, int>> kps1_coord;
	vector<pair<int, int>> kps2_coord;
	gui::GenerateKeypoints keypoint_gui(argv[1], argv[2], kps1_coord, kps2_coord);
	keypoint_gui.start_processing();
	
	Mat rgb1 = imread(argv[1], IMREAD_COLOR );
	Mat rgb2 = imread(argv[2], IMREAD_COLOR );
	Mat depth1 = imread(argv[3], IMREAD_ANYDEPTH);
	Mat depth2 = imread(argv[4], IMREAD_ANYDEPTH);

	CloudOperations cloud_processor{rgb1, rgb2, depth1, depth2, kps1_coord, kps2_coord, 1};
	cloud_processor.start_processing();
	
	return 0;
}