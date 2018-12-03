#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;

void display_image(const Mat& image){
	namedWindow("opencv_viewer", WINDOW_AUTOSIZE);
	imshow("opencv_viewer", image);
	waitKey(0);
	destroyWindow("opencv_viewer");
}

template <typename detectorType>
void match_keypoints(const Mat& rgb1, const Mat& rgb2, Ptr<detectorType> feature_detector,
					 Ptr<DescriptorMatcher> matcher, const Mat& mask1, const Mat& mask2, bool mask=false){
	Mat final_image;
	vector<DMatch> matches;
	Mat descriptors1, descriptors2;
	vector<KeyPoint> keypoints1, keypoints2;
	
	if(mask==true){
		feature_detector->detectAndCompute(rgb1, mask1, keypoints1, descriptors1);
		feature_detector->detectAndCompute(rgb2, mask2, keypoints2, descriptors2);
	}
	else{
		feature_detector->detectAndCompute(rgb1, noArray(), keypoints1, descriptors1);
		feature_detector->detectAndCompute(rgb2, noArray(), keypoints2, descriptors2);	
	}

	vector<vector<DMatch>> knn_matches;
	matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
	
	const float threshold_ratio = 0.7f;
	for(size_t i=0; i<knn_matches.size(); ++i){
		if(knn_matches[i][0].distance < threshold_ratio * knn_matches[i][1].distance)
			matches.push_back(knn_matches[i][0]);
	}
	drawMatches(rgb1, keypoints1, rgb2, keypoints2, matches, final_image);
	display_image(final_image);
}

void get_mask(Mat& mask, int start_x, int start_y, int width, int height){
	Rect ROI(start_x, start_y, width, height);
	mask(ROI) = 255;	
}

int main(int argc, char const *argv[]){
	if(argc != 4){
		fprintf(stdout, "Usage: %s rgb1.png rgb2.png mask(1/0)\n", argv[0]);
		return 1;
	}
	Mat rgb1 = imread(argv[1], IMREAD_COLOR );
	Mat rgb2 = imread(argv[2], IMREAD_COLOR );
	if(rgb1.empty() || rgb2.empty()){
		fprintf(stdout, "Unable to open images\n");
		return 1;
	}

	const int features_count = 500;
	Ptr<xfeatures2d::SIFT> sift_detector = xfeatures2d::SIFT::create(features_count);
	Ptr<DescriptorMatcher> sift_matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
	
	Ptr<ORB> orb_detector = ORB::create();
	Ptr<DescriptorMatcher> orb_matcher = DescriptorMatcher::create("BruteForce-Hamming");

	Mat mask1(rgb1.size(), CV_8UC1, Scalar::all(0)); 
	Mat mask2(rgb2.size(), CV_8UC1, Scalar::all(0));
	get_mask(mask1, 0, 0, 320, 480);
	get_mask(mask2, 320, 0, 320, 480);
	display_image(rgb1);
	display_image(mask1);
	display_image(rgb2);
	display_image(mask2);
	bool mask_activate = bool(stoi(argv[3]));

	Mat rgb_flipped1, rgb_flipped2;
	cv::flip(rgb1, rgb_flipped1, 1);
	cv::flip(rgb2, rgb_flipped2, 1);
	
	match_keypoints<xfeatures2d::SIFT>(rgb1, rgb2, sift_detector, sift_matcher, mask1, mask2, mask_activate);
	match_keypoints<ORB>(rgb1, rgb2, orb_detector, orb_matcher, mask1, mask2, mask_activate);

	return 0;
}
