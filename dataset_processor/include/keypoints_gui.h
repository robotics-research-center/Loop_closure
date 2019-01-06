#ifndef RECORD_KEYPOINTS_H
#define RECORD_KEYPOINTS_H

#include <iostream>
#include <opencv2/opencv.hpp>

namespace gui{

using namespace std;
using namespace cv;

struct ImageInfo{
	Mat& image;
	vector<pair<int, int>>& keypoints;
	const string& window_name;

	ImageInfo(Mat& arg_image, vector<pair<int, int>>& arg_keypoints,
				const string& window):
				image{arg_image},
				keypoints{arg_keypoints},
				window_name{window}{};
};

class GenerateKeypoints{
private:
	const string rgb1;
	const string rgb2;
	vector<pair<int, int>>& keypoints1;
	vector<pair<int, int>>& keypoints2;

private:
	static void on_mouse1(int event, int x, int y, int flags, void* user_data){
		if(flags == EVENT_FLAG_LBUTTON){
			static int counter1=0;
			ImageInfo& info = *(ImageInfo*) user_data;
			Point point(x, y);
			
			circle(info.image, point, 3, Scalar(0, 0, 255), 1, 8, 0);
			imshow(info.window_name, info.image);

			if(counter1%2 == 0){
			cout << "Coordinate of image1: " << x << " " << y << "\n";
				info.keypoints.push_back(make_pair(x, y));
			}
			++counter1;
		}
	}

	static void on_mouse2(int event, int x, int y, int flags, void* user_data){
		if(flags == EVENT_FLAG_LBUTTON){
			static int counter2=0;
			ImageInfo& info = *(ImageInfo*) user_data;
			Point point(x, y);
			
			circle(info.image, point, 3, Scalar(0, 0, 255), 1, 8, 0);
			imshow(info.window_name, info.image);

			if(counter2%2 == 0){
			cout << "Coordinate of image2: " << x << " " << y << "\n";
				info.keypoints.push_back(make_pair(x, y));
			}
			++counter2;
		}
	}
	
	void start_gui(void){
		Mat image1 = imread(rgb1, IMREAD_COLOR);
		const string window_name1 = "opencv_viewer1";
		
		Mat image2 = imread(rgb2, IMREAD_COLOR);
		const string window_name2 = "opencv_viewer2";

		if(image1.empty() || image2.empty()){
			fprintf(stdout, "Unable to open images\n");
			cout << rgb1 << endl << rgb2 << endl;
			return ;
		}
	
		ImageInfo info1{image1, keypoints1, window_name1};
		ImageInfo info2{image2, keypoints2, window_name2};

		namedWindow(window_name1, WINDOW_AUTOSIZE);
		namedWindow(window_name2, WINDOW_AUTOSIZE);
		moveWindow(window_name1, 10, 50);
		moveWindow(window_name2, 700, 50);

		setMouseCallback(window_name1, on_mouse1, &info1);
		setMouseCallback(window_name2, on_mouse2, &info2);

		imshow(window_name1, image1);
		imshow(window_name2, image2);

		waitKey(0);

		destroyWindow(window_name1);
		destroyWindow(window_name2);
	}

	void print_keypoints(void){
		cout << "Keypoints in image1: " << keypoints1.size() << endl;
		for(pair<int, int> element : keypoints1){
			cout << element.first << "\t" << element.second << endl;
		}

		cout << "Keypoints in image2: " << keypoints2.size() << endl;
		for(pair<int, int> element : keypoints2){
			cout << element.first << "\t" << element.second << endl;
		}
	}

public:
	GenerateKeypoints(const string image1, const string image2,
						vector<pair<int, int>>& kps1, vector<pair<int, int>>& kps2):
						rgb1{image1}, rgb2{image2},
						keypoints1{kps1}, keypoints2{kps2}{};

	void start_processing(void){
		start_gui();
		print_keypoints();
	}	
};

} // namespace gui

#endif