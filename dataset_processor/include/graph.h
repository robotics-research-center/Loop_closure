#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "graph.h"

using namespace std;
using namespace Eigen;

#define PI 3.1415
#define rad2deg(num)(num * 180/PI)

struct Pose{
	float x;
	float y;
	float theta_z;
	int id;
	Pose(float x=0.0, float y=0.0, float theta_z=0.0, int id=0){
		this->x = x;
		this->y = y;
		this->theta_z = theta_z;
		this->id = id;
	};
	
	friend ostream& operator<<(ostream& out, const Pose& pose){
		out << "Id: " << pose.id << "\tx: "<< pose.x << "\ty: " << pose.y
			<< "\ttheta_z(deg): " << rad2deg(pose.theta_z) << "\n--\n";
		return out;
	}
};

struct Edge{
	float delta_x;
	float delta_y;
	float delta_theta;
	int from_id;
	int to_id;
	Edge(float delta_x=0.0, float delta_y=0.0, float delta_theta=0.0,
			int from_id=0, int to_id=0){
		this->delta_x = delta_x;
		this->delta_y = delta_y;
		this->delta_theta = delta_theta;
		this->from_id =from_id;
		this->to_id = to_id;
	};

	friend ostream& operator<<(ostream& out, const Edge& edge){
		out << "from: " << edge.from_id << " to: " << edge.to_id 
			<< " delta_x: " << edge.delta_x << " delta_y: " << edge.delta_y
			<< " delta_theta(deg): " << rad2deg(edge.delta_theta) << "\n--\n";
	}	
};

class GenerateRT{
private:
	const vector<pair<string, string>>& rgb_pairs;
	const vector<pair<string, string>>& depth_pairs;
	const vector<vector<float>>& transforms;
	vector<Edge>& edges;

private:
	float get_delta_theta_z(const Eigen::Matrix3f& mat){
		float y_angle = atan2(-mat(2, 0), sqrt(pow(mat(0, 0), 2) + pow(mat(1, 0), 2)));
		float z_angle = atan2(mat(1, 0)/cos(y_angle), mat(0, 0)/cos(y_angle));
		float x_angle = atan2(mat(2, 1)/cos(y_angle), mat(2, 2)/cos(y_angle));

		return z_angle;
	}

	int get_transform_index(const int id){
		int index = 0;
		for(index; index<transforms.size(); ++index){
			if(transforms[index][0] == id){
				return index;
			}
		}
	}

	void get_pose(Pose& pose){
		const vector<float> transform = transforms[get_transform_index(pose.id)];
		Quaternionf q(transform[7], transform[4], transform[5], transform[6]);
		Eigen::Matrix3f rotate = q.toRotationMatrix();

		pose.x = transform[1];
		pose.y = transform[2];
		pose.theta_z = get_delta_theta_z(rotate);
	}

	int find_last_slash(const string& str){
		int last_index = 0;
		const char slash = '/';

		for(int i=0; i<str.size(); ++i){
			if(str[i] == slash){
				last_index = i;
			}
		}
		return last_index;
	}

	int get_image_id(const string &str){
		string result{};
		for(int index=find_last_slash(str); str[index] != '.'; ++index){
			if(isdigit(str[index])){
				result += str[index];
			}
		}
		return stoi(result);
	}
	
	void get_current_frame_in_world_frame(Eigen::Affine3f& frame, const Pose& pose){
		frame = Eigen::Affine3f::Identity();
		frame.translation() << pose.x, pose.y, 0;
		frame.rotate(AngleAxisf(pose.theta_z, Vector3f::UnitZ()));
	}

	void get_relative_transform(const Pose& first, const Pose& second, 
								Eigen::Affine3f& second_frame_in_first){
		Eigen::Affine3f first_in_world_frame, second_in_world_frame;
		get_current_frame_in_world_frame(first_in_world_frame, first);
		get_current_frame_in_world_frame(second_in_world_frame, second);
		second_frame_in_first = first_in_world_frame.inverse()*second_in_world_frame;
		
		float delta_x = second_frame_in_first.matrix()(0, 3);
		float delta_y = second_frame_in_first.matrix()(1, 3);
		float delta_theta = acos(second_frame_in_first.matrix()(0, 0));

		Edge edge{delta_x, delta_y, delta_theta, first.id, second.id};
		edges.push_back(edge);
	}

	void print_edges(void){
		for(auto edge : edges){
			cout << edge;
		}
	}

	void generate_edges(void){
		for(int i=0; i<rgb_pairs.size(); ++i){
			Pose pose1(0.0, 0.0, 0.0, get_image_id(rgb_pairs[i].first));
			Pose pose2(0.0, 0.0, 0.0, get_image_id(rgb_pairs[i].second));
			get_pose(pose1);
			get_pose(pose2);
			
			Eigen::Affine3f second_frame_in_first;
			get_relative_transform(pose1, pose2, second_frame_in_first);
		}
	}

	void write_relative_transforms(void){
		ofstream file_write("image_pairs.txt");
		for(int i=0; i<rgb_pairs.size(); ++i){
			file_write << rgb_pairs[i].first << endl << rgb_pairs[i].second << endl
						<< edges[i];
		}
		file_write.close();
	}

public:
	GenerateRT(const vector<pair<string, string>>& rgbs,
				const vector<pair<string, string>>& depths,
				const vector<vector<float>>& arg_transforms,
				vector<Edge>& arg_edges):
				rgb_pairs{rgbs},
				depth_pairs{depths},
				transforms{arg_transforms},
				edges{arg_edges}{};
	
	void start_processing(void){
		generate_edges();
		print_edges();
		write_relative_transforms();
	}
};

#endif