#ifndef GENERATE_GRAPH_H
#define GENERATE_GRAPH_H

#include <iostream>
#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <cstdlib>
#include <fstream>
#include <string>
#include <cmath>

#define PI 3.1415

namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;

float rad2deg(float theta_radian){
	return (180*theta_radian)/PI;
}

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
		out << "from: " << edge.from_id << "\tto: " << edge.to_id 
			<< "\tdelta_x: " << edge.delta_x << "\tdelta_y: " << edge.delta_y
			<< "\tdelta_theta(deg): " << rad2deg(edge.delta_theta) << "\n--\n";
	}	
};

class GraphVisualizer{
private:
	const vector<Pose>& poses;
	const vector<Edge>& edges;

private:
	void plot_coordinate_axis(void){
		vector<int> x_points{0, 0, 10};
		vector<int> y_points{10, 0, 0};
		plt::plot(x_points, y_points, "bo-");
	}

	void plot_graph(void){
		vector<float> x_points, y_points, thetas;
		for(int i=0; i<poses.size(); ++i){
			x_points.push_back(poses[i].x);
			y_points.push_back(poses[i].y);
			thetas.push_back(poses[i].theta_z);
		}

		const int length = 1;
		plt::plot(x_points, y_points, "ro");
		for(int i=0; i<x_points.size(); ++i){
			vector<float> x_arrow{x_points[i], x_points[i] + length*cos(thetas[i])};
			vector<float> y_arrow{y_points[i], y_points[i] + length*sin(thetas[i])};
			plt::plot(x_arrow, y_arrow, "b->");
		}
		plt::xlim(-2, 30);
		plt::ylim(-2, 30);
		plot_coordinate_axis();
		plt::show();
	}

	void print_graph(void){
		cout << "--Poses--\n";
		for(int i=0; i<poses.size(); ++i){
			cout << poses[i];
		}
		cout << "--Edges--\n";
		for(int i=0; i<edges.size(); ++i){
			cout << edges[i];
		}
	}

public:
	GraphVisualizer(const vector<Pose>& arg_poses, const vector<Edge>& arg_edges):
					poses{arg_poses},
					edges{arg_edges}{};
	
	void start_processing(void){
		print_graph();
		plot_graph();
	}
};

class GenerateGraph{
private:
	vector<Pose>& poses;
	vector<Edge>& edges;
	Pose current_pose_world_frame;
	Affine3f current_frame_in_world_frame;

private:
	void new_point_in_current_frame(const Edge& edge, const Pose& old_pose, 
									Pose& new_pose){
		new_pose.x = edge.delta_x;
		new_pose.y = edge.delta_y;
		new_pose.theta_z = old_pose.theta_z + edge.delta_theta; 
	}

	void new_point_in_world_frame(const Pose& pose_current, const Affine3f& homogeneous,
									Pose& pose_world){
		const Vector4f old_vector{pose_current.x, pose_current.y, 0, 1};
		Vector4f new_vector;
		new_vector = homogeneous*old_vector;
		pose_world.x = new_vector[0];
		pose_world.y = new_vector[1];
		pose_world.theta_z = pose_current.theta_z;
	}
	
	void get_current_frame_in_world_frame(Affine3f& frame, const Pose& pose){
		frame = Affine3f::Identity();
		frame.translation() << pose.x, pose.y, 0;
		frame.rotate(AngleAxisf(pose.theta_z, Vector3f::UnitZ()));
	}

public:
	GenerateGraph(vector<Pose>& arg_poses, vector<Edge>& arg_edges):
					poses{arg_poses},
					edges{arg_edges}{
						current_pose_world_frame = Pose{0, 0, 0, 0};
						current_frame_in_world_frame = Affine3f::Identity();
						poses.push_back(current_pose_world_frame);
					};

	void add_edge(const float delta_x, const float delta_y, const float delta_theta){
		Pose new_pose_current_frame;

		Edge edge{delta_x, delta_y, delta_theta};
		edge.from_id = current_pose_world_frame.id;
		edge.to_id = current_pose_world_frame.id + 1;
		edges.push_back(edge);	

		new_point_in_current_frame(edge, current_pose_world_frame, 
										new_pose_current_frame);
		new_point_in_world_frame(new_pose_current_frame,
									current_frame_in_world_frame,
									current_pose_world_frame);
		++current_pose_world_frame.id;
		poses.push_back(current_pose_world_frame);

		get_current_frame_in_world_frame(current_frame_in_world_frame,
												current_pose_world_frame);
	}

	void write_g2o_file(const char* file_path){
		ofstream file_write(file_path);
		string vertex_type = "VERTEX_SE2";
		string edge_type = "EDGE_SE2";
		string upper_triangular_info_matrix = "1000.0 0.0 0.0 1000.0 0.0 1000.0";
		for(int i=0; i<poses.size(); ++i){
			file_write << vertex_type << " " << poses[i].id << " "<< poses[i].x << " "
						<< poses[i].y << " " << poses[i].theta_z << endl;
		}

		for(int i=0; i<edges.size(); ++i){
			file_write << edge_type << " " << edges[i].from_id  << " " << edges[i].to_id 
						<< " " << edges[i].delta_x << " " << edges[i].delta_y << " "
						<< edges[i].delta_theta << " " << upper_triangular_info_matrix
						<< endl;
		}
		
		file_write << "FIX 0" << endl;

		file_write.close();
	}
};

#endif