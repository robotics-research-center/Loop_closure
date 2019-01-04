#include "graph.h"

void generate_sub_trajectory(Pose& current_pose_world_frame,
								Affine3f& current_frame_in_world_frame, 
								float delta_x, float delta_y,
								float delta_theta, const int steps,
								vector<Pose>& poses, vector<Edge>& edges,
								const float noise){
	Pose new_pose_current_frame;
	for(int i=0; i<steps; ++i){
		if(noise != 0){
			add_noise(delta_x, delta_y, delta_theta, noise);
		}
		Edge edge(delta_x, delta_y, delta_theta, current_pose_world_frame.id,
					current_pose_world_frame.id+1);
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
}

void assemble_sub_trajectory(vector<Pose>& poses, vector<Edge>& edges,
								const float noise=0.0){
	Pose current_pose_world_frame = Pose{2, 2, 0};
	Affine3f current_frame_in_world_frame;
	get_current_frame_in_world_frame(current_frame_in_world_frame, 
										current_pose_world_frame);

	poses.push_back(current_pose_world_frame);

	float delta_x = 2;
	float delta_y = 0;
	float delta_theta_straight = 0;
	float delta_theta_curve = PI/6;
	const int steps_straight = 10;
	const int steps_curve = 6;

	generate_sub_trajectory(current_pose_world_frame, current_frame_in_world_frame,
							delta_x, delta_y, delta_theta_straight, steps_straight, 
							poses, edges, noise);
	generate_sub_trajectory(current_pose_world_frame, current_frame_in_world_frame,
							delta_x, delta_y, delta_theta_curve, steps_curve, 
							poses, edges, noise);
	generate_sub_trajectory(current_pose_world_frame, current_frame_in_world_frame,
							delta_x, delta_y, delta_theta_straight, steps_straight, 
							poses, edges, noise);

}

int main(int argc, char const *argv[]){
	if(argc != 3){
		fprintf(stdout, "Usage: ./relative_edges output.g2o noise\n");
		return 1;
	}

	Pose initial_pose{2, 2, 0};
	vector<Pose> correct_poses;
	vector<Edge> correct_edges;
	vector<Pose> noisy_poses;
	vector<Edge> noisy_edges;

	assemble_sub_trajectory(correct_poses, correct_edges, 0);
	assemble_sub_trajectory(noisy_poses, noisy_edges, stof(argv[2]));

	add_loop_closing_edges(correct_poses, noisy_edges);
	print_graph(noisy_poses, noisy_edges);
	plot_graph(correct_poses);
	plot_graph(noisy_poses);
	write_g2o_file(noisy_poses, noisy_edges, argv[1]);
	return 0;
}