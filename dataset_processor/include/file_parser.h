#ifndef FILE_PARSER_H
#define FILE_PARSER_H

#include <iostream>
#include <dirent.h>
#include <string>
#include <algorithm>
#include <fstream>
#include <cctype>

using namespace std;

class ParseImages{
private:
	const char* rgb_folder;
	const char* depth_folder;
	vector<string>& rgb_images;
	vector<string>& depth_images;

private:
	int get_image_number(const string &str){
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
				list.push_back(get_image_number(pointer->d_name));
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

class FileParser{
private:
	const char* file;
	vector<vector<int>> lines;
	pair<int, int>& going;
	pair<int, int>& coming;

private:
	void find_spaces(const string &line, vector<int>& index){
		char space = ' ';
		int i=0;
		for(; i<line.length(); ++i){
			if(line[i]==space){
				index.push_back(i);
			}
		}
		index.push_back(i);
	}

	void extract_substring(const string &line, vector<int> &substring){
		vector<int> spaces;
		find_spaces(line, spaces);
		for(int i=1; i<3; ++i){
			string word{};
			for(int j=spaces[i]+1; j<spaces[i+1]; ++j){
				word += line[j];
			}
			substring.push_back(stoi(word));
		}
	}

	void parse_file(void){
		ifstream file_read(file);
		string line;
		if(file_read.is_open()){
			while(getline(file_read, line)){
				vector<int> int_line;
				extract_substring(line, int_line);
				lines.push_back(int_line);
			}
		}
	}

	void fill_trajectory(void){
		going = make_pair(lines[0][0], lines[0][1]);
		coming = make_pair(lines[1][0], lines[1][1]);
	}

public:
	FileParser(const char* arg_file, pair<int, int>& go, pair<int, int>& come): 
				file{arg_file}, going{go}, coming{come}{};

	void start_processing(void){
		parse_file();
		fill_trajectory();
	}
};

class MatchImagePair{
private:
	const vector<string>& rgb_images;
	const vector<string>& depth_images;
	const pair<int, int>& going;
	const pair<int, int>& coming;
	vector<pair<string, string>>& rgb_pairs;
	vector<pair<string, string>>& depth_pairs;

private:
	int find_image_index(const int index){
		const string keyword = to_string(index) + ".";
		for(int i=0; i<rgb_images.size(); ++i){
			size_t found = rgb_images[i].find(keyword);
			if(found!=std::string::npos){
				return i;
			}
		}
	}

	void generate_pairs(vector<pair<string, string>>& image_pairs, 
						const vector<string>& images){
		const int going_start = find_image_index(going.first);
		const int going_end = find_image_index(going.second);
		const int coming_start = find_image_index(coming.first);
		const int coming_end = find_image_index(coming.second);
		const float ratio = float(going_end-going_start+1)/(coming_end-coming_start+1);
		const int min_length = min(going_end-going_start+1, coming_end-coming_start+1);
		
		vector<string> going_images, coming_images;

		if(ratio>=1){
			for(int i=1; i<=min_length; ++i){
				coming_images.push_back(images[coming_start+i-1]);
				going_images.push_back(images[going_start+round(ratio*i)-1]);
			}
			reverse(coming_images.begin(), coming_images.end());
			for(int i=0; i<going_images.size(); ++i){
				image_pairs.push_back(make_pair(going_images[i], coming_images[i]));
			}
		}

		else{
			for(int i=1; i<=min_length; ++i){
				going_images.push_back(images[going_start+i-1]);
				coming_images.push_back(images[coming_start+round(ratio*i)-1]);
			}
			reverse(coming_images.begin(), coming_images.end());
			for(int i=0; i<going_images.size(); ++i){
				image_pairs.push_back(make_pair(going_images[i], coming_images[i]));
			}	
		}
	}

	static void print_image_pairs(const vector<pair<string, string>>& image_pairs){
		for(int i=0; i<image_pairs.size(); ++i){
			cout << image_pairs[i].first << endl << image_pairs[i].second << endl;
		}
	}

public:
	MatchImagePair(const vector<string>& rgb, 
					const vector<string>& depth,
					const pair<int ,int>& arg_going,
					const pair<int, int>& arg_coming,
					vector<pair<string, string>>& rgb_image_pairs,
					vector<pair<string, string>>& depth_image_pairs):
					rgb_images{rgb}, depth_images{depth},
					going{arg_going}, coming{arg_coming},
					rgb_pairs{rgb_image_pairs}, depth_pairs{depth_image_pairs}{};

	void start_processing(void){
		generate_pairs(rgb_pairs, rgb_images);
		generate_pairs(depth_pairs, depth_images);
		print_image_pairs(depth_pairs);
	}
	
	void write_image_pairs(void){
		ofstream file_write("image_pairs.txt");
		for(auto rgb_pair : rgb_pairs){
			file_write << rgb_pair.first << endl << rgb_pair.second << "\n---\n";
		}
		file_write.close();
	}
};

#endif