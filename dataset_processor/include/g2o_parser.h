#ifndef G2O_PARSER_H
#define G2O_PARSER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

class g2oParser{
private:
	vector<vector<float>>& transforms;
	const char *g2o_file;
	const string keyword = "VERTEX_SE3:QUAT";
	const int word_count = 8;

private:
	vector<int> find_spaces(const string &line){
		char space = ' ';
		vector<int> index;
		for(int i=0; i<line.length(); ++i){
			if(line[i]==space){
				index.push_back(i);
			}
		}
		index.push_back(line.length());
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

	void print_transforms(void){
		for(vector<float> transform : transforms){
			for(size_t index=0; index<transform.size(); ++index){
				cout << transform[index] << " ";
			}
			cout << "\n";
		}
	}

public:
	g2oParser(const char *file, vector<vector<float>>& arg_transforms):
				g2o_file{file}, transforms{arg_transforms}{};

	void start_processing(void){
		parse_g2o_file();
		// print_transforms();
	}
};

#endif