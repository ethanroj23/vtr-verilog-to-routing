#include "vtr_assert.h"
#include "folded_rr_graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>



FoldedRRGraph::FoldedRRGraph(const t_rr_graph_storage& node_storage) : node_storage_(node_storage){
#ifdef BUILD_FROM_FILE
std::string current_line;
/* load in all_patterns_ from file */
std::ifstream read_file_1("/home/ethan/rr_graphs/all_node_patterns.txt");
while (getline (read_file_1, current_line)) {
    std::istringstream ss_1(current_line);
    std::string token_1;
    std::array<std::string, 9> current_array_1;
    int idx_1 = 0;
    while(std::getline(ss_1, token_1, ' ')) {
        current_array_1[idx_1] = token_1;
        idx_1++;
    }
    all_node_patterns_.push_back(current_array_1);
}
read_file_1.close();

/* load in rr_node_id_to_x_y_idx_ from file */

std::ifstream read_file_2("/home/ethan/rr_graphs/node_to_x_y_pattern.txt");
while (getline (read_file_2, current_line)) {
    std::istringstream ss(current_line);
    std::string token;
    std::array<int, 3> current_array;
    int idx = 0;
    while(std::getline(ss, token, ' ')) {
        current_array[idx] = std::stoi(token);
        idx++;
    }
    rr_node_id_to_x_y_idx_.push_back(current_array);
}
read_file_2.close();
#endif



}

void FoldedRRGraph::initialize_folded_rr_graph(){
    // empty out the data structures
    all_node_patterns_.clear();
    rr_node_id_to_x_y_idx_.resize(node_storage_.size());
    for (int i=0; i<node_storage_.size(); i++){
        //initialize empty
        rr_node_id_to_x_y_idx_[RRNodeId(i)] = {-1, -1, -1};
    }
}

void FoldedRRGraph::add_empty_pattern(){
    std::array<std::string, 9> pattern_array;
    pattern_array[0] = "EMPTY";
    pattern_array[1] = "EMPTY";
    pattern_array[2] = "EMPTY";
    pattern_array[3] = "EMPTY";
    pattern_array[4] = "EMPTY";
    pattern_array[5] = "EMPTY";
    pattern_array[6] = "EMPTY";
    pattern_array[7] = "EMPTY";
    pattern_array[8] = "EMPTY";
    all_node_patterns_.push_back(pattern_array);
}

void FoldedRRGraph::build_folded_rr_graph(){
    std::cout << "test\n";
    std::vector<std::string> temp_node_patterns;
    all_node_patterns_.clear();
    rr_node_id_to_x_y_idx_.resize(node_storage_.size());
    // dx, dy, type, capacity, direction, side, C, R, segment
    for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
        RRNodeId id = RRNodeId(idx);
        std::string dx = std::to_string(node_storage_.node_xhigh(id) - node_storage_.node_xlow(id));
        std::string dy = std::to_string(node_storage_.node_yhigh(id) - node_storage_.node_ylow(id));
        std::string node_type = node_storage_.node_type_string(id);
        std::string capacity = std::to_string(node_storage_.node_capacity(id));
        std::string direction = "NONE";
        if (node_type=="CHANX" || node_type=="CHANY"){
            direction = node_storage_.node_direction_string(id);
        }
        std::string side = "NONE";
        if (node_type=="IPIN" || node_type=="OPIN"){
            side = node_storage_.node_side_string(id);
        }
        std::string C = std::to_string(node_storage_.node_C(id));
        std::string R = std::to_string(node_storage_.node_R(id));
        std::string pattern = dx + " " + dy + " " + node_type + " " + capacity + " " + direction + " " + side + " " + C + " " + R;

        int index_of_pattern = -1;
        for(std::size_t i=0; i<temp_node_patterns.size(); i++) {
            if (pattern == temp_node_patterns[i]){
                index_of_pattern = i;
                break;
            }
        }
        if (index_of_pattern==-1){ // pattern not found
            temp_node_patterns.push_back(pattern);
            index_of_pattern = temp_node_patterns.size()-1;

            std::array<std::string, 9> pattern_array;
            pattern_array[0] = dx;
            pattern_array[1] = dy;
            pattern_array[2] = node_type;
            pattern_array[3] = capacity;
            pattern_array[4] = direction;
            pattern_array[5] = side;
            pattern_array[6] = C; //temp
            pattern_array[7] = R;
            pattern_array[8] = "t";
            all_node_patterns_.push_back(pattern_array);

        }



        rr_node_id_to_x_y_idx_[id][0] = node_storage_.node_xlow(id);
        rr_node_id_to_x_y_idx_[id][1] = node_storage_.node_ylow(id);
        rr_node_id_to_x_y_idx_[id][2] = index_of_pattern;



    } 

    std::cout << "Done\n";

}

