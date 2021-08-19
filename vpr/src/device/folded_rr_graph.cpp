#include "vtr_assert.h"
#include "folded_rr_graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include "globals.h"
#include "vtr_time.h"



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


  /* Get the capacitance of a routing resource node. This function is inlined for runtime optimization. */
  float FoldedRRGraph::node_C(RRNodeId node) const {
    if (!built) return node_storage_.node_C(node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(node)].C;
  }

  /* Get the resistance of a routing resource node. This function is inlined for runtime optimization. */
  float FoldedRRGraph::node_R(RRNodeId node) const {
    if (!built) return node_storage_.node_R(node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(node)].R;
  }
  

void FoldedRRGraph::initialize_folded_rr_graph(){
    // empty out the data structures
    /*all_node_patterns_.clear();
    rr_node_id_to_x_y_idx_.resize(node_storage_.size());
    for (int i=0; i<node_storage_.size(); i++){
        //initialize empty
        rr_node_id_to_x_y_idx_[RRNodeId(i)] = {-1, -1, -1};
    }*/
}

void FoldedRRGraph::add_empty_pattern(){
    //FoldedNodePattern pattern_array = { -1, -1, NUM_RR_TYPES, -1, Direction::NONE, "EMPTY", 0, 0, "EMPTY"};
    //all_node_patterns_.push_back(pattern_array);
}

void FoldedRRGraph::build_folded_rr_graph(){
    vtr::ScopedStartFinishTimer timer("Build FoldedRRGraph");
    //std::vector<std::vector<FoldedTilePattern>> tile_patterns; // Every tile has a starting_node_id and node_patterns_idx

    //std::vector<std::vector<int>> node_patterns; // Every Tile Type has a set of FoldedNodePatterns

    //std::vector<FoldedNodePattern> node_pattern_data;
    tile_patterns.clear();
    node_pattern_data.clear();
    node_patterns_.clear();
    remapped_ids_.clear();
    node_to_x_y_.clear();
    node_type_.clear();

    std::vector<std::string> temp_node_patterns;
    std::vector<std::vector<std::vector<RRNodeId>>> ids_in_tile; // 
    // dx, dy, type, capacity, direction, side, C, R, segment

    if (node_to_x_y_.size() < node_storage_.size()){
            node_to_x_y_.resize((size_t) node_storage_.size());
        }
    if (node_type_.size() < node_storage_.size()){
            node_type_.resize((size_t) node_storage_.size());
        }
    for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
        RRNodeId id = RRNodeId(idx);
        int16_t x = node_storage_.node_xlow(id);
        int16_t y = node_storage_.node_ylow(id);
        int16_t dx = node_storage_.node_xhigh(id) - x;
        int16_t dy = node_storage_.node_yhigh(id) - y;
        node_to_x_y_[id] = { x, y };
        t_rr_type current_type = node_storage_.node_type(id);
        node_type_[id] = current_type;
        
        FoldedNodePattern node_pattern = { 
                                            node_storage_.node_cost_index(id),
                                            node_storage_.node_rc_index(id),
                                            dx,
                                            dy,
                                            (uint16_t) node_storage_.node_capacity(id),
                                            {Direction::NUM_DIRECTIONS}

        };
        if (current_type== CHANX || current_type == CHANY){
                    node_pattern.dir_side_.direction_ = node_storage_.node_direction(id);
        }

        FoldedTilePattern tile_pattern = {RRNodeId(-1), -1};
                                        
        if (tile_patterns.size() < (size_t) x+1){
            ids_in_tile.resize(x+1);
            tile_patterns.resize(x+1);
        }
        if (tile_patterns[x].size() < (size_t) y+1){
            tile_patterns[x].resize(y+1);
            ids_in_tile[x].resize(y+1);
        }
        
        tile_patterns[x][y] = tile_pattern;
        ids_in_tile[x][y].push_back(id);
        if ( idx < (size_t) tile_pattern.starting_node_id_ || tile_pattern.starting_node_id_ == RRNodeId(-1) ){
            tile_patterns[x][y].starting_node_id_ = id;
        }


        bool node_pattern_in_data = false;
        for (size_t i = 0; i < node_pattern_data.size(); i++){
            if (node_pattern_data[i] == node_pattern){
                // found it
                node_pattern_in_data = true;
            }
        }
        if (!node_pattern_in_data){ // add node pattern to node_pattern_data
            node_pattern_data.push_back(node_pattern);
        }


    } 
    int current_remapped_id = 0;
    // search for and set all starting_node_id_ values
    for (size_t x = 0; x < ids_in_tile.size(); x++){ // iterate over x
        if (tile_patterns.size() < x+1){
            ids_in_tile.resize(x+1);
            tile_patterns.resize(x+1);
        }
        for (size_t y = 0; y < ids_in_tile[x].size(); y++){ // iterate over y (For every tile)
            if (tile_patterns[x].size() < y+1){
                tile_patterns[x].resize(y+1);
                ids_in_tile[x].resize(y+1);
            }
            
            sort(ids_in_tile[x][y].begin(), ids_in_tile[x][y].end());
            if (ids_in_tile[x][y].size() == 0){
                continue; // there are no nodes in this tile, so skip it
            }

            std::vector<int16_t> node_pattern_list;
            // Iterate over every node found in the tile and create a node_pattern_list of indexes to node_pattern_data for each node
            for (auto id : ids_in_tile[x][y]){
                // iterate until the previous id is not one less than the current id, then create a new node_pattern_list
                if (remapped_ids_.size() < (size_t) id + 1){
                    remapped_ids_.resize((size_t) id + 1);
                }
                remapped_ids_[id] = RRNodeId(current_remapped_id);
                current_remapped_id++;
                FoldedNodePattern node_pattern = { 
                                                            (int16_t) node_storage_.node_cost_index(id),
                                                            (int16_t) node_storage_.node_rc_index(id),
                                                            (int16_t) (node_storage_.node_xhigh(id) - node_storage_.node_xlow(id)),
                                                            (int16_t) (node_storage_.node_yhigh(id) - node_storage_.node_ylow(id)),
                                                            (uint16_t) node_storage_.node_capacity(id),
                                                            {Direction::NUM_DIRECTIONS}

                        };
                if (node_type_[id] == CHANX || node_type_[id] == CHANY){
                    node_pattern.dir_side_.direction_ = node_storage_.node_direction(id);
                }
                auto it = std::find(node_pattern_data.begin(), node_pattern_data.end(), node_pattern);
                // Check if element was found in node_pattern_data
                VTR_ASSERT(it != node_pattern_data.end());

                int16_t node_pattern_data_idx = std::distance(node_pattern_data.begin(), it);
                node_pattern_list.push_back(node_pattern_data_idx);
            }

            // find the node_pattern_list in node_patterns. If it is not found, insert it.
            auto it = std::find(node_patterns_.begin(), node_patterns_.end(), node_pattern_list);
            int16_t node_pattern_idx = -1;
            if (it != node_patterns_.end()){ // node_pattern_list found
                node_pattern_idx = std::distance(node_patterns_.begin(), it);
            }
            else{ //node_pattern_list not found
                node_patterns_.push_back(node_pattern_list);
                node_pattern_idx = node_patterns_.size() - 1;
            }
            tile_patterns[x][y].node_patterns_idx_ = node_pattern_idx;            

            tile_patterns[x][y].starting_node_id_ = remapped_ids_[ids_in_tile[x][y][0]];


        }

    }

    /*for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
        RRNodeId id = RRNodeId(idx);
        VTR_ASSERT(node_type(id) == node_storage_.node_type(id));
    }*/


    std::cout << "Done\n";
    built = true;
    size_ = node_storage_.size();
}
