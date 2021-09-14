#include "vtr_assert.h"
#include "folded_rr_graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include "globals.h"
#include "vtr_time.h"

#define FOLDED_RR_GRAPH_USING_EDGES


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


  /* Get the capacitance of a routing resource node.*/
  float FoldedRRGraph::node_C(RRNodeId node) const {
    if (!built) return node_storage_.node_C(node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(node)].C;
  }

  /* Get the resistance of a routing resource node. */
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

    // Clear all data structures of any previous data
    tile_patterns_.clear();
    node_pattern_data_.clear();
    node_patterns_.clear();
    remapped_ids_.clear();
    node_to_x_y_.clear();

    //std::vector<std::string> temp_node_patterns;
    std::vector<std::vector<std::vector<RRNodeId>>> ids_in_tile;

    if (node_to_x_y_.size() < node_storage_.size()){
            node_to_x_y_.resize((size_t) node_storage_.size());
        }
    // iterate over every RRNodeId and find node_patterns
    for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
        RRNodeId id = RRNodeId(idx);
        int16_t x = node_storage_.node_xlow(id);
        int16_t y = node_storage_.node_ylow(id);
        int16_t dx = node_storage_.node_xhigh(id) - x;
        int16_t dy = node_storage_.node_yhigh(id) - y;
        node_to_x_y_[id] = { x, y };
        t_rr_type current_type = node_storage_.node_type(id);

        #ifdef FOLDED_RR_GRAPH_USING_EDGESS
        std::vector<FoldedEdgePattern> temp_edge_patterns;
        std::vector<FoldedEdgePattern> edge_patterns;
        for (RREdgeId from_edge : node_storage_.edge_range(id)) {
            RRNodeId sink_node = node_storage_.edge_sink_node(from_edge);
            int32_t edge_diff = (size_t) sink_node - (size_t) id;
            auto switch_id = node_storage_.edge_switch(from_edge);
            int16_t edge_dx = node_storage_.node_xlow(sink_node) - node_storage_.node_xlow(id);
            int16_t edge_dy = node_storage_.node_ylow(sink_node) - node_storage_.node_ylow(id);
            FoldedEdgePattern edge_pattern = {
                edge_dx, // diff of the x position
                edge_dy, // diff of the x position
                switch_id
            };
           // VTR_LOG( "dx:%d, dy:%d, switch:%d\n", edge_dx, edge_dy, switch_id);
            edge_patterns.push_back(edge_pattern);
        }
        #endif

        std::vector<FoldedEdgePattern> edge_patterns;
        for (RREdgeId from_edge : node_storage_.edge_range(id)) {
            RRNodeId sink_node = node_storage_.edge_sink_node(from_edge);
            int32_t edge_diff = (size_t) sink_node - (size_t) id;
            auto switch_id = node_storage_.edge_switch(from_edge);
            //int16_t offset = node_offset(sink_node);
            int16_t offset = 0;
            int16_t edge_dx = node_storage_.node_xlow(sink_node) - node_storage_.node_xlow(id);
            int16_t edge_dy = node_storage_.node_ylow(sink_node) - node_storage_.node_ylow(id);
            FoldedEdgePattern edge_pattern = {
                edge_dx, // diff of the x position
                edge_dy, // diff of the y position
                offset
            };
            // VTR_LOG( "dx:%d, dy:%d, switch:%d\n", edge_dx, edge_dy, switch_id);
            edge_patterns.push_back(edge_pattern);
        }

        FoldedNodePattern node_pattern = { 
                                            node_storage_.node_cost_index(id),
                                            node_storage_.node_rc_index(id),
                                            dx,
                                            dy,
                                            current_type,//edge_patterns,
                                            (uint16_t) node_storage_.node_capacity(id),
                                            {Direction::NUM_DIRECTIONS},
                                            edge_patterns

        };
        if (current_type== CHANX || current_type == CHANY){
                    node_pattern.dir_side_.direction_ = node_storage_.node_direction(id);
        }
        if (current_type== IPIN || current_type == OPIN){
                for (auto side : SIDES){ // iterate over SIDES to find the side of the current node
                    if (strcmp(SIDE_STRING[side],node_storage_.node_side_string(id))==0){
                        node_pattern.dir_side_.sides_ = side;
                    }
                }
        }

        FoldedTilePattern tile_pattern = {RRNodeId(-1), -1};
                                        
        if (tile_patterns_.size() < (size_t) x+1){
            ids_in_tile.resize(x+1);
            tile_patterns_.resize(x+1);
        }
        if (tile_patterns_[x].size() < (size_t) y+1){
            tile_patterns_[x].resize(y+1);
            ids_in_tile[x].resize(y+1);
        }
        
        tile_patterns_[x][y] = tile_pattern;
        ids_in_tile[x][y].push_back(id);
        if ( idx < (size_t) tile_pattern.starting_node_id_ || tile_pattern.starting_node_id_ == RRNodeId(-1) ){
            tile_patterns_[x][y].starting_node_id_ = id;
        }


        bool node_pattern_in_data = false;
        for (size_t i = 0; i < node_pattern_data_.size(); i++){
            if (node_pattern_data_[i] == node_pattern){
                // found it
                node_pattern_in_data = true;
            }
        }
        if (!node_pattern_in_data){ // add node pattern to node_pattern_data_
            node_pattern_data_.push_back(node_pattern);
        }


    } 
    int current_remapped_id = 0;
    
    // search for and set all starting_node_id_ values
    for (size_t x = 0; x < ids_in_tile.size(); x++){ // iterate over x
        if (tile_patterns_.size() < x+1){
            ids_in_tile.resize(x+1);
            tile_patterns_.resize(x+1);
        }
        for (size_t y = 0; y < ids_in_tile[x].size(); y++){ // iterate over y (For every tile)
            if (tile_patterns_[x].size() < y+1){
                tile_patterns_[x].resize(y+1);
                ids_in_tile[x].resize(y+1);
            }
            
            sort(ids_in_tile[x][y].begin(), ids_in_tile[x][y].end());
            if (ids_in_tile[x][y].size() == 0){
                continue; // there are no nodes in this tile, so skip it
            }

            std::vector<int16_t> node_pattern_list;
            // Iterate over every node found in the tile and create a node_pattern_list of indexes to node_pattern_data_ for each tile
            for (auto id : ids_in_tile[x][y]){
                // iterate until the previous id is not one less than the current id, then create a new node_pattern_list
                if (remapped_ids_.size() < (size_t) id + 1){
                    remapped_ids_.resize((size_t) id + 1);
                }
                remapped_ids_[id] = RRNodeId(current_remapped_id);
                current_remapped_id++;

                std::vector<FoldedEdgePattern> edge_patterns;
                for (RREdgeId from_edge : node_storage_.edge_range(id)) {
                    RRNodeId sink_node = node_storage_.edge_sink_node(from_edge);
                    int32_t edge_diff = (size_t) sink_node - (size_t) id;
                    auto switch_id = node_storage_.edge_switch(from_edge);
                    //int16_t offset = node_offset(sink_node);
                    int16_t offset = 0;
                    int16_t edge_dx = node_storage_.node_xlow(sink_node) - node_storage_.node_xlow(id);
                    int16_t edge_dy = node_storage_.node_ylow(sink_node) - node_storage_.node_ylow(id);
                    FoldedEdgePattern edge_pattern = {
                        edge_dx, // diff of the x position
                        edge_dy, // diff of the x position
                        offset
                    };
                    // VTR_LOG( "dx:%d, dy:%d, switch:%d\n", edge_dx, edge_dy, switch_id);
                    edge_patterns.push_back(edge_pattern);
                }
                t_rr_type current_type = node_storage_.node_type(id);
                FoldedNodePattern node_pattern = { 
                                                            (int16_t) node_storage_.node_cost_index(id),
                                                            (int16_t) node_storage_.node_rc_index(id),
                                                            (int16_t) (node_storage_.node_xhigh(id) - node_storage_.node_xlow(id)),
                                                            (int16_t) (node_storage_.node_yhigh(id) - node_storage_.node_ylow(id)),
                                                            current_type,
                                                            (uint16_t) node_storage_.node_capacity(id),
                                                            {Direction::NUM_DIRECTIONS},
                                                            edge_patterns
                        };
                if (current_type == CHANX || current_type == CHANY){
                    node_pattern.dir_side_.direction_ = node_storage_.node_direction(id);
                }
                if (current_type== IPIN || current_type == OPIN){
                for (auto side : SIDES){ // iterate over SIDES to find the side of the current node
                    if (strcmp(SIDE_STRING[side],node_storage_.node_side_string(id))==0){
                        node_pattern.dir_side_.sides_ = side;
                    }
                }
        }


                auto it = std::find(node_pattern_data_.begin(), node_pattern_data_.end(), node_pattern);
                // Check if element was found in node_pattern_data_
                VTR_ASSERT(it != node_pattern_data_.end());

                int16_t node_pattern_data_idx = std::distance(node_pattern_data_.begin(), it);
                node_pattern_list.push_back(node_pattern_data_idx);
            }

            //std::sort(node_pattern_list.begin(), node_pattern_list.end());
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
            tile_patterns_[x][y].node_patterns_idx_ = node_pattern_idx;            

            tile_patterns_[x][y].starting_node_id_ = remapped_ids_[ids_in_tile[x][y][0]];

        }
    }



    std::vector<std::array<int, 2>> temp_node_patterns;
    for (int i=0; i<node_patterns_.size(); i++){ // main one
        int largest=0;
        for (int j=0; j<node_patterns_.size(); j++){ // compare against these patterns
            if (node_patterns_[i].size() > node_patterns_[j].size()){
                continue;
            }
            if (i==j) continue; // dont compare against itself
            bool match = true;
            for(int k=0; k<node_patterns_[i].size(); k++){
                if (node_patterns_[i][k] != node_patterns_[j][k]){
                    match = false;
                }
            }
            if (node_patterns_[j].size() > largest && match){
                    if (largest != 0){ // this was not the first subset
                        temp_node_patterns.pop_back();
                    }
                    temp_node_patterns.push_back({i, j});
                    largest = node_patterns_[j].size();
            }
        }
    }
    


    // iterate over every node and look for edge patterns
    // for the edge pattern, store dx, dy and offset into the tile
    /*
    for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
            RRNodeId id = RRNodeId(idx);
            std::vector<FoldedEdgePattern> temp_edge_patterns;
            std::vector<FoldedEdgePattern> edge_patterns;
            for (RREdgeId from_edge : node_storage_.edge_range(id)) {
            RRNodeId sink_node = node_storage_.edge_sink_node(from_edge);
            int32_t edge_diff = (size_t) sink_node - (size_t) id;
            auto switch_id = node_storage_.edge_switch(from_edge);
            int16_t offset = node_offset(sink_node);
            int16_t edge_dx = node_storage_.node_xlow(sink_node) - node_storage_.node_xlow(id);
            int16_t edge_dy = node_storage_.node_ylow(sink_node) - node_storage_.node_ylow(id);
            FoldedEdgePattern edge_pattern = {
                edge_dx, // diff of the x position
                edge_dy, // diff of the x position
                offset
            };
           // VTR_LOG( "dx:%d, dy:%d, switch:%d\n", edge_dx, edge_dy, switch_id);
            edge_patterns.push_back(edge_pattern);
            }
            set_node_edges(id, edge_patterns);
    }
    */


    /*for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
        RRNodeId id = RRNodeId(idx);
        VTR_ASSERT(node_type(id) == node_storage_.node_type(id));
    }*/
/*
    node_storage_.for_each_edge(
        [&](RREdgeId edge, RRNodeId src, RRNodeId sink) {
            RRNodeId r_src = remapped_ids_[src];
            RRNodeId r_sink = remapped_ids_[sink];
            VTR_LOG( "%d -> %d  diff[%d]\n", r_src, r_sink, (size_t) r_sink - (size_t) r_src);
            FoldedEdgePattern cur_pattern = { node_offset(src),
                                    get_node_patterns_idx(src),
                                    node_offset(sink),
                                    get_node_patterns_idx(sink),
                                    node_storage_.edge_switch(edge)
                                    };

        // find the node_pattern_list in node_patterns. If it is not found, insert it.
        auto it = std::find(edge_patterns_.begin(), edge_patterns_.end(), cur_pattern);
        int16_t node_pattern_idx = -1;
        if (!(it != edge_patterns_.end())){ // node_pattern_list found
            edge_patterns_.push_back(cur_pattern);
        }

        edge_storage_.push_back(cur_pattern);
        
        });*/
    std::cout << "Flat Representation: (including edges)" <<node_storage_memory_used()/1024/1024.0 << " MiB" << "\n";
    std::cout << "This Folded Representation: (no edges)" <<memory_used()/1024/1024.0 << " MiB" << "\n";
    std::cout << "Done\n";
    
    //for (int i=0; i<node_patterns_.size(); i++){
    //    std::sort(node_patterns_[i].begin(), node_patterns_[i].end());
    //}
    built = true;
    size_ = node_storage_.size();
    verify_folded_rr_graph();
}

void FoldedRRGraph::verify_folded_rr_graph(){
    vtr::ScopedStartFinishTimer timer("Verify FoldedRRGraph");
    for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
        RRNodeId id = RRNodeId(idx);
        VTR_ASSERT(node_storage_.node_xlow(id) == node_xlow(id));
        VTR_ASSERT(node_storage_.node_xhigh(id) == node_xhigh(id));
        VTR_ASSERT(node_storage_.node_ylow(id) == node_ylow(id));
        VTR_ASSERT(node_storage_.node_yhigh(id) == node_yhigh(id));

        VTR_ASSERT(node_storage_.node_R(id) == node_R(id));
        VTR_ASSERT(node_storage_.node_C(id) == node_C(id));

        VTR_ASSERT(node_storage_.node_cost_index(id) == node_cost_index(id));
        VTR_ASSERT(node_storage_.node_type(id) == node_type(id));
        VTR_ASSERT(node_storage_.node_capacity(id) == node_capacity(id));
        

        if (node_type(id) == CHANX || node_type(id) == CHANY) 
            VTR_ASSERT(node_storage_.node_direction(id) == node_direction(id));
        
        if(node_type(id) == IPIN || node_type(id) == OPIN)
            VTR_ASSERT(strcmp(node_storage_.node_side_string(id),node_side_string(id))==0);

    }
}

#ifdef FOLDED_RR_GRAPH_USING_EDGES
struct EdgePattern { // 9 Bytes
        int16_t src_node_offset_ = -1; // 2 Bytes
        int16_t src_node_patterns_idx_ = -1; // 2 Bytes

        int16_t dest_node_offset_ = -1; // 2 Bytes
        int16_t dest_node_patterns_idx_ = -1; // 2 Bytes

        int8_t switch_id = -1; // 1 Byte
      }; 
#endif