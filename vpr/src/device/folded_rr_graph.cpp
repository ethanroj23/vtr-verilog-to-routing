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
    // Begin timing 
    vtr::ScopedStartFinishTimer timer("Build FoldedRRGraph");

    // Clear all data structures of any previous data
    tile_patterns_.clear();
    node_pattern_data_.clear();
    node_patterns_.clear();
    remapped_ids_.clear();
    node_to_x_y_.clear();
    node_first_edge_id_.clear();

    // Store every id in each tile (x, y)
    std::vector<std::vector<std::vector<RRNodeId>>> ids_in_tile;

    // Resize node_to_x_y_ so that it fits every node
    if (node_to_x_y_.size() < node_storage_.size()){
            node_to_x_y_.resize((size_t) node_storage_.size());
    }
    // Resize node_first_edge_id_ so that it fits every node
    if (node_first_edge_id_.size() < node_storage_.size()){
            node_first_edge_id_.resize((size_t) node_storage_.size());
    }

    // Iterate over every RRNodeId and find node_patterns
    for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
        RRNodeId id = RRNodeId(idx);
        int16_t x = node_storage_.node_xlow(id);
        int16_t y = node_storage_.node_ylow(id);
        int16_t dx = node_storage_.node_xhigh(id) - x;
        int16_t dy = node_storage_.node_yhigh(id) - y;
        node_to_x_y_[id] = { x, y };
        t_rr_type current_type = node_storage_.node_type(id);

        FoldedNodePattern node_pattern = { 
                                            node_storage_.node_cost_index(id),
                                            node_storage_.node_rc_index(id),
                                            dx,
                                            dy,
                                            current_type,//edge_patterns,
                                            (uint16_t) node_storage_.node_capacity(id),
                                            {Direction::NUM_DIRECTIONS}

        };
        // set direction if using CHANX or CHANY
        if (current_type== CHANX || current_type == CHANY){
                    node_pattern.dir_side_.direction_ = node_storage_.node_direction(id);
        }
        // set sides if using IPIN or OPIN
        if (current_type== IPIN || current_type == OPIN){
                for (auto side : SIDES){ // iterate over SIDES to find the side of the current node
                    if (strcmp(SIDE_STRING[side],node_storage_.node_side_string(id))==0){
                        node_pattern.dir_side_.sides_ = side;
                    }
                }
        }

        // initialize FoldedTilePattern
        FoldedTilePattern tile_pattern = {RRNodeId(-1), -1, -1};

        // Resize tile_patterns_ to fit every x             
        if (tile_patterns_.size() < (size_t) x+1){
            ids_in_tile.resize(x+1);
            tile_patterns_.resize(x+1);
        }
        // Resize tile_patterns_ to fit every y          
        if (tile_patterns_[x].size() < (size_t) y+1){
            tile_patterns_[x].resize(y+1);
            ids_in_tile[x].resize(y+1);
        }
        
        // add current tile_pattern to tile_patterns_ at the appropriate (x, y)
        tile_patterns_[x][y] = tile_pattern;
        ids_in_tile[x][y].push_back(id);

        // Set starting_node_id_ if this idx is less than the previous starting node id or the starting node id has not been set yet
        if ( idx < (size_t) tile_pattern.starting_node_id_ || tile_pattern.starting_node_id_ == RRNodeId(-1) ){
            tile_patterns_[x][y].starting_node_id_ = id;
        }


        // Search for node_pattern in node_pattern_data_ vector
        bool node_pattern_in_data = false;
        for (size_t i = 0; i < node_pattern_data_.size(); i++){
            if (node_pattern_data_[i] == node_pattern){
                // found it
                node_pattern_in_data = true;
            }
        }
        if (!node_pattern_in_data){ // Add node pattern to node_pattern_data_ since it was not found there yet
            node_pattern_data_.push_back(node_pattern);
        }


    } 
    int current_remapped_id = 0;
    
    /*------------------------------------------------*/
    /*-------------SECOND LOOP OVER TILES-------------*/
    /*------------------------------------------------*/

    // search for and set all starting_node_id_ values
    for (size_t x = 0; x < ids_in_tile.size(); x++){ // iterate over x
        if (tile_patterns_.size() < x+1){ // Resize if necessary
            ids_in_tile.resize(x+1);
            tile_patterns_.resize(x+1);
        }
        for (size_t y = 0; y < ids_in_tile[x].size(); y++){ // iterate over y (For every tile)
            if (tile_patterns_[x].size() < y+1){ // Resize if necessary
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

                

                t_rr_type current_type = node_storage_.node_type(id);
                FoldedNodePattern node_pattern = { 
                                                            (int16_t) node_storage_.node_cost_index(id),
                                                            (int16_t) node_storage_.node_rc_index(id),
                                                            (int16_t) (node_storage_.node_xhigh(id) - node_storage_.node_xlow(id)),
                                                            (int16_t) (node_storage_.node_yhigh(id) - node_storage_.node_ylow(id)),
                                                            current_type,
                                                            (uint16_t) node_storage_.node_capacity(id),
                                                            {Direction::NUM_DIRECTIONS}
                        };
                // set direction if using CHANX or CHANY
                if (current_type == CHANX || current_type == CHANY){
                    node_pattern.dir_side_.direction_ = node_storage_.node_direction(id);
                }

                // set sides if using IPIN or OPIN
                if (current_type== IPIN || current_type == OPIN){
                    for (auto side : SIDES){ // iterate over SIDES to find the side of the current node
                        if (strcmp(SIDE_STRING[side],node_storage_.node_side_string(id))==0){
                            node_pattern.dir_side_.sides_ = side;
                        }
                    }
                }

                // Check if element was found in node_pattern_data_
                auto it = std::find(node_pattern_data_.begin(), node_pattern_data_.end(), node_pattern);
                VTR_ASSERT(it != node_pattern_data_.end());

                int16_t node_pattern_data_idx = std::distance(node_pattern_data_.begin(), it);
                node_pattern_list.push_back(node_pattern_data_idx); // Add node pattern data index and edge index
            }

            // Find the node_pattern_list in node_patterns. If it is not found, insert it.
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



    /* // Attempt to find Subsets within node patterns (currently not using this)
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
    */
    

    /*------------------------------------------------*/
    /*--------------THIRD LOOP OVER TILES-------------*/
    /*------------------------------------------------*/

    // search for and set all starting_node_id_ values
    int edge_starting_idx = 0;
    for (size_t x = 0; x < ids_in_tile.size(); x++){ // iterate over x
        if (tile_patterns_.size() < x+1){ // Resize if necessary
            ids_in_tile.resize(x+1);
            tile_patterns_.resize(x+1);
        }
        for (size_t y = 0; y < ids_in_tile[x].size(); y++){ // iterate over y (For every tile)
            if (tile_patterns_[x].size() < y+1){ // Resize if necessary
                tile_patterns_[x].resize(y+1);
                ids_in_tile[x].resize(y+1);
            }
            
            sort(ids_in_tile[x][y].begin(), ids_in_tile[x][y].end());
            if (ids_in_tile[x][y].size() == 0){
                continue; // there are no nodes in this tile, so skip it
            }

            std::vector<int16_t> edge_pattern_list;
            // Iterate over every node found in the tile and create a node_pattern_list of indexes to node_pattern_data_ for each tile
            for (auto id : ids_in_tile[x][y]){
                // iterate until the previous id is not one less than the current id, then create a new node_pattern_list

                
                node_first_edge_id_[remapped_ids_[id]] = RREdgeId(edge_starting_idx); 
                int current_edge_starting_idx = edge_starting_idx;
                std::vector<int16_t> edge_patterns;
                for (RREdgeId from_edge : node_storage_.edge_range(id)) {
                    RRNodeId sink_node = node_storage_.edge_sink_node(from_edge);
                    int8_t switch_id = node_storage_.edge_switch(from_edge);
                    int16_t offset = node_offset(sink_node);
                    int16_t edge_dx = node_storage_.node_xlow(sink_node) - node_storage_.node_xlow(id);
                    int16_t edge_dy = node_storage_.node_ylow(sink_node) - node_storage_.node_ylow(id);
                    FoldedEdgePattern edge_pattern = {
                        edge_dx, // diff of the x position
                        edge_dy, // diff of the x position
                        offset,
                        switch_id
                    };

                    // add FoldedEdgePattern to edge_data_ if it is not there. Then get the edge_data_idx
                    auto it = std::find(edge_data_.begin(), edge_data_.end(), edge_pattern);
                    int16_t edge_data_idx = -1;
                    if (it != edge_data_.end()){ // edge_pattern found
                        edge_data_idx = std::distance(edge_data_.begin(), it);
                    }
                    else{ //edge_pattern not found
                        edge_data_.push_back(edge_pattern);
                        edge_data_idx = edge_data_.size() - 1;
                    }
                    edge_patterns.push_back(edge_data_idx);

                    edge_starting_idx++; // increment edge_starting_idx for every edge
                }
                if (current_edge_starting_idx == edge_starting_idx){
                    node_first_edge_id_[remapped_ids_[id]] = RREdgeId(-1);
                }


                // Search for edge_pattern in edge_pattern_data_ vector
                bool edge_pattern_in_data = false;
                for (size_t i = 0; i < edge_pattern_data_.size(); i++){
                    if (edge_pattern_data_[i] == edge_patterns){
                        // found it
                        edge_pattern_in_data = true;
                    }
                }
                if (!edge_pattern_in_data){ // Add edge pattern to edge_pattern_data_ since it was not found there yet
                    edge_pattern_data_.push_back(edge_patterns);
                }

                // Check if element was found in edge_pattern_data_
                auto it = std::find(edge_pattern_data_.begin(), edge_pattern_data_.end(), edge_patterns);
                VTR_ASSERT(it != edge_pattern_data_.end());

                int16_t edge_pattern_data_idx = std::distance(edge_pattern_data_.begin(), it);
                edge_pattern_list.push_back(edge_pattern_data_idx); // Add edge pattern data index and edge index
            }

            // Find the edge_pattern_list in edge_patterns. If it is not found, insert it.
            auto it = std::find(edge_patterns_.begin(), edge_patterns_.end(), edge_pattern_list);
            int16_t edge_pattern_idx = -1;
            if (it != edge_patterns_.end()){ // edge_pattern_list found
                edge_pattern_idx = std::distance(edge_patterns_.begin(), it);
            }
            else{ //edge_pattern_list not found
                edge_patterns_.push_back(edge_pattern_list);
                edge_pattern_idx = edge_patterns_.size() - 1;
            }
            tile_patterns_[x][y].edge_patterns_idx_ = edge_pattern_idx;
        }
    }











    // iterate over every node and look for edge patterns
    // for the edge pattern, store dx, dy and offset into the tile
    
    for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
            RRNodeId id = RRNodeId(idx);
            std::vector<FoldedEdgePattern> edge_patterns;
            for (RREdgeId from_edge : node_storage_.edge_range(id)) {
            RRNodeId sink_node = node_storage_.edge_sink_node(from_edge);
            int8_t switch_id = node_storage_.edge_switch(from_edge);
            int16_t offset = node_offset(sink_node);
            int16_t edge_dx = node_storage_.node_xlow(sink_node) - node_storage_.node_xlow(id);
            int16_t edge_dy = node_storage_.node_ylow(sink_node) - node_storage_.node_ylow(id);
            FoldedEdgePattern edge_pattern = {
                edge_dx, // diff of the x position
                edge_dy, // diff of the x position
                offset,
                switch_id
            };
           // VTR_LOG( "dx:%d, dy:%d, switch:%d\n", edge_dx, edge_dy, switch_id);
            edge_patterns.push_back(edge_pattern);
            }
    }
    

    
    
    //for (int i=0; i<node_patterns_.size(); i++){
    //    std::sort(node_patterns_[i].begin(), node_patterns_[i].end());
    //}
    built = true;
    size_ = node_storage_.size();

    std::cout << "Flat Representation: (including edges)" <<node_storage_memory_used()/1024/1024.0 << " MiB" << "\n";
    std::cout << "This Folded Representation: (including edges)" <<memory_used()/1024/1024.0 << " MiB" << "\n";
    std::cout << "Done\n";

    RRNodeId first_node = get_edge_src_node(RREdgeId(0));
    RRNodeId rand_node = get_edge_src_node(RREdgeId(124));
    auto rand_edges = edge_range(rand_node);
    RRNodeId rand_dest = get_edge_dest_node(RREdgeId(124));
    short rand_switch = get_edge_switch(RREdgeId(124));

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
        
        // verify edges
        auto edges = edge_range(id);
        int edge_idx = 0;
        for (RREdgeId edge : node_storage_.edge_range(id)) {
            RRNodeId legacy_src_node = id;
            auto legacy_dest_node = node_storage_.edge_sink_node(edge);
            auto legacy_switch_id = node_storage_.edge_switch(edge);

            auto src_node = (RRNodeId)edges[edge_idx].src_node;
            auto dest_node = (RRNodeId)edges[edge_idx].dest_node;
            auto switch_id = edges[edge_idx].switch_id;

            VTR_ASSERT(legacy_src_node == src_node);
            VTR_ASSERT(legacy_dest_node == dest_node);
            VTR_ASSERT(legacy_switch_id == switch_id);

            edge_idx++;

        }
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