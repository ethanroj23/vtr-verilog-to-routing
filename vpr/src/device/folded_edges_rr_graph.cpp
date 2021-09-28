#include "vtr_assert.h"
#include "folded_edges_rr_graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include "globals.h"
#include "vtr_time.h"

#define FOLDED_RR_GRAPH_USING_EDGES


FoldedEdgesRRGraph::FoldedEdgesRRGraph(const t_rr_graph_storage& node_storage) : node_storage_(node_storage){
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

std::ifstream read_file_2("/home/ethan/rr_graphs/legacy_node_to_x_y_pattern.txt");
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
  float FoldedEdgesRRGraph::node_C(RRNodeId legacy_node) const {
    if (!built) return node_storage_.node_C(legacy_node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].C;
  }

  /* Get the resistance of a routing resource node. */
  float FoldedEdgesRRGraph::node_R(RRNodeId legacy_node) const {
    if (!built) return node_storage_.node_R(legacy_node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].R;
  }
  

void FoldedEdgesRRGraph::build_graph(){
    // Begin timing 
    vtr::ScopedStartFinishTimer timer("Build FoldedEdgesRRGraph");

    // Clear all data structures of any previous data

    node_first_edge_id_.clear();
    node_edges_idx_.clear();
    edge_src_node_.clear();


    // Resize legacy_node_to_x_y_ so that it fits every node

    // Resize node_first_edge_id_ so that it fits every node plus one for the dummy node
    if (node_first_edge_id_.size() < node_storage_.size()){
            node_first_edge_id_.resize((size_t) node_storage_.size()+1);
    }
    // Resize node_edges_idx_ so that it fits every node
    if (node_edges_idx_.size() < node_storage_.size()){
            node_edges_idx_.resize((size_t) node_storage_.size());
    }
    // Resize edge_src_node_ so that it fits every edge
    if (edge_src_node_.size() < node_storage_.edge_count()){
            edge_src_node_.resize((size_t) node_storage_.edge_count());
    }

    
    

    /*------------------------------------------------*/
    /*--------------THIRD LOOP OVER TILES-------------*/
    /*------------------------------------------------*/



    // search for and set all starting_node_id_ values
    int edge_starting_idx = 0;
    for (size_t idx = 0; idx < node_storage_.size(); idx++) { 
            std::vector<int16_t> edge_pattern_list;
            RRNodeId id = RRNodeId(idx);  
            node_first_edge_id_[id] = RREdgeId(edge_starting_idx); 
            std::vector<int16_t> edges_in_node;
            bool first_edge = true;
            for (RREdgeId from_edge : node_storage_.edge_range(id)) { // --- start  each node ---
                if (first_edge){
                    node_first_edge_id_[id] = from_edge; // only set first
                    first_edge = false;
                }
                RRNodeId sink_node = node_storage_.edge_sink_node(from_edge);
                int8_t switch_id = node_storage_.edge_switch(from_edge);
                int16_t edge_dx = node_storage_.node_xlow(sink_node) - node_storage_.node_xlow(id);
                int16_t edge_dy = node_storage_.node_ylow(sink_node) - node_storage_.node_ylow(id);
                int16_t id_diff = (size_t) id - (size_t) sink_node;
                FoldedEdgePattern edge_pattern = {
                    id_diff,
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
                edges_in_node.push_back(edge_data_idx);
                //node_edge_list_[id].push_back(edge_data_idx); // add edge data index to node edge list

                edge_src_node_[from_edge] = id; // set edge legacy source node
                edge_starting_idx++;
            } // --- end each node ---
            if (first_edge) node_first_edge_id_[id] = RREdgeId(edge_starting_idx); // only set first



            // Search for edge_pattern in edge_pattern_data_ vector
            bool edge_pattern_in_data = false;
            for (size_t i = 0; i < edge_pattern_data_.size(); i++){
                if (edge_pattern_data_[i] == edges_in_node){
                    // found it
                    edge_pattern_in_data = true;
                }
            }
            if (!edge_pattern_in_data){ // Add edge pattern to edge_pattern_data_ since it was not found there yet
                edge_pattern_data_.push_back(edges_in_node);
            }

            // Check if element was found in edge_pattern_data_
            auto it = std::find(edge_pattern_data_.begin(), edge_pattern_data_.end(), edges_in_node);
            VTR_ASSERT(it != edge_pattern_data_.end());

            int16_t edge_pattern_data_idx = std::distance(edge_pattern_data_.begin(), it);
            edge_pattern_list.push_back(edge_pattern_data_idx); // Add edge pattern data index and edge index
            node_edges_idx_[id] = edge_pattern_data_idx;
        }

    edges_size_ = edge_src_node_.size(); // set total number of edges
    RRNodeId dummy_node = RRNodeId(node_storage_.size());
    node_first_edge_id_[dummy_node] = RREdgeId(edge_starting_idx); // only set first
    
    
    //for (int i=0; i<node_patterns_.size(); i++){
    //    std::sort(node_patterns_[i].begin(), node_patterns_[i].end());
    //}
    built = true;
    size_ = node_storage_.size();

    std::cout << "Flat Representation: (including edges)" <<node_storage_memory_used()/1024/1024.0 << " MiB" << "\n";
    std::cout << "This Folded Representation: (including edges)" <<memory_used()/1024/1024.0 << " MiB" << "\n";
    std::cout << "Done\n";

    RRNodeId rand_node = edge_src_node(RREdgeId(124));

    /* Don't verify for now */
    verify_folded_rr_graph();
}

void FoldedEdgesRRGraph::verify_folded_rr_graph(){
    vtr::ScopedStartFinishTimer timer("Verify FoldedEdgesRRGraph");
    for (size_t idx = 0; idx < edges_size_; idx++) {   
        RREdgeId id = RREdgeId(idx);
        VTR_ASSERT(node_storage_.edge_src_node(id) == edge_src_node(id));
        VTR_ASSERT(node_storage_.edge_sink_node(id) == edge_sink_node(id));      
    }
    for (size_t idx = 0; idx < size(); idx++) {   
        RRNodeId id = RRNodeId(idx);    
        //VTR_ASSERT(node_storage_.edge_range(id).begin() == edge_range(id).begin());      
        //VTR_ASSERT(node_storage_.edge_range(id).end() == edge_range(id).end());      
        VTR_ASSERT(node_storage_.first_edge(id) == first_edge(id));      
        VTR_ASSERT(node_storage_.last_edge(id) == last_edge(id));      
    }
}

short FoldedEdgesRRGraph::node_ptc_num(RRNodeId id) const {
    return node_storage_.node_ptc_num(id);
}
short FoldedEdgesRRGraph::node_pin_num(RRNodeId id) const {
    return node_storage_.node_pin_num(id);
}

short FoldedEdgesRRGraph::node_track_num(RRNodeId id) const {
    return node_storage_.node_track_num(id);
}
short FoldedEdgesRRGraph::node_class_num(RRNodeId id) const {
    return node_storage_.node_class_num(id);
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