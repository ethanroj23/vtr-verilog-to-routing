#include "vtr_assert.h"
#include "folded_edges_1_rr_graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include "globals.h"
#include "vtr_time.h"

#define FOLDED_RR_GRAPH_USING_EDGES


FoldedEdges1RRGraph::FoldedEdges1RRGraph(const t_rr_graph_storage& node_storage) : node_storage_(node_storage){
}

  /* Get the capacitance of a routing resource node.*/
  float FoldedEdges1RRGraph::node_C(RRNodeId legacy_node) const {
    if (!built) return node_storage_.node_C(legacy_node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].C;
  }

  /* Get the resistance of a routing resource node. */
  float FoldedEdges1RRGraph::node_R(RRNodeId legacy_node) const {
    if (!built) return node_storage_.node_R(legacy_node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].R;
  }
  

void FoldedEdges1RRGraph::build_graph(){
    // Begin timing 
    vtr::ScopedStartFinishTimer timer("Build FoldedEdges1RRGraph");

    // Clear all data structures of any previous data
    node_first_edge_id_.clear();
    node_edges_idx_.clear();
    edge_src_node_.clear();

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


    std::map<std::string, int> temp_edge_patterns {};
    std::map<std::string, int> temp_edge_pattern_data {};

    int edge_data_idx = 0;
    int edge_pattern_data_idx = 0;
    int edge_starting_idx = 0;
    for (size_t idx = 0; idx < node_storage_.size(); idx++) { 
            std::vector<int16_t> edge_pattern_list;
            RRNodeId id = RRNodeId(idx);  
            node_first_edge_id_[id] = RREdgeId(edge_starting_idx); 
            std::vector<int32_t> edges_in_node;
            std::string edges_in_node_string = "";
            bool first_edge = true;
            for (RREdgeId from_edge : node_storage_.edge_range(id)) { // --- start  each node ---
                if (first_edge){
                    node_first_edge_id_[id] = from_edge; // only set first
                    first_edge = false;
                }
                RRNodeId sink_node = node_storage_.edge_sink_node(from_edge);
                int8_t switch_id = node_storage_.edge_switch(from_edge);
                int32_t id_diff = (size_t) id - (size_t) sink_node;
                FoldedEdgePattern edge_pattern = {
                    id_diff,
                    switch_id
                };

                std::string edge_pattern_string = std::to_string(id_diff)+"_"+std::to_string(switch_id);

                 // insert into map if not in map
                if (!(temp_edge_patterns.count(edge_pattern_string)>0)){
                    temp_edge_patterns[edge_pattern_string] = edge_data_idx;
                    edge_data_idx++;
                    edge_data_.push_back(edge_pattern);
                }

                auto cur_pattern_idx = temp_edge_patterns[edge_pattern_string];

                edges_in_node.push_back(cur_pattern_idx);
                edges_in_node_string += std::to_string(cur_pattern_idx) + "_";

                edge_src_node_[from_edge] = id; // set edge legacy source node
                edge_starting_idx++;
            } // --- end each node ---
            if (first_edge) node_first_edge_id_[id] = RREdgeId(edge_starting_idx); // only set first


            // insert into map if not in map
            if (!(temp_edge_pattern_data.count(edges_in_node_string)>0)){
                temp_edge_pattern_data[edges_in_node_string] = edge_pattern_data_idx;
                edge_pattern_data_idx++;
                edge_pattern_data_.push_back(edges_in_node);
            }

            int cur_edge_pattern_idx = temp_edge_pattern_data[edges_in_node_string];

            edge_pattern_list.push_back(cur_edge_pattern_idx); // Add edge pattern data index and edge index
            node_edges_idx_[id] = cur_edge_pattern_idx;
        }

    edges_size_ = edge_src_node_.size(); // set total number of edges
    RRNodeId dummy_node = RRNodeId(node_storage_.size());
    node_first_edge_id_[dummy_node] = RREdgeId(edge_starting_idx); // only set first
    
    built = true;
    size_ = node_storage_.size();

    float flat_mem = node_storage_memory_used()/1024/1024.0;
    float folded_mem = memory_used()/1024/1024.0;

    //std::cout << "Folded/Flat = " << folded_mem << "/" << flat_mem << " = " << folded_mem / flat_mem << "(MiB)" << "\n";

    printf("Folded/Flat = %.2f/%.2f(MiB) = %.2f%%\n", folded_mem, flat_mem, 100.0 * folded_mem / flat_mem);


    //RRNodeId rand_node = edge_src_node(RREdgeId(124));

    /* Don't verify for now */
    verify_folded_rr_graph();
}

void FoldedEdges1RRGraph::verify_folded_rr_graph(){
    vtr::ScopedStartFinishTimer timer("Verify FoldedEdges1RRGraph");
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

t_edge_size FoldedEdges1RRGraph::num_configurable_edges(const RRNodeId& legacy_node) const {
    VTR_ASSERT(!node_first_edge_id_.empty());

    const auto& device_ctx = g_vpr_ctx.device();
    auto first_id = size_t(node_first_edge_id_[legacy_node]);
    auto last_id = size_t((&node_first_edge_id_[legacy_node])[1]);
    for (size_t idx = first_id; idx < last_id; ++idx) {
        auto switch_idx = edge_switch(RREdgeId(idx));
        if (!device_ctx.rr_switch_inf[switch_idx].configurable()) {
            return idx - first_id;
        }
    }

    return last_id - first_id;
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