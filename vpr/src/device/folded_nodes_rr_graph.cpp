#include "vtr_assert.h"
#include "folded_nodes_rr_graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>
#include "globals.h"
#include "vtr_time.h"

FoldedNodesRRGraph::FoldedNodesRRGraph(const t_rr_graph_storage& node_storage) : node_storage_(node_storage){
}

  /* Get the capacitance of a routing resource node.*/
  float FoldedNodesRRGraph::node_C(RRNodeId legacy_node) const {
    if (!built) return node_storage_.node_C(legacy_node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].C;
  }

  /* Get the resistance of a routing resource node. */
  float FoldedNodesRRGraph::node_R(RRNodeId legacy_node) const {
    if (!built) return node_storage_.node_R(legacy_node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].R;
  }
  



void FoldedNodesRRGraph::build_graph(){
    // Begin timing 
    vtr::ScopedStartFinishTimer timer("Build FoldedNodesRRGraph");

    // Clear all data structures of any previous data
    nodes_.clear();
    node_patterns_.clear();

    int node_pattern_idx = 0;

    std::map<std::string, int> temp_node_patterns {};
  
    // Iterate over every legacy node and find node_patterns
    for (size_t idx = 0; idx < node_storage_.size(); idx++) {   
        RRNodeId id = RRNodeId(idx);
        int16_t x = node_storage_.node_xlow(id);
        int16_t y = node_storage_.node_ylow(id);
        int16_t dx = node_storage_.node_xhigh(id) - x;
        int16_t dy = node_storage_.node_yhigh(id) - y;
        t_rr_type current_type = node_storage_.node_type(id);

        FoldedNodePattern node_pattern = { 
                                            node_storage_.node_cost_index(id),
                                            node_storage_.node_rc_index(id),
                                            dx,
                                            dy,
                                            // current_type,//edge_patterns,
                                            (uint16_t) node_storage_.node_capacity(id),
                                            {Direction::NUM_DIRECTIONS}

        };
        std::string node_pattern_string = std::to_string(node_storage_.node_cost_index(id)) + "_" +
                                          std::to_string(node_storage_.node_rc_index(id)) + "_" +
                                          std::to_string(dx) + "_" +
                                          std::to_string(dy) + "_" +
                                        //   std::to_string(current_type) + "_" +  
                                          std::to_string(node_storage_.node_capacity(id)) + "_"; 
                                          
        // set direction if using CHANX or CHANY
        if (current_type== CHANX || current_type == CHANY){
                    node_pattern.dir_side_.direction_ = node_storage_.node_direction(id);
                    node_pattern_string += "_" + std::string(node_storage_.node_direction_string(id));
        }
        // set sides if using IPIN or OPIN
        if (current_type== IPIN || current_type == OPIN){
                for (auto side : SIDES){ // iterate over SIDES to find the side of the current node
                    if (strcmp(SIDE_STRING[side],node_storage_.node_side_string(id))==0){
                        node_pattern.dir_side_.sides_ = side;
                    }
                }
                node_pattern_string += "_" + std::string(node_storage_.node_side_string(id));
        }

        // insert into map if not in map
        if (!(temp_node_patterns.count(node_pattern_string)>0)){
            temp_node_patterns[node_pattern_string] = node_pattern_idx;
            node_pattern_idx++;
            node_patterns_.push_back(node_pattern);
        }

        int16_t cur_pattern_idx = temp_node_patterns[node_pattern_string];
       
        t_folded_node_data cur_node = {
            cur_pattern_idx,
            x,
            y,
            current_type
        };

        nodes_.push_back(cur_node);
    }


    built = true;
    size_ = node_storage_.size();

    print_memory_stats();

    /* Don't verify for now */
    verify_folded_rr_graph();
}
/* Verify that folded_rr_graph contains the correct information from node_storage_ */
void FoldedNodesRRGraph::verify_folded_rr_graph(){
    vtr::ScopedStartFinishTimer timer("Verify FoldedNodesRRGraph");
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

short FoldedNodesRRGraph::node_ptc_num(RRNodeId id) const {
    return node_storage_.node_ptc_num(id);
}
short FoldedNodesRRGraph::node_pin_num(RRNodeId id) const {
    return node_storage_.node_pin_num(id);
}

short FoldedNodesRRGraph::node_track_num(RRNodeId id) const {
    return node_storage_.node_track_num(id);
}
short FoldedNodesRRGraph::node_class_num(RRNodeId id) const {
    return node_storage_.node_class_num(id);
}
