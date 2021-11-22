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
    node_first_edge_.clear();
    edge_dest_node_.clear();
    edge_switch_.clear();

    // Resize node_first_edge_ so that it fits every node plus one for the dummy node
    if (node_first_edge_.size() < node_storage_.size()){
            node_first_edge_.resize((size_t) node_storage_.size()+1);
    }

    // Resize node_first_edge_ so that it fits every node plus one for the dummy node
    if (edge_dest_node_.size() < node_storage_.edge_count()){
            edge_dest_node_.resize((size_t) node_storage_.edge_count());
    }

    // Resize node_first_edge_ so that it fits every node plus one for the dummy node
    if (edge_switch_.size() < node_storage_.edge_count()){
            edge_switch_.resize((size_t) node_storage_.edge_count());
    }


    int edge_starting_idx = 0;
    for (size_t idx = 0; idx < node_storage_.size(); idx++) { /* for every node */
            std::vector<int16_t> edge_pattern_list;
            RRNodeId id = RRNodeId(idx);  
            node_first_edge_[id] = RREdgeId(edge_starting_idx); 
            bool first_edge = true;
            for (RREdgeId from_edge : node_storage_.edge_range(id)) { // --- start  each node ---
                if (first_edge){
                    node_first_edge_[id] = from_edge; // only set first
                    first_edge = false;
                }
                edge_starting_idx++;
            } // --- end each node ---
            if (first_edge) node_first_edge_[id] = RREdgeId(edge_starting_idx); // only set first
        }


    for (size_t idx = 0; idx < node_storage_.edge_count(); idx++){ /* for every edge */
        RREdgeId id = RREdgeId(idx);
        edge_dest_node_[id] = node_storage_.edge_sink_node(id);
        edge_switch_[id] = node_storage_.edge_switch(id);
    }

    edges_size_ = node_storage_.edge_count(); // set total number of edges
    RRNodeId dummy_node = RRNodeId(node_storage_.size());
    node_first_edge_[dummy_node] = RREdgeId(edge_starting_idx); // only set first
    
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

short FoldedEdges1RRGraph::node_ptc_num(RRNodeId id) const {return node_storage_.node_ptc_num(id);}
short FoldedEdges1RRGraph::node_pin_num(RRNodeId id) const {return node_storage_.node_pin_num(id);}
short FoldedEdges1RRGraph::node_track_num(RRNodeId id) const {return node_storage_.node_track_num(id);}
short FoldedEdges1RRGraph::node_class_num(RRNodeId id) const {return node_storage_.node_class_num(id);}

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
    VTR_ASSERT(!node_first_edge_.empty());

    const auto& device_ctx = g_vpr_ctx.device();
    auto first_id = size_t(node_first_edge_[legacy_node]);
    auto last_id = size_t((&node_first_edge_[legacy_node])[1]);
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