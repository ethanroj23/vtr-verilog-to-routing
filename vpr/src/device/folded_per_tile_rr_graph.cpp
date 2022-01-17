#include "vtr_assert.h"
#include "folded_per_tile_rr_graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>
#include "globals.h"
#include "vtr_time.h"

FoldedPerTileRRGraph::FoldedPerTileRRGraph(t_rr_graph_storage& node_storage) : node_storage_(node_storage){
}

  /* Get the capacitance of a routing resource node.*/
  float FoldedPerTileRRGraph::node_C(RRNodeId legacy_node) const {
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].C;
  }

  /* Get the resistance of a routing resource node. */
  float FoldedPerTileRRGraph::node_R(RRNodeId legacy_node) const {
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].R;
  }
  

void FoldedPerTileRRGraph::build_graph(){
    // Begin timing 
    vtr::ScopedStartFinishTimer timer("Build FoldedPerTileRRGraph");

    // Clear all data structures of any previous data
    tile_to_node_.clear();
    node_type_.clear();
    node_coords_.clear();
    node_to_rc_.clear();
    edge_count_ = node_storage_.edge_count();


    // std::map<DxDyIdx, std::array<int16_t, 2>> temp_dx_dy {};
    std::map<DxDy, DxDyIdx> temp_dx_dy {};
    int dx_dy_idx = 0; // index for dx_dy patterns
    std::map<t_folded_node_edge_pattern, NodePatternIdx> temp_node_patterns {};
    std::map<t_folded_node_data, SharedNodeDataIdx> temp_shared_node_data {};
    NodePatternIdx node_pattern_idx = 0;
    SharedNodeDataIdx shared_node_data_idx = 0;

    // vvv CREATE ORDERED NODES IN TILES
    size_ = node_storage_.size();
    node_coords_.reserve(size_);
    node_to_rc_.reserve(size_);
    node_to_dir_side_.reserve(size_);
    node_type_.reserve(size_);
    


    vtr::vector<RRNodeId, std::array<TileIdx, 3>> temp_node_to_tile; // goes from RRNodeId to array of [x, y]
    temp_node_to_tile.resize(size_);

    for (int i=0; i<size_; i++){ // for each NODE
        RRNodeId node_id = RRNodeId(i);
        int x_low = node_storage_.node_xlow(node_id);
        int y_low = node_storage_.node_ylow(node_id);
        if (x_low+1 > tile_to_node_.size())
            tile_to_node_.resize(x_low+1); // make sure tile structures are big enough
        if (y_low+1 > tile_to_node_[x_low].size())
            tile_to_node_[x_low].resize(y_low+1);
        tile_to_node_[x_low][y_low].push_back(node_id);
    }

    // sort by length, type, segment id
    // start with just sorting by length
    
    for (int x = 0; x < tile_to_node_.size(); x++){
        for (int y = 0; y < tile_to_node_[x].size(); y++){
            std::vector<std::array<int, 3>> sorting_params; // length, type, cost index
            sorting_params.reserve(tile_to_node_[x][y].size());
            for (auto node : tile_to_node_[x][y]){
                sorting_params.push_back({node_length(node), node_storage_.node_type(node), node_storage_.node_cost_index(node)}); // get length of each node
            }
            // generate ordered indices based on sorting params
            std::vector<int> indices(sorting_params.size()); 
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(),
                    [&](int A, int B) -> bool {
                            if (sorting_params[A][0] < sorting_params[B][0]){ // length
                                return true; // A is smaller
                            } else if (sorting_params[A][0] == sorting_params[B][0] && // length equal and
                                        sorting_params[A][1] < sorting_params[B][1]){ // type
                                return true; // A is smaller
                            }
                            else if (sorting_params[A][0] == sorting_params[B][0] && // length equal and
                                     sorting_params[A][1] == sorting_params[B][1] && // type equal and
                                     sorting_params[A][2] < sorting_params[B][2] // cost_index
                                        ){
                                return true; // A is smaller
                            }
                            else{
                                return false; // B is smaller
                            }
                        });
            std::vector<RRNodeId> sorted_nodes;
            sorted_nodes.reserve(tile_to_node_[x][y].size());

            for (int i=0; i < indices.size(); i++){
                int idx = indices[i];
                sorted_nodes.push_back(tile_to_node_[x][y][idx]);
            }
            tile_to_node_[x][y] = sorted_nodes;
        }
    }


    // create node_to_tile
    int nodes_processed = 0;
    for (int x = 0; x < tile_to_node_.size(); x++){
        for (int y = 0; y < tile_to_node_[x].size(); y++){
            for (int idx = 0; idx < tile_to_node_[x][y].size(); idx++){
                temp_node_to_tile[tile_to_node_[x][y][idx]] = {x, y, idx};
                nodes_processed += 1;
            }
        }
    }
            
    // ^^^ CREATE ORDERED NODES IN TILES

    // vvv FOR EACH NODE
    for (int i=0; i<size_; i++){
        RRNodeId node_id = RRNodeId(i);
        uint16_t x_low = node_storage_.node_xlow(node_id);
        uint16_t y_low = node_storage_.node_ylow(node_id);
        uint16_t x_high = node_storage_.node_xhigh(node_id);
        uint16_t y_high = node_storage_.node_yhigh(node_id);
        t_rr_type current_type = node_storage_.node_type(node_id);
        node_type_.push_back(current_type);

        t_folded_node_data node_pattern = {
            node_storage_.node_cost_index(node_id),
            node_storage_.node_rc_index(node_id),
            node_length(node_id),
            node_storage_.node_capacity(node_id),
            {Direction::NUM_DIRECTIONS}
        };
        

        t_folded_dir_side node_folded_dir_side = {
            node_pattern.capacity_, 
            {Direction::NUM_DIRECTIONS}
        };

        // set direction if using CHANX or CHANY
        if (current_type == CHANX || current_type == CHANY){
                    node_pattern.dir_side_.direction_ = node_storage_.node_direction(node_id);    
                    node_folded_dir_side.dir_side.direction = node_storage_.node_direction(node_id);    
        }
        // set sides if using IPIN or OPIN
        if (current_type== IPIN || current_type == OPIN){
            for (auto side : SIDES){ // iterate over SIDES to find the side of the current node
                if (strcmp(SIDE_STRING[side], node_storage_.node_side_string(node_id))==0){
                    node_pattern.dir_side_.sides_ = side;
                    node_folded_dir_side.dir_side.sides = side;
                }
            }
        }
        node_to_rc_.push_back({x_high, y_high, node_pattern.cost_index_, node_pattern.rc_index_});
        node_to_dir_side_.push_back(node_folded_dir_side);


        // vvv FOR EACH EDGE
        std::vector<t_folded_edge_data> edges; //  used for edge pattern
        std::vector<t_folded_edge_data> temp_edges; //  used for edge pattern

        for (RREdgeId edge : edge_range(node_id)) {// for each edge of this node

            RRNodeId dest_node = node_storage_.edge_sink_node(edge);
            int switch_id = node_storage_.edge_switch(edge);

            int dest_x_low = node_storage_.node_xlow(dest_node);
            int dest_y_low = node_storage_.node_ylow(dest_node);
            t_rr_type dest_current_type = node_storage_.node_type(dest_node);

            short dx = dest_x_low - x_low;
            short dy = dest_y_low - y_low;
            TileIdx tile_idx = temp_node_to_tile[dest_node][2];

            t_folded_edge_data cur_edge = {
                dx,
                dy,
                switch_id,
                tile_idx
            };
            edges.push_back(cur_edge);

            temp_edges.push_back(cur_edge);
        }
        // ^^^ FOR EACH EDGE
    
        std::sort(temp_edges.begin(), temp_edges.end());


        t_folded_node_edge_pattern node_edge_pattern = {
            node_pattern,
            temp_edges
        };

        if (!(temp_node_patterns.count(node_edge_pattern)>0)){ // node_edge pattern has not been found before
            temp_node_patterns[node_edge_pattern] = node_pattern_idx;
            shared_edges_.push_back(edges);
            node_pattern_idx++;
        }

        NodePatternIdx cur_pattern_idx = temp_node_patterns[node_edge_pattern];
        node_coords_.push_back({x_low, y_low, cur_pattern_idx});
    }
    // ^^^ for each node

    VTR_LOG("Total patterns: %d of %d nodes %.2f\n", node_pattern_idx, size_, (float)node_pattern_idx/size_);
    // verify_folded_rr_graph();
}


    // print_memory_stats();
    // node_storage_.clear_edge_src_nodes();

    // /* Don't verify for now */
    // verify_folded_rr_graph();

//The entire flow of VPR took 17.72 seconds (max_rss 48.8 MiB)

/* Verify that folded_rr_graph contains the correct information from node_storage_ */
void FoldedPerTileRRGraph::verify_folded_rr_graph(){
    vtr::ScopedStartFinishTimer timer("Verify FoldedPerTileRRGraph");
    /*
    Could potentially exceed 65,536
    x
    y
    TileIdx
    NodePatternIdx
    DxDyIdx
    SharedNodeDataIdx
    */
   int max_x = tile_to_node_.size(); // x
   int max_y = 0;
   int max_TileIdx = 0;
    for (int x = 0; x < tile_to_node_.size(); x++){
        for (int y = 0; y < tile_to_node_[x].size(); y++){
            if (y > max_y) max_y = y; // y
            if (tile_to_node_[x][y].size() > max_TileIdx) max_TileIdx = tile_to_node_[x][y].size(); // TileIdx
        }
    }
    // int max_NodePatternIdx = shared_node_.size();
    // int max_DxDyIdx = dx_dy_.size();
    // int max_SharedNodeDataIdx = shared_node_data_.size();

    printf("The Following numbers should not exceed 65,536 since they only have 2 Bytes allotted to them\n"
           "x -> %d\n"
           "y -> %d\n"
           "TileIdx -> %d\n",
        //    "NodePatternIdx -> %d\n"
        //    "DxDyIdx -> %d\n"
        //    "SharedNodeDataIdx -> %d\n",
           max_x,
           max_y,
           max_TileIdx
        //    max_NodePatternIdx,
        //    max_DxDyIdx,
        //    max_SharedNodeDataIdx
    );
    // return;


        VTR_ASSERT(node_storage_.size() == size());
        VTR_ASSERT(node_storage_.edge_count() == edge_count());
        VTR_ASSERT(node_storage_.empty() == empty());

    // NODES
    for (size_t idx = 0; idx < node_storage_.size(); idx++) { 
        RRNodeId id = RRNodeId(idx);
        if (idx % 1000 == 0)
            printf("%lu\n", idx);

        VTR_ASSERT(node_storage_.node_xlow(id) == node_xlow(id));
        VTR_ASSERT(node_storage_.node_xhigh(id) == node_xhigh(id));
        VTR_ASSERT(node_storage_.node_ylow(id) == node_ylow(id));
        VTR_ASSERT(node_storage_.node_yhigh(id) == node_yhigh(id)); // SINK/SOURCE should always have the same ylow/yhigh, but they don't in some instances. vtr_main RRNodeId 1494

        VTR_ASSERT(node_storage_.node_rc_index(id) == node_rc_index(id));
        VTR_ASSERT(node_storage_.node_R(id) == node_R(id));
        VTR_ASSERT(node_storage_.node_C(id) == node_C(id));
        VTR_ASSERT(node_storage_.node_fan_in(id) == node_fan_in(id));
        VTR_ASSERT(node_storage_.fan_in(id) == fan_in(id));

        VTR_ASSERT(node_storage_.node_ptc_num(id) == node_ptc_num(id));
        // VTR_ASSERT(node_storage_.node_pin_num(id) == node_pin_num(id));
        // VTR_ASSERT(node_storage_.node_track_num(id) == node_track_num(id));
        // VTR_ASSERT(node_storage_.node_class_num(id) == node_class_num(id));

        VTR_ASSERT(node_storage_.node_cost_index(id) == node_cost_index(id));
        VTR_ASSERT(node_storage_.node_type(id) == node_type(id));
        VTR_ASSERT(node_storage_.node_capacity(id) == node_capacity(id));


        // Verify Edges
        VTR_ASSERT(node_storage_.num_configurable_edges(id) == num_configurable_edges(id));
        VTR_ASSERT(node_storage_.num_non_configurable_edges(id) == num_non_configurable_edges(id));
        VTR_ASSERT(node_storage_.num_edges(id) == num_edges(id));
        VTR_ASSERT(node_storage_.first_edge(id) == first_edge(id));
        VTR_ASSERT(node_storage_.last_edge(id) == last_edge(id));
        

        if (node_type(id) == CHANX || node_type(id) == CHANY) {
            VTR_ASSERT(node_storage_.node_direction(id) == node_direction(id));
            VTR_ASSERT(node_storage_.node_direction_string(id) == node_direction_string(id));
        }
        
        if(node_type(id) == IPIN || node_type(id) == OPIN)
            VTR_ASSERT(strcmp(node_storage_.node_side_string(id),node_side_string(id))==0);


        for (int k = 0; k < num_edges(id); k++){
            VTR_ASSERT(node_storage_.edge_sink_node(id, k) == edge_sink_node(id, k));
            VTR_ASSERT(node_storage_.edge_switch(id, k) == edge_switch(id, k));
        }

        // EDGES using RREdgeIds...
        // for (RREdgeId edge : node_storage_.edge_range(id)) {//ESR_EDGE iterate over edges
        //     VTR_ASSERT(node_storage_.edge_switch(edge) == edge_switch(edge));
        //     VTR_ASSERT(node_storage_.edge_sink_node(edge) == edge_sink_node(edge));
        // }

        // EDGES - Get a sorted list of edges for this node, then compare the lists
        std::vector<std::array<int, 3>> original_edges;
        std::vector<std::array<int, 3>> folded_edges;

        for (auto edge : node_storage_.edge_range(id)){
            std::array<int, 3> cur_edge = {(size_t) edge_src_node(edge), (size_t) edge_sink_node(edge), edge_switch(edge)};
            original_edges.push_back(cur_edge);
        }
        for (auto edge : edge_range_src(id)){
            std::array<int, 3> cur_edge = {(size_t) edge.src, (size_t) edge.dest, (size_t) edge.switch_id};
            folded_edges.push_back(cur_edge);
        }

        VTR_ASSERT(original_edges == folded_edges);

        // for (RREdgeId edge : node_storage_.edge_range(id)) {//ESR_EDGE iterate over edges
        //     std::array<int, 3> cur_edges = {(size_t)id, (size_t)node_storage_.edge_sink_node(edge), (size_t)node_storage_.edge_switch(edge)};
        //     original_edges.push_back(cur_edges);
        //     // apply(edge, RRNodeId(node), edge_sink_node(edge));
        // }

        // NodePatternIdx pattern_idx = node_to_pattern_idx_[id];
        // auto edges = shared_edges_[pattern_idx];
        // for (auto edge : edges){
        //     int dx = dx_dy_[edge.dx_dy_idx].dx_;
        //     int dy = dx_dy_[edge.dx_dy_idx].dy_;
        //     int x = node_xlow(id) + dx;
        //     int y = node_ylow(id) + dy;
        //     RRNodeId dest_node = tile_to_node_[x][y][edge.tile_idx];
        //     std::array<int, 3> cur_edges = {(size_t)id, (size_t)dest_node, (size_t)edge.switch_id};
        //     folded_edges.push_back(cur_edges);
        // }
        // std::sort(original_edges.begin(), original_edges.end());
        // std::sort(folded_edges.begin(), folded_edges.end());
        // VTR_ASSERT(folded_edges == original_edges);
    }
}

short FoldedPerTileRRGraph::node_ptc_num(RRNodeId id) const {
    return node_storage_.node_ptc_num(id);
}
short FoldedPerTileRRGraph::node_pin_num(RRNodeId id) const {
    auto cur_type = node_type(id);
    if (cur_type != IPIN && cur_type != OPIN) {
        VPR_FATAL_ERROR(VPR_ERROR_ROUTE, "Attempted to access RR node 'pin_num' for non-IPIN/OPIN type '%s'", rr_node_typename[cur_type]);
    }
    return node_storage_.node_pin_num(id, false);
}

short FoldedPerTileRRGraph::node_track_num(RRNodeId id) const {
    auto cur_type = node_type(id);
    if (cur_type != CHANX && cur_type != CHANY) {
        VPR_FATAL_ERROR(VPR_ERROR_ROUTE, "Attempted to access RR node 'track_num' for non-CHANX/CHANY type '%s'", rr_node_typename[cur_type]);
    }
    return node_storage_.node_track_num(id, false);
}
short FoldedPerTileRRGraph::node_class_num(RRNodeId id) const {
    auto cur_type = node_type(id);
    if (cur_type != SOURCE && cur_type != SINK) {
        VPR_FATAL_ERROR(VPR_ERROR_ROUTE, "Attempted to access RR node 'class_num' for non-SOURCE/SINK type '%s'", rr_node_typename[cur_type]);
    }
    return node_storage_.node_class_num(id, false);
}
