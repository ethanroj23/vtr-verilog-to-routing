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
    if (!built) return node_storage_.node_C(legacy_node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].C;
  }

  /* Get the resistance of a routing resource node. */
  float FoldedPerTileRRGraph::node_R(RRNodeId legacy_node) const {
    if (!built) return node_storage_.node_R(legacy_node);
    auto& device_ctx = g_vpr_ctx.device();
    VTR_ASSERT(node_rc_index(legacy_node) < (short)device_ctx.rr_rc_data.size());
    return device_ctx.rr_rc_data[node_rc_index(legacy_node)].R;
  }
  

int FoldedPerTileRRGraph::node_length(RRNodeId node){
    int x_low = node_storage_.node_xlow(node);
    int y_low = node_storage_.node_ylow(node);
    int x_high = node_storage_.node_xhigh(node);
    int y_high = node_storage_.node_yhigh(node);
    int dx = x_high - x_low;
    int dy = y_high - y_low;

    if (dx) return dx;
    if (dy) return dy;
    return 0;
}


void FoldedPerTileRRGraph::build_graph(){
    // Begin timing 
    vtr::ScopedStartFinishTimer timer("Build FoldedPerTileRRGraph");

    // Clear all data structures of any previous data
    tile_to_node_.clear();


    // std::map<DxDyIdx, std::array<int16_t, 2>> temp_dx_dy {};
    std::map<DxDy, DxDyIdx> temp_dx_dy {};
    int dx_dy_idx = 0; // index for dx_dy patterns
    std::map<std::string, NodePatternIdx> temp_node_patterns {};
    std::map<std::string, SharedNodeDataIdx> temp_shared_node_data {};
    int node_pattern_idx = 0;
    int shared_node_data_idx = 0;

    // vvv CREATE ORDERED NODES IN TILES
    size_ = node_storage_.size();
    node_to_tile_.resize(size_);
    node_to_pattern_idx_.resize(size_);
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
            std::vector<int> cur_lengths;
            for (auto node : tile_to_node_[x][y]){
                cur_lengths.push_back(node_length(node)); // get length of each node
            }
            // generate ordered indices based on sorting params
            std::vector<int> indices(cur_lengths.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(),
                    [&](int A, int B) -> bool {
                            return cur_lengths[A] < cur_lengths[B];
                        });
            std::vector<RRNodeId> sorted_nodes(tile_to_node_[x][y].size());

            for (int i=0; i < indices.size(); i++){
                int idx = indices[i];
                sorted_nodes[i] = tile_to_node_[x][y][idx];
            }
            tile_to_node_[x][y] = sorted_nodes;
        }
    }


    // create node_to_tile
    int nodes_processed = 0;
    for (int x = 0; x < tile_to_node_.size(); x++){
        for (int y = 0; y < tile_to_node_[x].size(); y++){
            for (int idx = 0; idx < tile_to_node_[x][y].size(); idx++){
                node_to_tile_[tile_to_node_[x][y][idx]] = {x, y, idx};
                nodes_processed += 1;
            }
        }
    }
            
    printf("here\n");
    // ^^^ CREATE ORDERED NODES IN TILES

    // vvv FOR EACH NODE
    for (int i=0; i<size_; i++){
        RRNodeId node_id = RRNodeId(i);
        int x_low = node_storage_.node_xlow(node_id);
        int y_low = node_storage_.node_ylow(node_id);
        t_rr_type current_type = node_storage_.node_type(node_id);

        t_folded_node_data node_pattern = {
            node_storage_.node_cost_index(node_id),
            node_storage_.node_rc_index(node_id),
            node_length(node_id),
            node_storage_.node_capacity(node_id),
            current_type,
            {Direction::NUM_DIRECTIONS}
        };
        std::string dir_side_string = "";

        // set direction if using CHANX or CHANY
        if (current_type == CHANX || current_type == CHANY){
                    node_pattern.dir_side_.direction_ = node_storage_.node_direction(node_id);    
                    dir_side_string += node_storage_.node_direction_string(node_id);                
        }
        // set sides if using IPIN or OPIN
        if (current_type== IPIN || current_type == OPIN){
            for (auto side : SIDES){ // iterate over SIDES to find the side of the current node
                if (strcmp(SIDE_STRING[side], node_storage_.node_side_string(node_id))==0){
                    node_pattern.dir_side_.sides_ = side;
                }
            }
            dir_side_string += node_storage_.node_side_string(node_id);
        }

        // vvv FOR EACH EDGE
        std::vector<std::string> edge_strings; //  used for edge pattern
        std::vector<t_folded_edge_data> edges; //  used for edge pattern
        for (RREdgeId edge : edge_range(node_id)) {// for each edge of this node

            RRNodeId dest_node = node_storage_.edge_sink_node(edge);
            int switch_id = node_storage_.edge_switch(edge);

            int dest_x_low = node_storage_.node_xlow(dest_node);
            int dest_y_low = node_storage_.node_ylow(dest_node);
            t_rr_type dest_current_type = node_storage_.node_type(dest_node);

            // t_folded_node_data node_pattern = {
            //     node_storage_.node_cost_index(dest_node),
            //     node_storage_.node_rc_index(dest_node),
            //     node_length(dest_node),
            //     node_storage_.node_capacity(dest_node),
            //     dest_current_type,
            //     {Direction::NUM_DIRECTIONS}
            // };
            int16_t dx = dest_x_low - x_low;
            int16_t dy = dest_y_low - y_low;
            TileIdx tile_idx = node_to_tile_[dest_node][2];
            DxDy cur_dx_dy = {dx, dy};


            if (!(temp_dx_dy.count(cur_dx_dy)>0)){ // cur_dx_dy not in temp_dx_dy
                temp_dx_dy[cur_dx_dy] = dx_dy_idx;
                dx_dy_idx++;
                dx_dy_.push_back(cur_dx_dy);
            }
            DxDyIdx cur_dx_dy_idx = temp_dx_dy[cur_dx_dy];

            std::string cur_edge_string = std::to_string(cur_dx_dy_idx)+"_"+std::to_string(switch_id)+"_"+std::to_string(tile_idx);
            edge_strings.push_back(cur_edge_string);

            t_folded_edge_data cur_edge = {
                cur_dx_dy_idx,
                switch_id,
                tile_idx
            };
            edges.push_back(cur_edge);
            // edges.append([dx_dy_idx, switch, idx])
        }
        // ^^^ FOR EACH EDGE
        



        // get node string
        std::string node_pattern_string = std::to_string(node_pattern.cost_index_)+"_"+
                                          std::to_string(node_pattern.rc_index_)+"_"+
                                          std::to_string(node_pattern.length_)+"_"+
                                          std::to_string(node_pattern.capacity_)+"_"+
                                          node_storage_.node_type_string(node_id)+"_"+
                                          dir_side_string;
        std::sort(edge_strings.begin(), edge_strings.end());

        std::ostringstream imploded;
        std::copy(edge_strings.begin(), edge_strings.end(),
                std::ostream_iterator<std::string>(imploded, "_"));

        std::string pattern_string = node_pattern_string + "_" + imploded.str();


        if (!(temp_node_patterns.count(pattern_string)>0)){ // node_edge pattern has not been found before
            temp_node_patterns[pattern_string] = node_pattern_idx;
            shared_edges_.push_back(edges);
            if (!(temp_shared_node_data.count(node_pattern_string)>0)) { // not in shared node data yet
                temp_shared_node_data[node_pattern_string] = shared_node_data_idx;
                shared_node_data_.push_back(node_pattern);
                shared_node_data_idx++;
            }
            shared_node_.push_back(temp_shared_node_data[node_pattern_string]);


            node_pattern_idx++;
        }

        int16_t cur_pattern_idx = temp_node_patterns[pattern_string];
        node_to_pattern_idx_[node_id] = cur_pattern_idx;
        // TileIdx cur_node_tile_idx = node_to_tile_[node_id][2];
        // if (x_low+1 > tile_to_node_.size()){
        //     tile_to_node_pattern_idx_.resize(x_low+1); // make sure tile structures are big enough
        // if (y_low+1 > tile_to_node_[x_low].size())
        //     tile_to_node_pattern_idx_[x_low].resize(y_low+1);
        // if (cur_node_tile_idx+1 > tile_to_node_[x_low][y_low].size())
        //     tile_to_node_pattern_idx_[x_low][y_low].resize(cur_node_tile_idx+1);
        // tile_to_node_pattern_idx_[x_low][y_low][cur_node_tile_idx] = cur_pattern_idx;


    }
    // ^^^ FOR EACH NODE
    printf("here\n");
    verify_folded_rr_graph();
}


    // print_memory_stats();
    // node_storage_.clear_edge_src_nodes();

    // /* Don't verify for now */
    // verify_folded_rr_graph();

//The entire flow of VPR took 17.72 seconds (max_rss 48.8 MiB)

/* Verify that folded_rr_graph contains the correct information from node_storage_ */
void FoldedPerTileRRGraph::verify_folded_rr_graph(){
    vtr::ScopedStartFinishTimer timer("Verify FoldedPerTileRRGraph");
    // NODES
    for (size_t idx = 0; idx < node_storage_.size(); idx++) { 
        RRNodeId id = RRNodeId(idx);
        VTR_ASSERT(node_storage_.node_xlow(id) == node_xlow(id));
        VTR_ASSERT(node_storage_.node_xhigh(id) == node_xhigh(id));
        VTR_ASSERT(node_storage_.node_ylow(id) == node_ylow(id));
        VTR_ASSERT(node_storage_.node_yhigh(id) == node_yhigh(id)); // SINK/SOURCE should always have the same ylow/yhigh, but they don't in some instances. vtr_main RRNodeId 1494

        VTR_ASSERT(node_storage_.node_rc_index(id) == node_rc_index(id));

        VTR_ASSERT(node_storage_.node_cost_index(id) == node_cost_index(id));
        VTR_ASSERT(node_storage_.node_type(id) == node_type(id));
        VTR_ASSERT(node_storage_.node_capacity(id) == node_capacity(id));
        

        if (node_type(id) == CHANX || node_type(id) == CHANY) 
            VTR_ASSERT(node_storage_.node_direction(id) == node_direction(id));
        
        if(node_type(id) == IPIN || node_type(id) == OPIN)
            VTR_ASSERT(strcmp(node_storage_.node_side_string(id),node_side_string(id))==0);

        // EDGES - Get a sorted list of edges, then compare the lists
        std::vector<std::array<int, 3>> original_edges;
        std::vector<std::array<int, 3>> folded_edges;
        for (RREdgeId edge : node_storage_.edge_range(id)) {//ESR_EDGE iterate over edges
            std::array<int, 3> cur_edges = {(size_t)id, (size_t)node_storage_.edge_sink_node(edge), (size_t)node_storage_.edge_switch(edge)};
            original_edges.push_back(cur_edges);
            // apply(edge, RRNodeId(node), edge_sink_node(edge));
        }

        NodePatternIdx pattern_idx = node_to_pattern_idx_[id];
        auto edges = shared_edges_[pattern_idx];
        for (auto edge : edges){
            int dx = dx_dy_[edge.dx_dy_idx].dx_;
            int dy = dx_dy_[edge.dx_dy_idx].dy_;
            int x = node_xlow(id) + dx;
            int y = node_ylow(id) + dy;
            RRNodeId dest_node = tile_to_node_[x][y][edge.tile_idx];
            std::array<int, 3> cur_edges = {(size_t)id, (size_t)dest_node, (size_t)edge.switch_id};
            folded_edges.push_back(cur_edges);
        }
        std::sort(original_edges.begin(), original_edges.end());
        std::sort(folded_edges.begin(), folded_edges.end());
        VTR_ASSERT(folded_edges == original_edges);
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
