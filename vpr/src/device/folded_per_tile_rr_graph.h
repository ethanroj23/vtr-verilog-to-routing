#ifndef FOLDED_PER_TILE_RR_GRAPH_H
#define FOLDED_PER_TILE_RR_GRAPH_H

#include "vpr_types.h"
#include <iostream>
#include "rr_graph_storage.h"
#include "rr_graph_view_interface.h"

/* memory usage for FoldedPerTileRRGraph data members is defined here in Bytes */
#define FOLDED_TILE_PATTERN_SIZE 8
#define FOLDED_EDGE_PATTERN_SIZE 7
const short NODE_TO_TILE_XLOW = 0;
const short NODE_TO_TILE_YLOW = 1;
const short NODE_TO_TILE_IDX = 2;


class FoldedPerTileRRGraph : public RRGraphViewInterface{
    /* -- Constructors -- */
  public:
    using SharedNodeDataIdx = uint16_t; // 2 Bytes
    using NodePatternIdx = uint32_t; // 2 Bytes
    using TileIdx = uint16_t; // 2 Bytes
    using DxDyIdx = uint16_t; // 2 Bytes
    /* Explicitly define the only way to create an object */
    explicit FoldedPerTileRRGraph(t_rr_graph_storage& node_storage);

    /* Disable copy constructors and copy assignment operator
     * This is to avoid accidental copy because it could be an expensive operation considering that the 
     * memory footprint of the data structure could ~ Gb
     * Using the following syntax, we prohibit accidental 'pass-by-value' which can be immediately caught 
     * by compiler
     */
    FoldedPerTileRRGraph(const FoldedPerTileRRGraph&) = delete;
    void operator=(const FoldedPerTileRRGraph&) = delete;

    /* -- Accessors -- */
  public:
  
  // every function takes in an RRNodeId that has not been remapped yet
  /* The general method for accessing data from the FoldedPerTileRRGraph is as follows:
   * 1. Obtain the remapped_id from remapped_ids data member.v 
   * 2. Find the x and y coordinates of the tile that contains the input RRNode
   * 3. Obtain the node_patterns_idx from the tile
   * 4. Calculate the relative offset for the given RRNodeId
   * 5. Access the desired data with something like 
   *      node_data_[node_patterns[node_patterns_idx][offset]].type_; //replace "type_" with the data you are wanting
   */

  /*Return an RRNode's type.*/

  /* Print the current rr_graph type */
  inline const char* rr_graph_name() const{
    return "FoldedPerTileRRGraph";
  }

  /* Return the number of RRNodes */
  size_t size() const{
      return size_;
  }

  /* Return the number of bytes used to represent the nodes */
  int node_bytes() const{
      return 0;
  }

  /* Return the number of bytes used to represent the edges */
  size_t edge_bytes() const{
      return edge_count()*10;
  }

  void print_memory_stats() const{
    double current_bytes = node_bytes() + edge_bytes();
    std::cout << "Flat Representation: (including edges)" <<node_storage_bytes()/1024/1024.0 << " MiB" << "\n";
    std::cout << "This Folded Representation: (including edges)" << current_bytes/1024/1024.0 << " MiB" << "\n";
    printf("Flat Nodes: %lu, ", node_storage_.size() * 16);
    printf("Folded Nodes: %d, ", node_bytes() );
    printf("Node Reduction: %.2f, ", (double)(node_storage_.size() * 16) / (double)node_bytes() );
    printf("Edge Reduction: %.2f, ", (double)(edge_count() * 10) / (double)edge_bytes() );
    printf("Total Reduction: %.2f\n\n", node_storage_bytes() / current_bytes );
  }

  inline size_t edge_count() const {
        return edge_count_;
    }

  inline t_rr_type node_type(RRNodeId node) const{ 
    return node_type_[node];
  }

  inline short node_capacity(RRNodeId node) const{ 
    SharedNodeDataIdx data_idx = shared_node_[node_to_pattern_idx_[node]];
    return shared_node_data_[data_idx].capacity_;
  }
  
  /* Get the type string of a routing resource node. This function is inlined for runtime optimization. */
  inline const char* node_type_string(RRNodeId node) const {
      return rr_node_typename[node_type(node)];
  }

  /* Get the direction of a routing resource node. This function is inlined for runtime optimization.
    * Direction::INC: wire driver is positioned at the low-coordinate end of the wire.
    * Direction::DEC: wire_driver is positioned at the high-coordinate end of the wire.
    * Direction::BIDIR: wire has multiple drivers, so signals can travel either way along the wire
    * Direction::NONE: node does not have a direction, such as IPIN/OPIN
    */
  inline Direction node_direction(RRNodeId node) const {
    SharedNodeDataIdx data_idx = shared_node_[node_to_pattern_idx_[node]];
    return shared_node_data_[data_idx].dir_side_.direction_;
  }

  /* Get the direction string of a routing resource node. This function is inlined for runtime optimization. */
  inline const std::string& node_direction_string(RRNodeId node) const {
        Direction direction = node_direction(node);

        int int_direction = static_cast<int>(direction);
        VTR_ASSERT(int_direction >= 0 && int_direction < static_cast<int>(Direction::NUM_DIRECTIONS));
        return CONST_DIRECTION_STRING[int_direction];
  }

  /* Get the capacitance of a routing resource node. This function is inlined for runtime optimization. */
  float node_C(RRNodeId node) const;

  /* Get the resistance of a routing resource node. This function is inlined for runtime optimization. */
  float node_R(RRNodeId node) const;

  inline int node_length(RRNodeId node) const{
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

  /* Get the rc_index of a routing resource node. This function is inlined for runtime optimization. */
  inline int16_t node_rc_index(RRNodeId node) const {
    SharedNodeDataIdx data_idx = shared_node_[node_to_pattern_idx_[node]];
    return shared_node_data_[data_idx].rc_index_;
  }

  /* Get the xlow of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_xlow(RRNodeId node) const {
      return node_to_tile_[node][NODE_TO_TILE_XLOW];
  }

  /* Get the xhigh of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_xhigh(RRNodeId node) const {  
      SharedNodeDataIdx data_idx = shared_node_[node_to_pattern_idx_[node]];
    if (node_type_[node] == CHANX) //|| shared_node_data_[data_idx].type_ == SINK || shared_node_data_[data_idx].type_ == SOURCE)
        return node_to_tile_[node][NODE_TO_TILE_XLOW] + shared_node_data_[data_idx].length_;
      return node_to_tile_[node][NODE_TO_TILE_XLOW];
  }

  /* Get the ylow of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_ylow(RRNodeId node) const {
      return node_to_tile_[node][NODE_TO_TILE_YLOW];
  }

  /* Get the yhigh of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_yhigh(RRNodeId node) const {  
    SharedNodeDataIdx data_idx = shared_node_[node_to_pattern_idx_[node]];
    if (node_type_[node] == CHANY || node_type_[node]== SINK || node_type_[node] == SOURCE)
      return node_to_tile_[node][NODE_TO_TILE_YLOW] + shared_node_data_[data_idx].length_;
    return node_to_tile_[node][NODE_TO_TILE_YLOW];
  }

  void print_node_info_string(RRNodeId node) const {
    printf("Info for node: %lu\n"
            "Node type: %s\n"
            "xlow, xhigh, ylow, yhigh: %d, %d, %d, %d\n"
            "Node length: %d\n"
            "Node rc_index: %d\n"
            "Node cost_index: %d\n", (size_t)node, node_type_string(node), node_xlow(node), node_xhigh(node), node_ylow(node), node_yhigh(node),
              node_length(node), node_rc_index(node), node_cost_index(node));
  }

  /* Get the cost index of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_cost_index(RRNodeId node) const {
    SharedNodeDataIdx data_idx = shared_node_[node_to_pattern_idx_[node]];
    return shared_node_data_[data_idx].cost_index_;
  }

  inline bool is_node_on_specific_side(RRNodeId node, e_side side) const{
    t_rr_type current_type = node_type(node);
    if (current_type != IPIN && current_type != OPIN){
        VPR_FATAL_ERROR(VPR_ERROR_ROUTE,
          "Attempted to access RR node 'side' for non-IPIN/OPIN type '%s'",
          rr_node_typename[current_type]);
    }
    SharedNodeDataIdx data_idx = shared_node_[node_to_pattern_idx_[node]];
    return SIDES[shared_node_data_[data_idx].dir_side_.sides_]==side;
  }

  /* Check whether a routing node is on a specific side. This function is inlined for runtime optimization. */
  inline const char* node_side_string(RRNodeId node) const {// ⬛️
      for (const e_side& side : SIDES) {
          if (is_node_on_specific_side(node, side)) {
              return SIDE_STRING[side];
          }
      }
      /* Not found, return an invalid string*/
      return SIDE_STRING[NUM_SIDES];
  }

  /* Get the fan in of a routing resource node. This function is inlined for runtime optimization. */
  inline t_edge_size node_fan_in(RRNodeId node) const {
    return node_storage_.fan_in(node);
  }

  /* Get the fan in of a routing resource node. This function is inlined for runtime optimization. */
  inline t_edge_size fan_in(RRNodeId node) const {
    return node_storage_.fan_in(node);
  }

  /* PTC get methods */
  short node_ptc_num(RRNodeId id) const;
  short node_pin_num(RRNodeId id) const;   //Same as ptc_num() but checks that type() is consistent
  short node_track_num(RRNodeId id) const; //Same as ptc_num() but checks that type() is consistent
  short node_class_num(RRNodeId id) const; //Same as ptc_num() but checks that type() is consistent

  /* Edge Methods */


  // Is the RR graph currently empty?
  inline bool empty() const {
        return size_ == 0;
    }

  inline size_t node_count() const{
    return node_to_tile_.size();
  }

  // Estimate of memory taken by FoldedPerTileRRGraph (in Bytes)
  inline int memory_used() const {
/*
    node_to_tile_:           RRNodeId -> x, y, tile_idx
    tile_to_node_:     x, y, tile_idx -> RRNodeId
    node_to_pattern_idx_:             RRNodeId -> NodePatternIdx

    shared_edges_:     NodePatternIdx -> edges array
    shared_node_:      NodePatternIdx -> SharedNodeDataIdx
    shared_node_data_:   SharedNodeDataIdx -> t_folded_node_data

    dx_dy_:                      DxDyIdx -> DxDy
    
    

    */
    int node_count = node_to_tile_.size();
    int edge_count = 0;

    for (auto edges : shared_edges_){
      edge_count += edges.size();
    }
    printf("There are %d edges represented out of %lu total edges\n", edge_count, node_storage_.edge_count());

    int node_to_tile_bytes = node_count * (2 + 2 + 2); // x, y, tile_idx, try node_type EACH
    int tile_to_node_bytes = node_count * (4); // node_id EACH
    int node_to_pattern_idx_bytes = node_count * (4); // NodePatternIdx 2 or 4 depending on how many bytes needed per NodePatternIdx

    int shared_edges_bytes = edge_count * (2 + 2 + 2); // dx_dy, switch, tile_idx

    int shared_node_bytes = shared_node_.size() * (2); // SharedNodeDataIdx
    int shared_node_data_bytes = shared_node_data_.size() * (2 + 2 + 2 + 1 + 2); // type, length, rc, capacity, dir_side, cost_index EACH // trying without type for now
    int node_type_bytes = node_count * 1; // 1 byte each


    int dx_dy_bytes = dx_dy_.size() * (2 + 2); // dx, dy

    return node_to_tile_bytes + tile_to_node_bytes + node_to_pattern_idx_bytes + shared_edges_bytes + shared_node_bytes + shared_node_data_bytes + dx_dy_bytes + node_type_bytes;
    }

  /* must be called before the node_storage_ is deleted */
  inline int node_storage_bytes() const{
      return 0;
  }

  /* Compare folded rr_graph vs node_storage_ */
  void verify_folded_rr_graph();

  /* PUBLIC EDGE METHODS */

  // input: legacy node
  inline edge_idx_range edges(const RRNodeId& legacy_node) const {
        return vtr::make_range(edge_idx_iterator(0), edge_idx_iterator(num_edges(legacy_node)));
  }

  RREdgeId first_edge(const RRNodeId& legacy_node) const {
    return node_storage_.first_edge(legacy_node);
  }

  RREdgeId last_edge(const RRNodeId& legacy_node) const {
    return node_storage_.last_edge(legacy_node);
  }

  /* Edges are configurable if they have a switch that is configurable vtr/libs/libarchfpga/src/physical_types.cpp:83 */
  edge_idx_range configurable_edges(const RRNodeId& legacy_node) const {
        return vtr::make_range(edge_idx_iterator(0), edge_idx_iterator(num_edges(legacy_node) - num_non_configurable_edges(legacy_node)));
    }
  edge_idx_range non_configurable_edges(const RRNodeId& legacy_node) const {
        return vtr::make_range(edge_idx_iterator(num_edges(legacy_node) - num_non_configurable_edges(legacy_node)), edge_idx_iterator(num_edges(legacy_node)));
  }

  t_edge_size num_edges(const RRNodeId& legacy_node) const {
    return size_t(last_edge(legacy_node)) - size_t(first_edge(legacy_node));
  }

  t_edge_size num_configurable_edges(const RRNodeId& id) const {
      // VTR_ASSERT(!node_first_edge_.empty());
      auto first_id = size_t(first_edge(id));
      auto last_id = size_t(last_edge(id));
      for (size_t idx = first_id; idx < last_id; ++idx) {
          auto switch_idx = edge_switch(RREdgeId(idx));
          if (!node_storage_.switch_is_configurable(switch_idx)) {
              return idx - first_id;
          }
      }

      return last_id - first_id;
  }


  t_edge_size num_non_configurable_edges(const RRNodeId& legacy_node) const {
    return num_edges(legacy_node) - num_configurable_edges(legacy_node);
  }

  // Returns a range of RREdgeId's belonging to RRNodeId id.
  //
  // If this range is empty, then RRNodeId id has no edges.
  vtr::StrongIdRange<RREdgeId> edge_range(const RRNodeId legacy_node) const {
    return vtr::StrongIdRange<RREdgeId>(first_edge(legacy_node), last_edge(legacy_node));
  }

  /*
    input:  RREdgeId (remapped)
    output: RRNodeId (remapped)
  */


inline std::vector<t_edge_with_id> edge_range_with_id_direct(RRNodeId node) const{
      // returns a vector of edge_with_id structs, which each include src, sink, switch, and edge_id

      NodePatternIdx pattern_idx = node_to_pattern_idx_[node];
      auto folded_edges = shared_edges_[pattern_idx];

      std::vector<t_edge_with_id> return_edges; // initialize return vector
      auto x_y_tile = node_to_tile_[node];
      size_t k = 0; // kth edge
      size_t first = (size_t)first_edge(node);
      for (auto cur_edge : folded_edges){
        auto dx_dy = dx_dy_[cur_edge.dx_dy_idx];
        RREdgeId cur_edge_id = RREdgeId(first+k);
        t_edge_with_id add_edge = {
          node, // src
          tile_to_node_[x_y_tile[0]+dx_dy.dx_][x_y_tile[1]+dx_dy.dy_][cur_edge.tile_idx], // dest
          cur_edge.switch_id, // switch
          cur_edge_id
        };
        return_edges.push_back(add_edge);
      }
      return return_edges;
}

inline std::vector<t_edge_struct> edge_range_direct(RRNodeId node) const{
    return edges_direct(node); // TODO replace edges_direct() with edge_range_direct() later
}

inline std::vector<t_edge_struct> edges_direct(RRNodeId node) const{
      // returns a vector of edge structs, which each include src, sink, switch

      NodePatternIdx pattern_idx = node_to_pattern_idx_[node];
      auto folded_edges = shared_edges_[pattern_idx];

      std::vector<t_edge_struct> return_edges; // initialize return vector
      auto x_y_tile = node_to_tile_[node];
      for (auto cur_edge : folded_edges){
        auto dx_dy = dx_dy_[cur_edge.dx_dy_idx];
        t_edge_struct add_edge = {
          node, // src
          tile_to_node_[x_y_tile[0]+dx_dy.dx_][x_y_tile[1]+dx_dy.dy_][cur_edge.tile_idx], // dest
          cur_edge.switch_id // switch
        };
        return_edges.push_back(add_edge);
      }
      return return_edges;
}

inline t_edge_struct kth_edge_for_node(RRNodeId node, int k) const{
  return edges_direct(node)[k];
}

inline t_edge_struct get_t_edge_struct(RREdgeId edge_id) const {
   size_t edge_id_size = size_t(edge_id);
   for (size_t i=0; i<node_count(); i++){ //  for each node
      RRNodeId node = RRNodeId(i);
      size_t first = (size_t)first_edge(node);
      size_t last = (size_t)last_edge(node);
      if (edge_id_size >= first &&
          edge_id_size < last && 
          first != last){ // edge is a part of this node
            int edge_idx = edge_id_size - first;
            return kth_edge_for_node(node, edge_idx);
          }
   }
   return {RRNodeId(0), RRNodeId(0), 0}; // INVALID
}

inline RRNodeId edge_src_node(RREdgeId edge_id) const{
  // NOTE: Figure out how to get the correct value for this based on edge id, but 
  // remember that you will likely not be using this function as this folded
  // representation is not meant to use RREdgeIds.
    return get_t_edge_struct(edge_id).src;
}

inline RRNodeId edge_sink_node(const RREdgeId& edge_id) const {
    return get_t_edge_struct(edge_id).dest;
}
 
inline short edge_switch(const RREdgeId& edge_id) const {
    return get_t_edge_struct(edge_id).switch_id;
}

    // for (RREdgeId edge : rr_graph.edge_range(prev_node)) {//ESR_EDGE iterate over edges


// Call the `apply` function with the edge id, source, and sink nodes of every edge.
// inline void for_each_edge(std::function<void(RREdgeId, RRNodeId, RRNodeId)> apply) const {
//     return node_storage_.for_each_edge(apply);
// }

// Call the `apply` function with the edge id, source, and sink nodes of every edge.
    void for_each_edge(std::function<void(RREdgeId, RRNodeId, RRNodeId)> apply) const {
        for (size_t i = 0; i < edge_count(); i++) {
            RREdgeId edge(i);
            apply(edge, edge_src_node(edge), edge_sink_node(edge));
        }
    }
// Call the `apply` function with the edge id, source, and sink nodes of every edge.
inline void for_each_edge_direct(std::function<void(RREdgeId, RRNodeId, RRNodeId, short)> apply) const {
    for (size_t i = 0; i < node_count(); i++){
      int k = 0;
      RRNodeId node = RRNodeId(i);
      size_t first = (size_t)first_edge(node);
      for (auto edge : edges_direct(node)){
        apply(RREdgeId(first+k), node, edge.dest, edge.switch_id);
        k++;
      }
    }
}

// Call the `apply` function with the edge id, source, and sink nodes of every edge.
inline void for_each_edge_no_src(std::function<void(RREdgeId, RRNodeId)> apply) const {
    for (size_t i = 0; i < edge_count(); i++) {
        RREdgeId edge(i);
        apply(edge, edge_sink_node(edge));
    }
}

inline RREdgeId edge_id(const RRNodeId& id, t_edge_size iedge) const {
        RREdgeId first = first_edge(id);
        RREdgeId ret(size_t(first) + iedge);
        VTR_ASSERT_SAFE(ret < last_edge(id));
        return ret;
    }

// Get the destination node for the iedge'th edge from specified RRNodeId.
//
// This method should generally not be used, and instead first_edge and
// last_edge should be used.
inline RRNodeId edge_sink_node(const RRNodeId& id, t_edge_size iedge) const {
    return edge_sink_node(edge_id(id, iedge));
}

// Get the switch used for the iedge'th edge from specified RRNodeId.
//
// This method should generally not be used, and instead first_edge and
// last_edge should be used.
short edge_switch(const RRNodeId& id, t_edge_size iedge) const {
    return edge_switch(edge_id(id, iedge));
}

  /* OTHER METHODS */

  // This prefetechs hot RR node data required for optimization.
  // Note: This is optional, but may lower time spent on memory stalls in some circumstances.
  inline void prefetch_node(RRNodeId id) const {
    (void) id;
  }



    /* -- Mutators -- */
  public:
    void build_graph();

    void initialize_folded_rr_graph();

    void add_empty_pattern();

    

    /* -- Internal data storage -- */
  private:

  
/* Every tile has its own FoldedTilePattern. starting_node_id_ refers to the first node id in the tile.
     * node_patterns_idx_ refers to the index within node_patterns_ that contains the 
     */
    

  
    struct t_folded_node_data {
      int16_t cost_index_; // 2 Bytes
      int16_t rc_index_; // 2 Bytes

      int16_t length_; // 2 Bytes

      uint16_t capacity_; // 2 Bytes
      // t_rr_type type_ = NUM_RR_TYPES; // 1 Bytes // removing type to test it out. May add back later

      union {
          Direction direction_; //Valid only for CHANX/CHANY
          unsigned char sides_ = 0x0; //Valid only for IPINs/OPINs
      } dir_side_; // 1 Byte
    };


    // using SharedNodeDataIdx = uint16_t;
    // using NodePatternIdx = uint16_t;
    // using TileIdx = uint16_t;
    // using DxDyIdx = uint16_t;

    struct t_folded_edge_data {
      DxDyIdx dx_dy_idx = 0; // invalid
      int16_t switch_id = -1;
      TileIdx tile_idx = -1;
    };

    struct t_folded_node_edge_pattern {
      t_folded_node_data node_;
      std::vector<t_folded_edge_data> edges_; 
    };

  /* Pattern of data about a node. Many nodes will share the data within this struct and thus will have the same FoldedNodePattern */
    struct FoldedNodePattern { // 12 + 5*edges_.size() Bytes total
          int16_t cost_index_; // 2 Bytes
          int16_t rc_index_; // 2 Bytes

          int16_t dx_; // 2 Bytes
          int16_t dy_; // 2 Bytes

          uint16_t capacity_; // 2 Bytes

          union {
              Direction direction_; //Valid only for CHANX/CHANY
              unsigned char sides_ = 0x0; //Valid only for IPINs/OPINs
          } dir_side_; // 1 Byte

      };

    struct DxDy {
      int16_t dx_;
      int16_t dy_;
    };

    friend bool operator==(const t_folded_node_edge_pattern& lhs, const t_folded_node_edge_pattern& rhs)
          {
            return lhs.node_ == rhs.node_ &&
                   lhs.edges_ == rhs.edges_;
          }
    
    friend bool operator<(const t_folded_node_edge_pattern& lhs, const t_folded_node_edge_pattern& rhs)
          {
            if (lhs.node_ < rhs.node_) return true;
            else if (lhs.node_==rhs.node_){
                if (lhs.edges_ < rhs.edges_) return true;
              }
            return false;
          }

    friend bool operator==(const t_folded_edge_data& lhs, const t_folded_edge_data& rhs)
          {
            return lhs.dx_dy_idx == rhs.dx_dy_idx &&
                   lhs.switch_id == rhs.switch_id &&
                   lhs.tile_idx == rhs.tile_idx;
          }
    
    friend bool operator<(const t_folded_edge_data& lhs, const t_folded_edge_data& rhs)
          {
            if (lhs.dx_dy_idx < rhs.dx_dy_idx) return true;
            else if (lhs.dx_dy_idx==rhs.dx_dy_idx){
              if (lhs.switch_id < rhs.switch_id) return true;
              else if (lhs.switch_id==rhs.switch_id ){
                if (lhs.tile_idx < rhs.tile_idx) return true;
                }
              }
            return false;
          }
                
          
 

    friend bool operator==(const t_folded_node_data& lhs, const t_folded_node_data& rhs)
          {
            return lhs.cost_index_ == rhs.cost_index_ &&
                   lhs.rc_index_ == rhs.rc_index_ &&
                   lhs.length_ == rhs.length_ &&
                   lhs.dir_side_.direction_ == rhs.dir_side_.direction_ &&
                   lhs.dir_side_.sides_ == rhs.dir_side_.sides_ &&
                   lhs.capacity_ == rhs.capacity_;
          }
    
    friend bool operator<(const t_folded_node_data& lhs, const t_folded_node_data& rhs)
          {
            if (lhs.cost_index_ < rhs.cost_index_) return true;
            else if (lhs.cost_index_==rhs.cost_index_){
              if (lhs.rc_index_ < rhs.rc_index_) return true;
              else if (lhs.rc_index_==rhs.rc_index_ ){
                if (lhs.length_ < rhs.length_) return true;
                else if (lhs.length_==rhs.length_ ){
                  if (lhs.dir_side_.direction_ < rhs.dir_side_.direction_) return true;
                  else if (lhs.dir_side_.direction_==rhs.dir_side_.direction_ ){
                    if (lhs.dir_side_.sides_ < rhs.dir_side_.sides_) return true;
                    else if (lhs.dir_side_.sides_==rhs.dir_side_.sides_ ){
                      if (lhs.capacity_ < rhs.capacity_) return true;
                    }
                  }
                }
              }
            }
            return false;
          }

    friend bool operator==(const DxDy& lhs, const DxDy& rhs)
          {
            return lhs.dx_ == rhs.dx_ &&
                   lhs.dy_ == rhs.dy_;
          }
    
    friend bool operator<(const DxDy& lhs, const DxDy& rhs)
          {
            if (lhs.dx_ < rhs.dx_) return true;
            else if (lhs.dx_==rhs.dx_){
              if (lhs.dy_ < rhs.dy_) return true;
            }
            return false;
          }



    /*
    node_to_tile_:           RRNodeId -> x, y, tile_idx
    tile_to_node_:     x, y, tile_idx -> RRNodeId
    node_to_pattern_idx_:             RRNodeId -> NodePatternIdx

    shared_edges_:     NodePatternIdx -> edges array
    shared_node_:      NodePatternIdx -> SharedNodeDataIdx
    shared_node_data_:   SharedNodeDataIdx -> t_folded_node_data

    dx_dy_:                      DxDyIdx -> DxDy
    
    */


    vtr::vector<RRNodeId, std::array<TileIdx, 3>> node_to_tile_; // goes from RRNodeId to array of [x, y, tile_idx]
    vtr::vector<RRNodeId, t_rr_type> node_type_; // goes from RRNodeId to node type

    std::vector< std::vector< std::vector<RRNodeId>>> tile_to_node_; // goes from [x, y, tile_idx] to RRNodeId
    vtr::vector<RRNodeId, NodePatternIdx> node_to_pattern_idx_; // RRNodeId -> NodePatternIdx

    std::vector<std::vector<t_folded_edge_data>> shared_edges_; // goes from NodePatternIdx to array of edges
    std::vector<DxDy> dx_dy_; // goes from DxDyIdx to dx, dy

    std::vector<SharedNodeDataIdx> shared_node_; // goes from NodePatternIdx to shared_node_data_idx
    std::vector<t_folded_node_data> shared_node_data_; // goes from SharedNodeDataIdx to node_data


    size_t size_;
    size_t edge_count_;

    /* node-level storage including edge storages */
    t_rr_graph_storage& node_storage_;

    bool built = false; // flag for determining if the FoldedPerTileRRGraph has been built yet

};

#endif


