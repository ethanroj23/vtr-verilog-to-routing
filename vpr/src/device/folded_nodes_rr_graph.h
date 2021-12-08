#ifndef FOLDED_1_RR_GRAPH_H
#define FOLDED_1_RR_GRAPH_H

#include "vpr_types.h"
#include <iostream>
#include "rr_graph_storage.h"
#include "rr_graph_view_interface.h"

/* memory usage for FoldedNodesRRGraph data members is defined here in Bytes */
#define FOLDED_TILE_PATTERN_SIZE 8
#define FOLDED_EDGE_PATTERN_SIZE 7

class FoldedNodesRRGraph : public RRGraphViewInterface{
    /* -- Constructors -- */
  public:
    /* Explicitly define the only way to create an object */
    explicit FoldedNodesRRGraph(t_rr_graph_storage& node_storage);

    /* Disable copy constructors and copy assignment operator
     * This is to avoid accidental copy because it could be an expensive operation considering that the 
     * memory footprint of the data structure could ~ Gb
     * Using the following syntax, we prohibit accidental 'pass-by-value' which can be immediately caught 
     * by compiler
     */
    FoldedNodesRRGraph(const FoldedNodesRRGraph&) = delete;
    void operator=(const FoldedNodesRRGraph&) = delete;

    /* -- Accessors -- */
  public:
  // every function takes in an RRNodeId that has not been remapped yet
  /* The general method for accessing data from the FoldedNodesRRGraph is as follows:
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
    return "FoldedNodesRRGraph";
  }

  /* Return the number of RRNodes */
  size_t size() const{
      return size_;
  }

  /* Return the number of bytes used to represent the nodes */
  int node_bytes() const{
      return nodes_.size() * 6 + node_patterns_.size() * 12;
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
        // return node_storage_.edge_count();
        return 0; // temp
    }

  inline t_rr_type node_type(RRNodeId node) const{ 
      return nodes_[node].type_;
  }

  inline short node_capacity(RRNodeId node) const{ 
    return get_node_pattern(node).capacity_;
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
    return get_node_pattern(node).dir_side_.direction_;
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

  /* Get the rc_index of a routing resource node. This function is inlined for runtime optimization. */
  inline int16_t node_rc_index(RRNodeId node) const {
      return get_node_pattern(node).rc_index_;
  }

  /* Get the xlow of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_xlow(RRNodeId node) const {
      return nodes_[node].xlow_;
  }

  /* Get the xhigh of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_xhigh(RRNodeId node) const {  
      return nodes_[node].xlow_ + get_node_pattern(node).dx_;
  }

  /* Get the ylow of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_ylow(RRNodeId node) const {
      return nodes_[node].ylow_;
  }

  /* Get the yhigh of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_yhigh(RRNodeId node) const {  
      return nodes_[node].ylow_ + get_node_pattern(node).dy_;
  }

  /* Get the cost index of a routing resource node. This function is inlined for runtime optimization. */
  inline RRIndexedDataId node_cost_index(RRNodeId node) const {
    return RRIndexedDataId(get_node_pattern(node).cost_index_);
  }

  /* Check whether a routing node is on a specific side. This function is inlined for runtime optimization. */
  inline bool is_node_on_specific_side(RRNodeId node, e_side side) const{
    t_rr_type current_type = node_type(node);
    if (current_type != IPIN && current_type != OPIN){
        VPR_FATAL_ERROR(VPR_ERROR_ROUTE,
          "Attempted to access RR node 'side' for non-IPIN/OPIN type '%s'",
          rr_node_typename[current_type]);
    }
    return SIDES[get_node_pattern(node).dir_side_.sides_]==side;
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

  /* Get all data of a routing resource node. This function is inlined for runtime optimization. */
  inline t_rr_node_loc node_loc(RRNodeId node) const {
    FoldedNodePattern p = node_patterns_[nodes_[node].node_data_index_];
    int16_t xlow = nodes_[node].xlow_;
    int16_t ylow = nodes_[node].ylow_;
    // t_rr_node_loc loc = {
    //     nodes_[node].xlow_,
    //     nodes_[node].ylow_,
    //     int16_t(xlow + p.dx_),
    //     int16_t(ylow + p.dy_)
    // };
    return t_rr_node_loc(
        {xlow,
        ylow,
        int16_t(xlow + p.dx_),
        int16_t(ylow + p.dy_)}
    );
  }

  inline t_rr_node_loc_low node_loc_low(RRNodeId node) const {
    return t_rr_node_loc_low(
        {nodes_[node].xlow_,
        nodes_[node].ylow_}
    );
  }

  inline t_rr_node_loc_high node_loc_high(RRNodeId node) const {
    return t_rr_node_loc_high(
        {int16_t(nodes_[node].xlow_ + node_patterns_[nodes_[node].node_data_index_].dx_),
         int16_t(nodes_[node].ylow_ + node_patterns_[nodes_[node].node_data_index_].dy_)}
    );
  }



  /* PTC get methods */
  short node_ptc_num(RRNodeId id) const;
  short node_pin_num(RRNodeId id) const;   //Same as ptc_num() but checks that type() is consistent
  short node_track_num(RRNodeId id) const; //Same as ptc_num() but checks that type() is consistent
  short node_class_num(RRNodeId id) const; //Same as ptc_num() but checks that type() is consistent

  /* Edge Methods */


  // Is the RR graph currently empty?
  inline bool empty() const {
        return nodes_.empty();
    }

  // Estimate of memory taken by FoldedNodesRRGraph
  inline int memory_used() const {
      return node_bytes() + edge_bytes();
    }

  /* must be called before the node_storage_ is deleted */
  inline int node_storage_bytes() const{
      return nodes_.size() * 16 + edge_count() * 10;
  }

  /* Compare folded rr_graph vs node_storage_ */
  void verify_folded_rr_graph();

  /* PUBLIC EDGE METHODS */

  // input: legacy node
  inline edge_idx_range edges(const RRNodeId& legacy_node) const {
    return node_storage_.edges(legacy_node);
  }

  RREdgeId first_edge(const RRNodeId& legacy_node) const {
    return node_storage_.first_edge(legacy_node);
  }

  RREdgeId last_edge(const RRNodeId& legacy_node) const {
    return node_storage_.last_edge(legacy_node);
  }

  /* Edges are configurable if they have a switch that is configurable vtr/libs/libarchfpga/src/physical_types.cpp:83 */
  edge_idx_range configurable_edges(const RRNodeId& legacy_node) const {
    return node_storage_.configurable_edges(legacy_node);
    }
  edge_idx_range non_configurable_edges(const RRNodeId& legacy_node) const {
    return node_storage_.non_configurable_edges(legacy_node);
  }

  t_edge_size num_edges(const RRNodeId& legacy_node) const {
    return node_storage_.num_edges(legacy_node);
  }
    t_edge_size num_configurable_edges(const RRNodeId& legacy_node) const {
    return node_storage_.num_configurable_edges(legacy_node);
  }

  t_edge_size num_non_configurable_edges(const RRNodeId& legacy_node) const {
    return node_storage_.num_non_configurable_edges(legacy_node);
  }

  // Returns a range of RREdgeId's belonging to RRNodeId id.
  //
  // If this range is empty, then RRNodeId id has no edges.
  vtr::StrongIdRange<RREdgeId> edge_range(const RRNodeId legacy_node) const {
    return node_storage_.edge_range(legacy_node);
  }

  /*
    input:  RREdgeId (remapped)
    output: RRNodeId (remapped)
  */
// inline RRNodeId edge_src_node(RREdgeId legacy_node) const{
//     return node_storage_.edge_src_node(legacy_node);
// }

  inline RRNodeId edge_sink_node(const RREdgeId& legacy_node) const {
    return node_storage_.edge_sink_node(legacy_node);
}

inline short edge_switch(const RREdgeId& legacy_node) const {
    return node_storage_.edge_switch(legacy_node);
}

    // for (RREdgeId edge : rr_graph.edge_range(prev_node)) {//ESR_EDGE iterate over edges


// Call the `apply` function with the edge id, source, and sink nodes of every edge.
// inline void for_each_edge(std::function<void(RREdgeId, RRNodeId, RRNodeId)> apply) const {
//     return node_storage_.for_each_edge(apply);
// }

// Call the `apply` function with the edge id, source, and sink nodes of every edge.
void for_each_edge(std::function<void(RREdgeId, RRNodeId, RRNodeId)> apply) const {
    for (size_t node = 0; node < size_; node++){
      for (RREdgeId edge : edge_range(RRNodeId(node))) {//ESR_EDGE iterate over edges
        apply(edge, RRNodeId(node), edge_dest_node_[edge]);
      }
    }
}

// Call the `apply` function with the edge id, source, and sink nodes of every edge.
// inline void for_each_edge_no_src(std::function<void(RREdgeId, RRNodeId)> apply) const {
//     return node_storage_.for_each_edge_no_src(apply);
// }

inline RREdgeId edge_id(const RRNodeId& legacy_node, t_edge_size iedge) const {
    return node_storage_.edge_id(legacy_node, iedge);
}

// Get the destination node for the iedge'th edge from specified RRNodeId.
//
// This method should generally not be used, and instead first_edge and
// last_edge should be used.
RRNodeId edge_sink_node(const RRNodeId& legacy_node, t_edge_size iedge) const {
    return node_storage_.edge_sink_node(legacy_node, iedge);
}

// Get the switch used for the iedge'th edge from specified RRNodeId.
//
// This method should generally not be used, and instead first_edge and
// last_edge should be used.
short edge_switch(const RRNodeId& legacy_node, t_edge_size iedge) const {
    return edge_switch(edge_id(legacy_node, iedge));
}

  /* OTHER METHODS */

  // This prefetechs hot RR node data required for optimization.
  // Note: This is optional, but may lower time spent on memory stalls in some circumstances.
  inline void prefetch_node(RRNodeId id) const {
    VTR_PREFETCH(&nodes_[id], 0, 0);
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
      int16_t node_data_index_ = -1;
      int16_t xlow_ = -1;
      int16_t ylow_ = -1;
      t_rr_type type_ = NUM_RR_TYPES;
    };

    struct t_folded_edge_data {
      size_t diff = -1;
      int16_t switch_id = -1;
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

    /* Obtain node_data_ for specific node id */
    inline FoldedNodePattern get_node_pattern(RRNodeId node) const {
      return node_patterns_[nodes_[node].node_data_index_];
    } 
 

    friend bool operator==(const FoldedNodePattern& lhs, const FoldedNodePattern& rhs)
          {
            return lhs.cost_index_ == rhs.cost_index_ &&
                   lhs.rc_index_ == rhs.rc_index_ &&
                   lhs.dx_ == rhs.dx_ &&
                   lhs.dy_ == rhs.dy_ &&
                   lhs.dir_side_.direction_ == rhs.dir_side_.direction_ &&
                   lhs.dir_side_.sides_ == rhs.dir_side_.sides_ &&
                   lhs.capacity_ == rhs.capacity_;
          }
    
    friend bool operator<(const FoldedNodePattern& lhs, const FoldedNodePattern& rhs)
          {
            if (lhs.cost_index_ < rhs.cost_index_) return true;
            else if (lhs.cost_index_==rhs.cost_index_){
              if (lhs.rc_index_ < rhs.rc_index_) return true;
              else if (lhs.rc_index_==rhs.rc_index_ ){
                if (lhs.dx_ < rhs.dx_) return true;
                else if (lhs.dx_==rhs.dx_ ){
                  if (lhs.dy_ < rhs.dy_) return true;
                  else if (lhs.dy_==rhs.dy_ ){
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
            }
            return false;

            // return lhs.cost_index_ < rhs.cost_index_ ||
            //        lhs.rc_index_ < rhs.rc_index_ ||
            //        lhs.dx_ < rhs.dx_ ||
            //        lhs.dy_ < rhs.dy_ ||
            //        lhs.dir_side_.direction_ < rhs.dir_side_.direction_ ||
            //        lhs.dir_side_.sides_ < rhs.dir_side_.sides_ ||
            //        lhs.capacity_ < rhs.capacity_;
          }

/*
  You want to have a hierarchy of comparisons.
  if (a.1 < b.1) return 1
  else if (a.1 == b.1 && a.2 < b.2) return 1
  else if
  compare(a, b){
    if a.main < b.main{
      return a
    }
    else
  }
int compare_x_y(const void * a, const void * b){
    const point * a_point = (point*) a;
    const point * b_point = (point*) b;
    if (a_point->x > b_point->x) return 1;
    else if (a_point->x == b_point->x){
        if (a_point->y > b_point->y) return 1;
        else return -1;
    }
    else return -1;
}

*/


    /* Raw FoldedNodePattern data is stored here */
    std::vector<FoldedNodePattern> node_patterns_; // should probably be called node_data_ to match edge_data_
    //vtr::vector<RRNodeId, t_folded_node_data, vtr::aligned_allocator<t_folded_node_data>> nodes_;
    vtr::vector<RRNodeId, t_folded_node_data> nodes_;
    vtr::vector<RREdgeId, size_t> edge_data_idx_;
    vtr::vector<RRNodeId, t_folded_edge_data> edge_data_;
    vtr::vector<RREdgeId, RRNodeId> edge_dest_node_;

    // vtr::array_view_id<RRNodeId, t_folded_node_data> nodes_;


    size_t size_;

    /* node-level storage including edge storages */
    t_rr_graph_storage& node_storage_;

    bool built = false; // flag for determining if the FoldedNodesRRGraph has been built yet

};

#endif
