#ifndef FOLDED_EDGES_RR_GRAPH_H
#define FOLDED_EDGES_RR_GRAPH_H

#include "vpr_types.h"
#include <iostream>
#include "rr_graph_storage.h"
#include "rr_graph_view_interface.h"

/* memory usage for FoldedEdgesRRGraph data members is defined here in Bytes */
#define FOLDED_TILE_PATTERN_SIZE 8
#define FOLDED_EDGE_PATTERN_SIZE 7

class FoldedEdgesRRGraph : public RRGraphViewInterface{
    /* -- Constructors -- */
  public:
    /* Explicitly define the only way to create an object */
    explicit FoldedEdgesRRGraph(const t_rr_graph_storage& node_storage);

    /* Disable copy constructors and copy assignment operator
     * This is to avoid accidental copy because it could be an expensive operation considering that the 
     * memory footprint of the data structure could ~ Gb
     * Using the following syntax, we prohibit accidental 'pass-by-value' which can be immediately caught 
     * by compiler
     */
    FoldedEdgesRRGraph(const FoldedEdgesRRGraph&) = delete;
    void operator=(const FoldedEdgesRRGraph&) = delete;

    /* -- Accessors -- */
  public:
  // every function takes in an RRNodeId that has not been remapped yet
  /* The general method for accessing data from the FoldedEdgesRRGraph is as follows:
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
    return "FoldedEdgesRRGraph";
  }

  /* Return the number of RRNodes */
  size_t size() const{
      return size_;
  }

  inline size_t edge_count() const {
      return edges_size_;
  }

  inline t_rr_type node_type(RRNodeId node) const{ 
      return node_storage_.node_type(node);
  }

  inline short node_capacity(RRNodeId node) const{ 
      return node_storage_.node_capacity(node);
  }
  
  /* Get the type string of a routing resource node. This function is inlined for runtime optimization. */
  inline const char* node_type_string(RRNodeId node) const {
      return node_storage_.node_type_string(node);
  }

  /* Get the direction of a routing resource node. This function is inlined for runtime optimization.
    * Direction::INC: wire driver is positioned at the low-coordinate end of the wire.
    * Direction::DEC: wire_driver is positioned at the high-coordinate end of the wire.
    * Direction::BIDIR: wire has multiple drivers, so signals can travel either way along the wire
    * Direction::NONE: node does not have a direction, such as IPIN/OPIN
    */
  inline Direction node_direction(RRNodeId node) const {
      return node_storage_.node_direction(node);
  }

  /* Get the direction string of a routing resource node. This function is inlined for runtime optimization. */
  inline const std::string& node_direction_string(RRNodeId node) const {
      return node_storage_.node_direction_string(node);
  }

  /* Get the capacitance of a routing resource node. This function is inlined for runtime optimization. */
  float node_C(RRNodeId node) const;

  /* Get the resistance of a routing resource node. This function is inlined for runtime optimization. */
  float node_R(RRNodeId node) const;

  /* Get the rc_index of a routing resource node. This function is inlined for runtime optimization. */
  inline int16_t node_rc_index(RRNodeId node) const {
      return node_storage_.node_rc_index(node);
  }

  /* Get the xlow of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_xlow(RRNodeId node) const {
      return node_storage_.node_xlow(node);
  }

  /* Get the xhigh of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_xhigh(RRNodeId node) const {  
      return node_storage_.node_xhigh(node);
  }

  /* Get the ylow of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_ylow(RRNodeId node) const {
      return node_storage_.node_ylow(node);
  }

  /* Get the yhigh of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_yhigh(RRNodeId node) const {  
      return node_storage_.node_yhigh(node);
  }

  /* Get the cost index of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_cost_index(RRNodeId node) const {
      return node_storage_.node_cost_index(node);
  }

  /* Check whether a routing node is on a specific side. This function is inlined for runtime optimization. */
  inline bool is_node_on_specific_side(RRNodeId node, e_side side) const{
      return node_storage_.is_node_on_specific_side(node, side);
  }

  /* Check whether a routing node is on a specific side. This function is inlined for runtime optimization. */
  inline const char* node_side_string(RRNodeId node) const {// ⬛️
      return node_storage_.node_side_string(node);
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

  /* ------------ */
  /* Edge Methods */
  /* ------------ */


// a vector of FoldedEdge structs is returned by FoldedEdgesRRGraph::edge_range()
    struct FoldedEdge {
        size_t src_node;
        size_t dest_node;
        int8_t switch_id;
      }; 


  // Is the RR graph currently empty?
  inline bool empty() const {
        return node_storage_.empty();
    }

  // Estimate of memory taken by FoldedEdgesRRGraph
  inline int memory_used() const {
        /* Nodes */


        int nodes_memory = size() * 16;

        return nodes_memory;
    }

  /* must be called before the node_storage_ is deleted */
  inline int node_storage_memory_used() const{
      int memory_used = 0;
      for (uint id=0; id < node_storage_.size(); id++){
        memory_used += 16 + node_storage_.num_edges(RRNodeId(id)) * 9;
      }
      return memory_used;
  }

  /* Compare folded rr_graph vs node_storage_ */
  void verify_folded_rr_graph();

  /* PUBLIC EDGE METHODS */

  // input: legacy node
  inline edge_idx_range edges(const RRNodeId& legacy_node) const {
      return node_storage_.edges(legacy_node);
  }

  RREdgeId first_edge(const RRNodeId& legacy_node) const {
      return node_first_edge_id_[legacy_node];
  }

  RREdgeId last_edge(const RRNodeId& legacy_node) const {
      return (&node_first_edge_id_[legacy_node])[1];
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

  t_edge_size num_configurable_edges(const RRNodeId& id) const;
  t_edge_size num_non_configurable_edges(const RRNodeId& legacy_node) const {
      return num_edges(legacy_node) - num_configurable_edges(legacy_node);
  }


  // Returns a range of RREdgeId's belonging to RRNodeId id.
  //
  // If this range is empty, then RRNodeId id has no edges.
  vtr::StrongIdRange<RREdgeId> edge_range(const RRNodeId legacy_node) const {
      return vtr::StrongIdRange<RREdgeId>(first_edge(legacy_node), last_edge(legacy_node));
  }

inline RRNodeId edge_src_node(RREdgeId edge) const{
      return edge_src_node_[edge];
}

inline RRNodeId edge_sink_node(const RREdgeId& edge) const {
  RRNodeId src_node = edge_src_node_[edge];
  int offset = (size_t) edge - (size_t) node_first_edge_id_[src_node];
  return RRNodeId((size_t)src_node - (size_t)edge_data_[edge_pattern_data_[node_edges_idx_[src_node]][offset]].id_diff);
}


inline short edge_switch(const RREdgeId& edge) const {
  RRNodeId src_node = edge_src_node_[edge];
  int offset = (size_t) edge - (size_t) node_first_edge_id_[src_node];
  return edge_data_[edge_pattern_data_[node_edges_idx_[src_node]][offset]].switch_id;
}


// Call the `apply` function with the edge id, source, and sink nodes of every edge.
inline void for_each_edge(std::function<void(RREdgeId, RRNodeId, RRNodeId)> apply) const {
    for (size_t i = 0; i < edges_size_; i++) {
        RREdgeId edge(i);
        apply(edge, edge_src_node(edge), edge_sink_node(edge));
    }
}

inline RREdgeId edge_id(const RRNodeId& legacy_node, t_edge_size iedge) const {
    RREdgeId first_edge = this->first_edge(legacy_node); // this->first_edge(legacy_node);
    RREdgeId ret(size_t(first_edge) + iedge);
    VTR_ASSERT_SAFE(ret < last_edge(legacy_node));
    return ret;
}

// Get the destination node for the iedge'th edge from specified RRNodeId.
//
// This method should generally not be used, and instead first_edge and
// last_edge should be used.
RRNodeId edge_sink_node(const RRNodeId& legacy_node, t_edge_size iedge) const {
  return RRNodeId((size_t)legacy_node - (size_t)edge_data_[edge_pattern_data_[node_edges_idx_[legacy_node]][iedge]].id_diff);
}



// Get the switch used for the iedge'th edge from specified RRNodeId.
//
// This method should generally not be used, and instead first_edge and
// last_edge should be used.
short edge_switch(const RRNodeId& legacy_node, t_edge_size iedge) const {
  return edge_data_[edge_pattern_data_[node_edges_idx_[legacy_node]][iedge]].switch_id;
}

/* END EDGE METHODS */

/* OTHER METHODS */

  // This prefetechs hot RR node data required for optimization.
  // Note: This is optional, but may lower time spent on memory stalls in some circumstances.
  inline void prefetch_node(RRNodeId legacy_node) const {
    node_storage_.prefetch_node(legacy_node);
  }
  



    /* -- Mutators -- */
  public:
    void build_graph();


    

    /* -- Internal data storage -- */
  private:

  
/* Every tile has its own FoldedTilePattern. starting_node_id_ refers to the first node id in the tile.
     * node_patterns_idx_ refers to the index within node_patterns_ that contains the 
     */
    
    struct FoldedTilePattern { // 8 Bytes
        RRNodeId starting_node_id_ = RRNodeId(-1); // 4 Bytes
        int16_t node_patterns_idx_ = -1; // 2 Bytes  -> FoldedNodePattern
        int16_t edge_patterns_idx_ = -1; // 2 Bytes  -> FoldedEdgePattern
      }; 

    struct FoldedEdgePattern { // 7 Bytes
        int32_t id_diff; // 4 Bytes
        int8_t switch_id; // 1 Byte
      }; 

    

  /* Pattern of data about a node. Many nodes will share the data within this struct and thus will have the same FoldedNodePattern */
    struct FoldedNodePattern { // 12 + 5*edges_.size() Bytes total
          int16_t cost_index_; // 2 Bytes
          int16_t rc_index_; // 2 Bytes

          int16_t dx_; // 2 Bytes
          int16_t dy_; // 2 Bytes

          t_rr_type type_; // 1 Byte

          uint16_t capacity_; // 2 Bytes

          union {
              Direction direction_; //Valid only for CHANX/CHANY
              unsigned char sides_ = 0x0; //Valid only for IPINs/OPINs
          } dir_side_; // 1 Byte

      };










/*
    inline void set_node_edges(RRNodeId node, std::vector<FoldedEdgePattern> edges) {
      RRNodeId remapped_id = remapped_ids_[node];
      //std::array<size_t, 2> x_y = find_tile_coords(node);
      auto tile = tile_patterns_[legacy_node_to_x_y_[node][0]][legacy_node_to_x_y_[node][1]];
      int node_patterns_idx = tile.node_patterns_idx_;
      size_t offset = (size_t) remapped_id - (size_t) tile.starting_node_id_;
      //node_data_[node_patterns_[node_patterns_idx][offset]].edges_ = edges;
    } 
    */




    friend bool operator==(const FoldedNodePattern& lhs, const FoldedNodePattern& rhs)
          {
            return lhs.cost_index_ == rhs.cost_index_ &&
                   lhs.rc_index_ == rhs.rc_index_ &&
                   lhs.dx_ == rhs.dx_ &&
                   lhs.dy_ == rhs.dy_ &&
                   lhs.type_ == rhs.type_ &&
                   lhs.dir_side_.direction_ == rhs.dir_side_.direction_ &&
                   lhs.dir_side_.sides_ == rhs.dir_side_.sides_ &&
                   lhs.capacity_ == rhs.capacity_;

          }

    

    
    friend bool operator==(const FoldedEdgePattern& lhs, const FoldedEdgePattern& rhs)
        {
          return lhs.id_diff == rhs.id_diff &&
                  lhs.switch_id == rhs.switch_id;
        }


    /*        GENERAL NODE DATA FLOW

    tile X-----------.
    tile Y--------.  |
                  V  V
    tile_patterns_[x][y].node_patterns_idx----.
                                             |
    relative_id_offset-----------------------|-----.
                                             V     V             
                              node_patterns[idx][offset]-----.
                                                             V
                                          node_data_[idx]


    // to find X and Y
    tile_patterns_[x][y].starting_node_id_ < RRNodeId < tile_patterns_[any_other_x][any_other_y].starting_node_id_

    // to find relative_id_offset
    RRNodeId - tile_patterns_[x][y].starting_node_id_

              GENERAL EDGE DATA FLOW

    tile X-----------.
    tile Y--------.  |
                  V  V
    tile_patterns_[x][y].node_patterns_idx----.
                                             |
    relative_id_offset-----------------------|-----.
                                             V     V             
                              node_patterns[idx][offset]-----.
                                                             V
                                          node_data_[idx]

    */

    /* Every Tile Type has a set of FoldedEdgePatterns */
    // std::vector<std::vector<int16_t>> edge_patterns_; 

    /* Indexes into edge_data_ are stored here */
    std::vector<std::vector<int32_t>> edge_pattern_data_; // vector of vectors of indexes into edge_data_

    /* Raw FoldedEdgePattern data is stored here */
    std::vector<FoldedEdgePattern> edge_data_; // this is where actual edge data is stored

    /* First edge id of given node (RRNodeId is a remapped node)*/
    vtr::vector<RRNodeId, RREdgeId> node_first_edge_id_; // every node

    vtr::vector<RRNodeId, int32_t> node_edges_idx_; // every node

    /* FoldedEdge's legacy_src_node*/
    vtr::vector<RREdgeId, RRNodeId> edge_src_node_; // every edge

    size_t size_;

    /* number of edges */
    size_t edges_size_;

    /* node-level storage including edge storages */
    const t_rr_graph_storage& node_storage_;

    bool built = false; // flag for determining if the FoldedEdgesRRGraph has been built yet

};

#endif
