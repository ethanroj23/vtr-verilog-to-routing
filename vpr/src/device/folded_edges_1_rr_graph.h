#ifndef FOLDED_EDGES_1_RR_GRAPH_H
#define FOLDED_EDGES_1_RR_GRAPH_H

/*
  Folded Edges RRGraph 1 folds the edges like so:
    Node:
      first_edge_id // access this using a binary search
    Edge:
      sink_node
      switch
*/

#include "vpr_types.h"
#include <iostream>
#include "rr_graph_storage.h"
#include "rr_graph_view_interface.h"

/* memory usage for FoldedEdges1RRGraph data members is defined here in Bytes */
#define FOLDED_TILE_PATTERN_SIZE 8
#define FOLDED_EDGE_PATTERN_SIZE 7

class FoldedEdges1RRGraph : public RRGraphViewInterface{
    /* -- Constructors -- */
  public:
    /* Explicitly define the only way to create an object */
    explicit FoldedEdges1RRGraph(const t_rr_graph_storage& node_storage);
    FoldedEdges1RRGraph(const FoldedEdges1RRGraph&) = delete;
    void operator=(const FoldedEdges1RRGraph&) = delete;

    /* -- Accessors -- */
  public:
  inline const char* rr_graph_name() const{
    return "FoldedEdges1RRGraph";
  }

  /* Return the number of RRNodes */
  size_t size() const{return size_;}
  inline size_t edge_count() const {return edges_size_;}
  
  inline t_rr_type node_type(RRNodeId node) const {return node_storage_.node_type(node);}
  inline const char* node_type_string(RRNodeId node) const {return node_storage_.node_type_string(node);}
  inline short node_capacity(RRNodeId node) const {return node_storage_.node_capacity(node);}
  inline Direction node_direction(RRNodeId node) const {return node_storage_.node_direction(node);}
  inline const std::string& node_direction_string(RRNodeId node) const {return node_storage_.node_direction_string(node);}

  float node_C(RRNodeId node) const;
  float node_R(RRNodeId node) const;
  inline int16_t node_rc_index(RRNodeId node) const {return node_storage_.node_rc_index(node);}
  
  inline short node_xlow(RRNodeId node) const {return node_storage_.node_xlow(node);}
  inline short node_xhigh(RRNodeId node) const {  return node_storage_.node_xhigh(node);}
  inline short node_ylow(RRNodeId node) const {return node_storage_.node_ylow(node);}
  inline short node_yhigh(RRNodeId node) const {  return node_storage_.node_yhigh(node);}

  inline short node_cost_index(RRNodeId node) const {return node_storage_.node_cost_index(node);}
  inline bool is_node_on_specific_side(RRNodeId node, e_side side) const{return node_storage_.is_node_on_specific_side(node, side);}
  inline const char* node_side_string(RRNodeId node) const {return node_storage_.node_side_string(node);}
  inline t_edge_size node_fan_in(RRNodeId node) const {return node_storage_.fan_in(node);}
  inline t_edge_size fan_in(RRNodeId node) const {return node_storage_.fan_in(node);}

  short node_ptc_num(RRNodeId id) const;
  short node_pin_num(RRNodeId id) const;   //Same as ptc_num() but checks that type() is consistent
  short node_track_num(RRNodeId id) const; //Same as ptc_num() but checks that type() is consistent
  short node_class_num(RRNodeId id) const; //Same as ptc_num() but checks that type() is consistent

  /* ------------ */
  /* Edge Methods */
  /* ------------ */


// a vector of FoldedEdge structs is returned by FoldedEdges1RRGraph::edge_range()
    struct FoldedEdge {
        size_t src_node;
        size_t dest_node;
        int8_t switch_id;
      }; 


  // Is the RR graph currently empty?
  inline bool empty() const {
        return node_first_edge_.empty();
    }

  // Estimate of memory taken by FoldedEdges1RRGraph
  inline int memory_used() const {return node_bytes() + edge_bytes();}
  inline int node_bytes() const {return size() * 16;}
  inline int edge_bytes() const {return edge_dest_node_.size() * 6;}

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
      return node_first_edge_[legacy_node];
  }

  RREdgeId last_edge(const RRNodeId& legacy_node) const {
      return (&node_first_edge_[legacy_node])[1];
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

  int lower_bound = 0;
  int upper_bound = size();
  int count = 0;
  int midpoint;
  int prev_midpoint;
  while (true){
    count++;
    midpoint = lower_bound + (upper_bound - lower_bound) / 2;
    if (prev_midpoint == midpoint){
      return RRNodeId(midpoint-1);
    }
    prev_midpoint = midpoint;
    int rel = size_t(node_first_edge_[RRNodeId(midpoint)]) - size_t(edge);
    if (rel < 0){ // a < x
      lower_bound = midpoint + 1;
    }
    else if (rel > 0){ // a > x
      upper_bound = midpoint - 1;
    }
    else{
      while(first_edge(RRNodeId(midpoint)) == last_edge(RRNodeId(midpoint))){
        midpoint++;
      }
      return RRNodeId(midpoint);
    }

  }
}

// while (True):
//     count += 1
//     midpoint = int(lower_bound + (upper_bound - lower_bound) / 2)
//     print('midpoint val: ', a[midpoint])
//     if a[midpoint] < x:
//         lower_bound = midpoint + 1
//     elif a[midpoint] > x:
//         upper_bound = midpoint - 1
//     else:
//         print(str(x) + ' was found at idx ' + str(midpoint) + ' after ' + str(count) + ' loops\n')
//         exit()

  // int node_mid_point_ = size() / 2;
  // size_t cur_idx = node_mid_point_;
  // size_t mid_point = node_mid_point_;
  // size_t range = size() / 2;
  // int rel_location;
  // while (true) {
  //   rel_location = size_t(edge) - size_t(node_first_edge_[RRNodeId(cur_idx)]);
  //   range = (range + 1) / 2;

  //   if (range==0){
  //     while (node_first_edge_[RRNodeId(cur_idx)]==node_first_edge_[RRNodeId(cur_idx+1)])
  //       cur_idx++;
    //   return RRNodeId(cur_idx);
    // }
    // if (rel_location < 0){
    //   cur_idx = cur_idx - range;
    // }
    // else if (rel_location > 0) {
    //   cur_idx = cur_idx + range;
    // }
    // else {
    //   while (node_first_edge_[RRNodeId(cur_idx)]==node_first_edge_[RRNodeId(cur_idx+1)])
  //       cur_idx++;
  //     return RRNodeId(cur_idx);
  //   }
  // }
// }


inline RRNodeId edge_sink_node(const RREdgeId& edge) const {
  return edge_dest_node_[edge];
}


inline short edge_switch(const RREdgeId& edge) const {
  return edge_switch_[edge];

}


// Call the `apply` function with the edge id, source, and sink nodes of every edge.
inline void for_each_edge(std::function<void(RREdgeId, RRNodeId, RRNodeId)> apply) const {
    for (size_t i = 0; i < edges_size_; i++) {
        RREdgeId edge(i);
        apply(edge, edge_src_node(edge), edge_sink_node(edge));
    }
}

// Call the `apply` function with the edge id, source, and sink nodes of every edge.
inline void for_each_edge_no_src(std::function<void(RREdgeId, RRNodeId)> apply) const {
    for (size_t i = 0; i < edges_size_; i++) {
        RREdgeId edge(i);
        apply(edge,  edge_sink_node(edge));
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
        return edge_sink_node(edge_id(legacy_node, iedge));
  }



// Get the switch used for the iedge'th edge from specified RRNodeId.
//
// This method should generally not be used, and instead first_edge and
// last_edge should be used.
short edge_switch(const RRNodeId& legacy_node, t_edge_size iedge) const {
        return edge_switch(edge_id(legacy_node, iedge));
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




    /* Raw FoldedEdgePattern data is stored here */
    std::vector<FoldedEdgePattern> edge_data_; // this is where actual edge data is stored

    // vtr::array_view_id<RRNodeId, const RREdgeId> node_first_edge_;
    vtr::vector<RRNodeId, RREdgeId> node_first_edge_; // every node
    vtr::vector<RREdgeId, RRNodeId> edge_dest_node_; // every node
    vtr::vector<RREdgeId, short> edge_switch_; // every node


    // vtr::array_view_id<RREdgeId, const RRNodeId> edge_dest_node_;
    // vtr::array_view_id<RREdgeId, const short> edge_switch_;

    size_t size_;

    /* number of edges */
    size_t edges_size_;

    /* node-level storage including edge storages */
    const t_rr_graph_storage& node_storage_;

    bool built = false; // flag for determining if the FoldedEdges1RRGraph has been built yet

};

#endif
