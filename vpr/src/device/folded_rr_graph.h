#ifndef FOLDED_RR_GRAPH_H
#define FOLDED_RR_GRAPH_H

#include "vpr_types.h"
#include <iostream>
#include "rr_graph_storage.h"

class FoldedRRGraph {
    /* -- Constructors -- */
  public:
    /* Explicitly define the only way to create an object */
    explicit FoldedRRGraph(const t_rr_graph_storage& node_storage);

    /* Disable copy constructors and copy assignment operator
     * This is to avoid accidental copy because it could be an expensive operation considering that the 
     * memory footprint of the data structure could ~ Gb
     * Using the following syntax, we prohibit accidental 'pass-by-value' which can be immediately caught 
     * by compiler
     */
    FoldedRRGraph(const FoldedRRGraph&) = delete;
    void operator=(const FoldedRRGraph&) = delete;

    /* -- Accessors -- */
  public:
  // dx, dy, type, capacity, direction, side, C, R, segment
  t_rr_type node_type(RRNodeId id) const {
      int idx = rr_node_id_to_x_y_idx_[id][2];
      return all_node_patterns_[idx].type;
    }
   
  short node_capacity(RRNodeId id) const {
        int idx = rr_node_id_to_x_y_idx_[id][2];
        return all_node_patterns_[idx].capacity;
    }

    /* -- Mutators -- */
  public:
    void build_folded_rr_graph();

    void initialize_folded_rr_graph();

    void add_empty_pattern();

    

    /* -- Internal data storage -- */
  private:

    struct FoldedNodePattern {
      int dx;
      int dy;
      t_rr_type type;
      short capacity;
      Direction direction;
      std::string side;
      float C;
      float R;
      std::string segment;
    };  

    /* TODO: When the refactoring effort finishes, 
     * the data structure will be the owner of the data storages. 
     * That is why the reference is used here.
     * It can avoid a lot of code changes once the refactoring is finished 
     * (there is no function get data directly through the rr_node_indices in DeviceContext).
     * If pointers are used, it may cause many codes in client functions 
     * or inside the data structures to be changed later.
     * That explains why the reference is used here temporarily
     */

    // Contains every pattern needed to specify an RRNode's data members
    // dx, dy, type, capacity, direction, side, C, R, segment
    std::vector<FoldedNodePattern> all_node_patterns_;
    // Values in array are: x, y, pattern_idx
    vtr::vector<RRNodeId, std::array<int, 3>> rr_node_id_to_x_y_idx_;
    /* node-level storage including edge storages */
    const t_rr_graph_storage& node_storage_;
};

#endif
