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
  // every function takes in an RRNodeId that has not been remapped yet
  inline t_rr_type node_type(RRNodeId id) const{
      if (!built){
        return node_storage_.node_type(id);
      }
      RRNodeId remapped_id = remapped_ids_[id];
      std::array<size_t, 2> x_y = find_tile_coords(id);
      auto tile = tile_patterns[x_y[0]][x_y[1]];
      int node_patterns_idx = tile.node_patterns_idx_;
      size_t offset = (size_t) remapped_id - (size_t) tile.starting_node_id_;
      return node_pattern_data[node_patterns[node_patterns_idx][offset]].type_;
    }

  std::array<size_t, 2> find_tile_coords(RRNodeId id) const {
      RRNodeId remapped_id = remapped_ids_[id];
      std::array<size_t, 2> return_x_y;
      size_t min_id = (size_t) remapped_id;
      for (size_t x = 0; x < tile_patterns.size(); x++){ // iterate over x
        for (size_t y = 0; y < tile_patterns[x].size(); y++){ // iterate over y
          if ( (size_t) tile_patterns[x][y].starting_node_id_ <= min_id){
            return_x_y = {x, y};
          }
        }
      }
        return return_x_y; 
  }

  inline short node_capacity(RRNodeId id) const{
      if (!built){
        return node_storage_.node_type(id);
      }
      RRNodeId remapped_id = remapped_ids_[id];
      std::array<size_t, 2> x_y = find_tile_coords(id);
      auto tile = tile_patterns[x_y[0]][x_y[1]];
      int node_patterns_idx = tile.node_patterns_idx_;
      size_t offset = (size_t) remapped_id - (size_t) tile.starting_node_id_;
      return node_pattern_data[node_patterns[node_patterns_idx][offset]].capacity_;
    }












    /* -- Mutators -- */
  public:
    void build_folded_rr_graph();

    void initialize_folded_rr_graph();

    void add_empty_pattern();

    

    /* -- Internal data storage -- */
  private:

    struct FoldedNodePattern { // 12 Bytes
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


      

    struct FoldedTilePattern { // 6 Bytes
        RRNodeId starting_node_id_ = RRNodeId(-1); // 4 Bytes
        int16_t node_patterns_idx_ = -1; // 2 Bytes  -> FoldedNodePattern
      }; 

    struct AbnormalWire {
      int16_t x; // 2 Bytes
      int16_t y; // 2 Bytes
      int16_t node_pattern_data_idx; // 2 Bytes
    };

    // 2d vector the indexes into the vector are the tile's x and y position
    std::vector<std::vector<FoldedTilePattern>> tile_patterns; // Every tile has a starting_node_id and node_patterns_idx

    std::vector<std::vector<int>> node_patterns; // Every Tile Type has a set of FoldedNodePatterns

    std::vector<FoldedNodePattern> node_pattern_data;

    // due to the current gaps in ids, this vector remaps them so that there are no gaps
    vtr::vector<RRNodeId, RRNodeId> remapped_ids_;


    /* node-level storage including edge storages */
    const t_rr_graph_storage& node_storage_;

    bool built = false;


};

#endif
