#ifndef FOLDED_RR_GRAPH_H
#define FOLDED_RR_GRAPH_H

#include "vpr_types.h"
#include <iostream>
#include "rr_graph_storage.h"
#include "rr_graph_view_interface.h"

class FoldedRRGraph : public RRGraphViewInterface{
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
  // every function takes in an RRNodeId that has not been remapped yet
  /* The general method for accessing data from the FoldedRRGraph is as follows:
   * 1. Obtain the remapped_id from remapped_ids data member.v 
   * 2. Find the x and y coordinates of the tile that contains the input RRNode
   * 3. Obtain the node_patterns_idx from the tile
   * 4. Calculate the relative offset for the given RRNodeId
   * 5. Access the desired data with something like 
   *      node_pattern_data_[node_patterns[node_patterns_idx][offset]].type_; //replace "type_" with the data you are wanting
   */

  /*Return an RRNode's type.*/

  /* Print the current rr_graph type */
  inline const char* rr_graph_name() const{
    return "FoldedRRGraph";
  }

  /* Return the number of RRNodes */
  size_t size() const{
      return size_;
  }

  std::array<size_t, 2> find_tile_coords(RRNodeId id) const {
      RRNodeId remapped_id = remapped_ids_[id];
      std::array<size_t, 2> return_x_y;
      size_t min_id = (size_t) remapped_id;
      for (size_t x = 0; x < tile_patterns_.size(); x++){ // iterate over x
        for (size_t y = 0; y < tile_patterns_[x].size(); y++){ // iterate over y
          if ( (size_t) tile_patterns_[x][y].starting_node_id_ <= min_id){
            return_x_y = {x, y};
          }
        }
      }
        return return_x_y; 
  }

  inline t_rr_type node_type(RRNodeId node) const{ 
      return get_node_pattern(node).type_;
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
      return node_to_x_y_[node][0];
  }

  /* Get the xhigh of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_xhigh(RRNodeId node) const {  
      RRNodeId remapped_id = remapped_ids_[node];
      //std::array<size_t, 2> x_y = find_tile_coords(node);
      auto tile = tile_patterns_[node_to_x_y_[node][0]][node_to_x_y_[node][1]];
      int node_patterns_idx = tile.node_patterns_idx_;
      size_t offset = (size_t) remapped_id - (size_t) tile.starting_node_id_;
      return node_to_x_y_[node][0] + node_pattern_data_[node_patterns_[node_patterns_idx][offset]].dx_;
  }

  /* Get the ylow of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_ylow(RRNodeId node) const {
      return node_to_x_y_[node][1];
  }

  /* Get the yhigh of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_yhigh(RRNodeId node) const {  
      RRNodeId remapped_id = remapped_ids_[node];
      //std::array<size_t, 2> x_y = find_tile_coords(node);
      auto tile = tile_patterns_[node_to_x_y_[node][0]][node_to_x_y_[node][1]];
      int node_patterns_idx = tile.node_patterns_idx_;
      size_t offset = (size_t) remapped_id - (size_t) tile.starting_node_id_;
      return node_to_x_y_[node][1] + node_pattern_data_[node_patterns_[node_patterns_idx][offset]].dy_;
  }

  /* Get the cost index of a routing resource node. This function is inlined for runtime optimization. */
  inline short node_cost_index(RRNodeId node) const {
    return get_node_pattern(node).cost_index_;
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
    return node_storage_.node_fan_in(node);
  }

  // Is the RR graph currently empty?
  inline bool empty() const {
        return node_patterns_.empty();
    }

  // Estimate of memory taken by FoldedRRGraph
  inline int memory_used() const {
        //int tile_patterns_memory;
        //int node_patterns_memory = node_patterns_.size() * ;
        int tile_patterns_memory = 0;
        int node_patterns_memory = 0;
        for (auto x : tile_patterns_){
            tile_patterns_memory += x.size() * 6; // 6 bytes stored for every TilePattern struct
        }
        for (auto pattern_list : node_patterns_){
            node_patterns_memory += pattern_list.size() * 2; // 2 bytes for each idx into node_pattern_data_
        }
        int node_pattern_data_memory = node_pattern_data_.size() * 12; // Size of node_pattern_data : 12 + 5*edges_.size() for each FoldedNodePattern
        //for (auto node_data : node_pattern_data_){
          //node_pattern_data_memory += ( 12 + 5*node_data.edges_.size() );
        //  node_pattern_data_memory += 12;
        //}

        int remapped_ids_memory = size() * 4; // RRNode Count * 4 bytes for each remapped id
        int node_to_x_y_memory = size() * (2 + 2); // RRNode Count * (2 bytes for x, 2 bytes for y)
        int node_type_memory = size() * 1; // RRNode Count * 1 byte for the t_rr_type node type
        int size_memory = 8; // 8 bytes for size_t
        return tile_patterns_memory + node_patterns_memory + node_pattern_data_memory + remapped_ids_memory + node_to_x_y_memory + node_type_memory + size_memory;
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

    /* -- Mutators -- */
  public:
    void build_folded_rr_graph();

    void initialize_folded_rr_graph();

    void add_empty_pattern();

    

    /* -- Internal data storage -- */
  private:

  
/* Every tile has its own FoldedTilePattern. starting_node_id_ refers to the first node id in the tile.
     * node_patterns_idx_ refers to the index within node_patterns_ that contains the 
     */
    struct FoldedTilePattern { // 6 Bytes
        RRNodeId starting_node_id_ = RRNodeId(-1); // 4 Bytes
        int16_t node_patterns_idx_ = -1; // 2 Bytes  -> FoldedNodePattern
      }; 
    struct FoldedEdgePattern { // 9 Bytes
        int16_t dx;
        int16_t dy;
        int16_t offset; 
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

          std::vector<FoldedEdgePattern> edges_; // 5 Bytes for each edge
      };

    /* Obtain node_pattern_data_ for specific node id */
    inline FoldedNodePattern get_node_pattern(RRNodeId node) const {
      RRNodeId remapped_id = remapped_ids_[node];
      //std::array<size_t, 2> x_y = find_tile_coords(node);
      auto tile = tile_patterns_[node_to_x_y_[node][0]][node_to_x_y_[node][1]];
      int node_patterns_idx = tile.node_patterns_idx_;
      size_t offset = (size_t) remapped_id - (size_t) tile.starting_node_id_;
      return node_pattern_data_[node_patterns_[node_patterns_idx][offset]];
    } 

    inline int16_t node_offset(RRNodeId node){
      RRNodeId remapped_id = remapped_ids_[node];
      //std::array<size_t, 2> x_y = find_tile_coords(node);
      auto tile = tile_patterns_[node_to_x_y_[node][0]][node_to_x_y_[node][1]];
      size_t offset = (size_t) remapped_id - (size_t) tile.starting_node_id_;
      return offset;
    }

    inline int16_t get_node_patterns_idx(RRNodeId node){
      auto tile = tile_patterns_[node_to_x_y_[node][0]][node_to_x_y_[node][1]];
      return tile.node_patterns_idx_;
    }

    /* Obtain node_pattern_data_ for specific node id */
    inline void set_node_edges(RRNodeId node, std::vector<FoldedEdgePattern> edges) {
      RRNodeId remapped_id = remapped_ids_[node];
      //std::array<size_t, 2> x_y = find_tile_coords(node);
      auto tile = tile_patterns_[node_to_x_y_[node][0]][node_to_x_y_[node][1]];
      int node_patterns_idx = tile.node_patterns_idx_;
      size_t offset = (size_t) remapped_id - (size_t) tile.starting_node_id_;
      node_pattern_data_[node_patterns_[node_patterns_idx][offset]].edges_ = edges;
    } 




    friend bool operator==(const FoldedNodePattern& lhs, const FoldedNodePattern& rhs)
          {
            return lhs.cost_index_ == rhs.cost_index_ &&
                   lhs.rc_index_ == rhs.rc_index_ &&
                   lhs.dx_ == rhs.dx_ &&
                   lhs.dy_ == rhs.dy_ &&
                   lhs.edges_ == rhs.edges_ &&
                   lhs.type_ == rhs.type_ &&
                   lhs.dir_side_.direction_ == rhs.dir_side_.direction_ &&
                   lhs.dir_side_.sides_ == rhs.dir_side_.sides_ &&
                   lhs.capacity_ == rhs.capacity_;

          }

    

    
    friend bool operator==(const FoldedEdgePattern& lhs, const FoldedEdgePattern& rhs)
        {
          return lhs.dx == rhs.dx &&
                  lhs.dy == rhs.dy &&
                  lhs.offset == rhs.offset;
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
                                          node_pattern_data_[idx]


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
                                          node_pattern_data_[idx]

    */



    /* 2d vector. The indexes into the vector are the tile's x and y position */
    std::vector<std::vector<FoldedTilePattern>> tile_patterns_; // Every tile has a starting_node_id and node_patterns_idx

    /* Every Tile Type has a set of FoldedNodePatterns */
    std::vector<std::vector<int16_t>> node_patterns_; // 2 bytes

    /* Raw FoldedNodePattern data is stored here */
    std::vector<FoldedNodePattern> node_pattern_data_;

    /* Due to the current gaps in RRNodeIds, this vector remaps them so that there are no gaps */
    vtr::vector<RRNodeId, RRNodeId> remapped_ids_;

    /* Map of RRNodeId to  x_low and y_low*/ 
    vtr::vector<RRNodeId, std::array<int16_t, 2>> node_to_x_y_;

    size_t size_;

    /* node-level storage including edge storages */
    const t_rr_graph_storage& node_storage_;

    bool built = false; // flag for determining if the FoldedRRGraph has been built yet

};

#endif

/*
  You need to implement the following: 
        const vtr::array_view_id<RRNodeId, const t_rr_node_ptc_data> node_ptc,
        const vtr::array_view_id<RRNodeId, const RREdgeId> node_first_edge,
        const vtr::array_view_id<RRNodeId, const t_edge_size> node_fan_in,
        const vtr::array_view_id<RREdgeId, const RRNodeId> edge_src_node,
        const vtr::array_view_id<RREdgeId, const RRNodeId> edge_dest_node,
        const vtr::array_view_id<RREdgeId, const short> edge_switch)
        : node_storage_(node_storage)
        , node_ptc_(node_ptc)
        , node_first_edge_(node_first_edge)
        , node_fan_in_(node_fan_in)
        , edge_src_node_(edge_src_node)
        , edge_dest_node_(edge_dest_node)
        , edge_switch_(edge_switch) {}
*/