#ifndef RR_GRAPH_BUILDER_H
#define RR_GRAPH_BUILDER_H

/**
 * @file 
 * @brief This file defines the RRGraphBuilder data structure which allows data modification on a routing resource graph 
 *
 * The builder does not own the storage but it serves a virtual protocol for
 *   - node_storage: store the node list 
 *   - node_lookup: store a fast look-up for the nodes
 *
 * @note
 * - This is the only data structure allowed to modify a routing resource graph
 *
 */
#include "rr_graph_storage.h"
#include "rr_spatial_lookup.h"
#include "metadata_storage.h"

class RRGraphBuilder {
    /* -- Constructors -- */
  public:
    /* See detailed comments about the data structures in the internal data storage section of this file */
    RRGraphBuilder(t_rr_graph_storage* node_storage,
                   MetadataStorage<int>* rr_node_metadata,
                   MetadataStorage<std::tuple<int, int, short>>* rr_edge_metadata);

    /* Disable copy constructors and copy assignment operator
     * This is to avoid accidental copy because it could be an expensive operation considering that the 
     * memory footprint of the data structure could ~ Gb
     * Using the following syntax, we prohibit accidental 'pass-by-value' which can be immediately caught 
     * by compiler
     */
    RRGraphBuilder(const RRGraphBuilder&) = delete;
    void operator=(const RRGraphBuilder&) = delete;

    /* -- Mutators -- */
  public:
    /** @brief Return a writable object for rr_nodes */
    t_rr_graph_storage& node_storage();
    /** @brief Return a writable object for update the fast look-up of rr_node */
    RRSpatialLookup& node_lookup();
    /** @brief Set the type of a node with a given valid id */
    inline void set_node_type(RRNodeId id, t_rr_type type) {
        node_storage_.set_node_type(id, type);
    }
    inline void set_node_type_ptn(int id, t_rr_type type) {
        node_storage_.set_node_type_ptn(id, type);
    }
    inline void set_node_dx_dy_ptn(int id, int dx, int dy) {
        node_storage_.set_node_dx_dy_ptn(id, dx, dy);
    }
    inline void set_node_x_y(RRNodeId id, int x, int y) {
        node_storage_.set_node_x_y(id, x, y);
    }


    /** @brief Set the node ptn index of a node with a given valid id */
    inline void set_node_ptn(RRNodeId id, int ptn_idx) {
        node_storage_.set_node_ptn(id, ptn_idx);
    }

    /**
     * @brief Add an existing rr_node in the node storage to the node look-up
     *
     * The node will be added to the lookup for every side it is on (for OPINs and IPINs) 
     * and for every (x,y) location at which it exists (for wires that span more than one (x,y)).
     *
     * This function requires a valid node which has already been allocated in the node storage, with
     *   - a valid node id
     *   - valid geometry information: xlow/ylow/xhigh/yhigh
     *   - a valid node type
     *   - a valid node ptc number
     *   - a valid side (applicable to OPIN and IPIN nodes only
     */
    void add_node_to_all_locs(RRNodeId node);

    /** @brief Clear all the underlying data storage */
    void clear();
    /** @brief reorder all the nodes
     * Reordering the rr-graph nodes may be helpful in
     *   - Increasing cache locality during routing
     *   - Improving compile time
     * Reorder RRNodeId's using one of these algorithms:
     *   - DEGREE_BFS: Order by degree primarily, and BFS traversal order secondarily.
     *   - RANDOM_SHUFFLE: Shuffle using the specified seed. Great for testing.
     * The DEGREE_BFS algorithm was selected because it had the best performance of seven
     * existing algorithms here: https://github.com/SymbiFlow/vtr-rrgraph-reordering-tool
     * It might be worth further research, as the DEGREE_BFS algorithm is simple and
     * makes some arbitrary choices, such as the starting node.
     * The re-ordering algorithm (DEGREE_BFS) does not speed up the router on most architectures
     * vs. using the node ordering created by the rr-graph builder in VPR, so it is off by default.
     * The other use of this algorithm is for some unit tests; by changing the order of the nodes
     * in the rr-graph before routing we check that no code depends on the rr-graph node order
     * Nonetheless, it does improve performance ~7% for the SymbiFlow Xilinx Artix 7 graph.
     *
     * NOTE: Re-ordering will invalidate any references to rr_graph nodes, so this
     *       should generally be called before creating such references.
     */
    void reorder_nodes(e_rr_node_reorder_algorithm reorder_rr_graph_nodes_algorithm,
                       int reorder_rr_graph_nodes_threshold,
                       int reorder_rr_graph_nodes_seed);

    /** @brief Set capacity of this node (number of routes that can use it). */
    inline void set_node_capacity(RRNodeId id, short new_capacity) {
        node_storage_.set_node_capacity(id, new_capacity);
    }
    inline void set_node_capacity_ptn(int id, short new_capacity) {
        node_storage_.set_node_capacity_ptn(id, new_capacity);
    }


    /** @brief Set the node coordinate */
    inline void set_node_coordinates(RRNodeId id, short x1, short y1, short x2, short y2) {
        node_storage_.set_node_coordinates(id, x1, y1, x2, y2);
    }
    inline void set_node_coordinates_ptn(int id, short x1, short y1, short x2, short y2) {
        node_storage_.set_node_coordinates_ptn(id, x1, y1, x2, y2);
    }
    inline void add_last_node_ptn(){
        node_storage_.add_last_node_ptn();
    }

    /** @brief The ptc_num carries different meanings for different node types
     * (true in VPR RRG that is currently supported, may not be true in customized RRG)
     * CHANX or CHANY: the track id in routing channels
     * OPIN or IPIN: the index of pins in the logic block data structure
     * SOURCE and SINK: the class id of a pin (indicating logic equivalence of pins) in the logic block data structure 
     * @note 
     * This API is very powerful and developers should not use it unless it is necessary,
     * e.g the node type is unknown. If the node type is known, the more specific routines, `set_node_pin_num()`,
     * `set_node_track_num()`and `set_node_class_num()`, for different types of nodes should be used.*/

    inline void set_node_ptc_num(RRNodeId id, short new_ptc_num) {
        node_storage_.set_node_ptc_num(id, new_ptc_num);
    }

    /** @brief set_node_pin_num() is designed for logic blocks, which are IPIN and OPIN nodes */
    inline void set_node_pin_num(RRNodeId id, short new_pin_num) {
        node_storage_.set_node_pin_num(id, new_pin_num);
    }

    /** @brief set_node_track_num() is designed for routing tracks, which are CHANX and CHANY nodes */
    inline void set_node_track_num(RRNodeId id, short new_track_num) {
        node_storage_.set_node_track_num(id, new_track_num);
    }

    /** @brief set_ node_class_num() is designed for routing source and sinks, which are SOURCE and SINK nodes */
    inline void set_node_class_num(RRNodeId id, short new_class_num) {
        node_storage_.set_node_class_num(id, new_class_num);
    }

    /** @brief Set the node direction; The node direction is only available of routing channel nodes, such as x-direction routing tracks (CHANX) and y-direction routing tracks (CHANY). For other nodes types, this value is not meaningful and should be set to NONE. */
    inline void set_node_direction(RRNodeId id, Direction new_direction) {
        node_storage_.set_node_direction(id, new_direction);
    }


    /** @brief Reserve the lists of edges to be memory efficient.
     * This function is mainly used to reserve memory space inside RRGraph,
     * when adding a large number of edges in order to avoid memory fragements */
    inline void reserve_edges(size_t num_edges) {
        node_storage_.reserve_edges(num_edges);
    }

    /** @brief emplace_back_edge; It add one edge. This method is efficient if reserve_edges was called with
     * the number of edges present in the graph. */
    inline void emplace_back_edge(RRNodeId src, RRNodeId dest, short edge_switch) {
        node_storage_.emplace_back_edge(src, dest, edge_switch);
    }

    /** @brief alloc_and_load_edges; It adds a batch of edges.  */
    inline void alloc_and_load_edges(const t_rr_edge_info_set* rr_edges_to_create) {
        node_storage_.alloc_and_load_edges(rr_edges_to_create);
    }

    /** @brief set_node_cost_index gets the index of cost data in the list of cost_indexed_data data structure
     * It contains the routing cost for different nodes in the RRGraph
     * when used in evaluate different routing paths
     */
    inline void set_node_cost_index(RRNodeId id, RRIndexedDataId new_cost_index) {
        node_storage_.set_node_cost_index(id, new_cost_index);
    }
    inline void set_node_cost_index_ptn(int id, RRIndexedDataId new_cost_index) {
        node_storage_.set_node_cost_index_ptn(id, new_cost_index);
    }

    /** @brief Set the rc_index of routing resource node. */
    inline void set_node_rc_index(RRNodeId id, NodeRCIndex new_rc_index) {
        node_storage_.set_node_rc_index(id, new_rc_index);
    }
    inline void set_node_rc_index_ptn(int id, NodeRCIndex new_rc_index) {
        node_storage_.set_node_rc_index_ptn(id, new_rc_index);
    }


    /** @brief Add the side where the node physically locates on a logic block.
     * Mainly applicable to IPIN and OPIN nodes.*/
    inline void add_node_side(RRNodeId id, e_side new_side) {
        node_storage_.add_node_side(id, new_side);
    }


    /** @brief It maps arch_switch_inf indicies to rr_switch_inf indicies. */
    inline void remap_rr_node_switch_indices(const t_arch_switch_fanin& switch_fanin) {
        node_storage_.remap_rr_node_switch_indices(switch_fanin);
    }

    /** @brief Marks that edge switch values are rr switch indicies*/
    inline void mark_edges_as_rr_switch_ids() {
        node_storage_.mark_edges_as_rr_switch_ids();
    }

    /** @brief Counts the number of rr switches needed based on fan in to support mux
     * size dependent switch delays. */
    inline size_t count_rr_switches(
        size_t num_arch_switches,
        t_arch_switch_inf* arch_switch_inf,
        t_arch_switch_fanin& arch_switch_fanins) {
        return node_storage_.count_rr_switches(num_arch_switches, arch_switch_inf, arch_switch_fanins);
    }

    /** @brief This function reserve storage for RR nodes. */
    inline void reserve_nodes(size_t size) {
        node_storage_.reserve(size);
    }


    /** @brief This function resize node storage to accomidate size RR nodes. */
    inline void resize_nodes(size_t size) {
        node_storage_.resize(size);
    }

    /** brief Validate that edge data is partitioned correctly
     * @note This function is used to validate the correctness of the routing resource graph in terms
     * of graph attributes. Strongly recommend to call it when you finish the building a routing resource
     * graph. If you need more advance checks, which are related to architecture features, you should
     * consider to use the check_rr_graph() function or build your own check_rr_graph() function. */
    inline bool validate() const {
        return node_storage_.validate();
    }

    /** @brief Sorts edge data such that configurable edges appears before
     *  non-configurable edges. */
    inline void partition_edges() {
        node_storage_.partition_edges();
    }

    /** @brief Init per node fan-in data.  Should only be called after all edges have
     * been allocated.
     * @note
     * This is an expensive, O(N), operation so it should be called once you
     * have a complete rr-graph and not called often. */
    inline void init_fan_in() {
        node_storage_.init_fan_in();
    }

    /* -- Internal data storage -- */
  private:
    /* TODO: When the refactoring effort finishes, 
     * the builder data structure will be the owner of the data storages. 
     * That is why the reference to storage/lookup is used here.
     * It can avoid a lot of code changes once the refactoring is finished 
     * (there is no function get data directly through the node_storage in DeviceContext).
     * If pointers are used, it may cause many codes in client functions 
     * or inside the data structures to be changed later.
     * That explains why the reference is used here temporarily
     */
    /* node-level storage including edge storages */
    t_rr_graph_storage& node_storage_;
    /* Fast look-up for rr nodes */
    RRSpatialLookup node_lookup_;

    /* Metadata is an extra data on rr-nodes and edges, respectively, that is not used by vpr
     * but simply passed through the flow so that it can be used by downstream tools.
     * The main (perhaps only) current use of this metadata is the fasm tool of symbiflow,
     * which needs extra metadata on which programming bits control which switch in order to produce a bitstream.*/

    MetadataStorage<int>& rr_node_metadata_;
    MetadataStorage<std::tuple<int, int, short>>& rr_edge_metadata_;
};

#endif
