#ifndef RR_GRAPH_VIEW_H
#define RR_GRAPH_VIEW_H

#include "rr_graph_storage.h"
#include "rr_spatial_lookup.h"
#include "rr_graph_view_interface.h"

/* An read-only routing resource graph
 * which is an unified object including pointors to
 * - node storage
 * - TODO: edge_storage
 * - TODO: node_ptc_storage
 * - TODO: node_fan_in_storage
 * - rr_node_indices
 *
 * Note that the RRGraphView does not own the storage
 * It serves a virtual read-only protocol for
 * - placer
 * - router
 * - timing analyzer
 * - GUI
 *
 * Note that each client of rr_graph may get a frame view of the object
 * The RRGraphView is the complete frame view of the routing resource graph
 * - This helps to reduce the memory footprint for each client
 * - This avoids massive changes for each client on using the APIs
 *   as each frame view provides adhoc APIs for each client
 *
 * TODO: more compact frame views will be created, e.g., 
 * - a mini frame view: contains only node and edges, representing the connectivity of the graph
 * - a geometry frame view: an extended mini frame view with node-level attributes, 
 *                          in particular geometry information (type, x, y etc).
 *
 */

class RRGraphView : public RRGraphViewInterface {
    /* -- Constructors -- */
  public:
    /* See detailed comments about the data structures in the internal data storage section of this file */
    RRGraphView(const t_rr_graph_storage& node_storage,
                const RRSpatialLookup& node_lookup,
                RRGraphViewInterface* primary_rr_graph);

    /* Disable copy constructors and copy assignment operator
     * This is to avoid accidental copy because it could be an expensive operation considering that the 
     * memory footprint of the data structure could ~ Gb
     * Using the following syntax, we prohibit accidental 'pass-by-value' which can be immediately caught 
     * by compiler
     */
    RRGraphView(const RRGraphView&) = delete;
    void operator=(const RRGraphView&) = delete;

    /* -- Accessors -- */
    /* TODO: The accessors may be turned into private later if they are replacable by 'questionin' 
     * kind of accessors
     */
  public:
    /* ************************************************************* */
    /*                       NODE METHODS                            */
    /* ************************************************************* */
    
    /* Get the type of a routing resource node. This function is inlined for runtime optimization. */
    inline t_rr_type node_type(RRNodeId node) const {
        return primary_rr_graph_->node_type(node);
    }

    /* Get the type of a routing resource node. This function is inlined for runtime optimization. */
    inline const char* node_type_string(RRNodeId node) const {
        return primary_rr_graph_->node_type_string(node);
    }

    /* Get the capacity of a routing resource node. This function is inlined for runtime optimization. */
    inline short node_capacity(RRNodeId node) const {
        return primary_rr_graph_->node_capacity(node);
    }

    /* Get the direction of a routing resource node. This function is inlined for runtime optimization.
     * Direction::INC: wire driver is positioned at the low-coordinate end of the wire.
     * Direction::DEC: wire_driver is positioned at the high-coordinate end of the wire.
     * Direction::BIDIR: wire has multiple drivers, so signals can travel either way along the wire
     * Direction::NONE: node does not have a direction, such as IPIN/OPIN
     */
    inline Direction node_direction(RRNodeId node) const {
        return primary_rr_graph_->node_direction(node);
    }

    /* Get the direction string of a routing resource node. This function is inlined for runtime optimization. */
    inline const std::string& node_direction_string(RRNodeId node) const {
        return primary_rr_graph_->node_direction_string(node);
    }

    /* Get the capacitance of a routing resource node. This function is inlined for runtime optimization. */
    inline float node_C(RRNodeId node) const {
        return primary_rr_graph_->node_C(node);
    }

    /* Get the resistance of a routing resource node. This function is inlined for runtime optimization. */
    inline float node_R(RRNodeId node) const {
        return primary_rr_graph_->node_R(node);
    }

    /* Get the rc_index of a routing resource node. This function is inlined for runtime optimization. */
    inline int16_t node_rc_index(RRNodeId node) const {
        return primary_rr_graph_->node_rc_index(node);
    }

    /* Get the fan in of a routing resource node. This function is inlined for runtime optimization. */
    inline t_edge_size node_fan_in(RRNodeId node) const {
        return primary_rr_graph_->node_fan_in(node);
    }

    /* Get the fan in of a routing resource node. This function is inlined for runtime optimization. */
    inline t_edge_size fan_in(RRNodeId node) const {
        return primary_rr_graph_->node_fan_in(node);
    }

    /* Get the xlow of a routing resource node. This function is inlined for runtime optimization. */
    inline short node_xlow(RRNodeId node) const {
        return primary_rr_graph_->node_xlow(node);
    }

    /* Get the xhigh of a routing resource node. This function is inlined for runtime optimization. */
    inline short node_xhigh(RRNodeId node) const {
        return primary_rr_graph_->node_xhigh(node);
    }

    /* Get the ylow of a routing resource node. This function is inlined for runtime optimization. */
    inline short node_ylow(RRNodeId node) const {
        return primary_rr_graph_->node_ylow(node);
    }

    /* Get the yhigh of a routing resource node. This function is inlined for runtime optimization. */
    inline short node_yhigh(RRNodeId node) const {
        return primary_rr_graph_->node_yhigh(node);
    }

    /* Get the cost index of a routing resource node. This function is inlined for runtime optimization. */
    inline short node_cost_index(RRNodeId node) const {
        return primary_rr_graph_->node_cost_index(node);
    }


    /* Check whether a routing node is on a specific side. This function is inlined for runtime optimization. */
    inline bool is_node_on_specific_side(RRNodeId node, e_side side) const {
        return primary_rr_graph_->is_node_on_specific_side(node, side);
    }

    /* Check whether a routing node is on a specific side. This function is inlined for runtime optimization. */
    inline const char* node_side_string(RRNodeId node) const {
        return primary_rr_graph_->node_side_string(node);
    }

    inline short node_length(RRNodeId node) const{
        return std::max(
            node_yhigh(node) - node_ylow(node),
            node_xhigh(node) - node_xlow(node));
    }

    /* ************************************************************* */
    /*                   RR GRAPH METHODS                            */
    /* ************************************************************* */

    inline void set_primary_rr_graph(RRGraphViewInterface* new_rr_graph){
        primary_rr_graph_ = new_rr_graph;
    }

    inline const char* rr_graph_name() const{
        return primary_rr_graph_->rr_graph_name();
    }

    inline void print_rr_graph_name(){
        VTR_LOG("Current RRGraph in use: %s\n", rr_graph_name() );
    }

    inline size_t size() const{
        return primary_rr_graph_->size();
    }

    inline size_t edge_count() const{
        return primary_rr_graph_->edge_count();
    }

    inline bool empty() const{
        return primary_rr_graph_->empty();
    }

    inline int memory_used() const{
        return primary_rr_graph_->memory_used();
    }

    inline void print_memory_used() const {
        VTR_LOG("RRGraph[%s] is using %f MiB of memory'\n", rr_graph_name(), memory_used() / 1024 / 1024.0 );
        VTR_LOG("RRGraph[rr_graph_storage] would use %f MiB of memory'\n", (size()*16+edge_count()*10) / 1024 / 1024.0 );
    }
    /* ***************************************************************** */
    /* THE FOLLOWING FUNCTIONS HAVE NOT BEEN REPLACED THROUGHOUT VTR YET */
    /* ***************************************************************** */

    /* get PTC num. This function is inlined for runtime optimization. */
    inline short node_ptc_num(RRNodeId node) const {
        return primary_rr_graph_->node_ptc_num(node);
    }

    /* Same as ptc_num() but checks that type() is consistent. This function is inlined for runtime optimization. */
    inline short node_pin_num(RRNodeId node) const {
        return primary_rr_graph_->node_pin_num(node);
    }

    /* Same as ptc_num() but checks that type() is consistent. This function is inlined for runtime optimization. */
    inline short node_track_num(RRNodeId node) const {
        return primary_rr_graph_->node_track_num(node);
    }

    /* Same as ptc_num() but checks that type() is consistent. This function is inlined for runtime optimization. */
    inline short node_class_num(RRNodeId node) const {
        return primary_rr_graph_->node_class_num(node);
    }

    // This prefetechs hot RR node data required for optimization.
    // Note: This is optional, but may lower time spent on memory stalls in some circumstances.
    inline void prefetch_node(RRNodeId node) const {
        return primary_rr_graph_->prefetch_node(node);
    }

    /* ************************************************************* */
    /*                       EDGE METHODS                            */
    /* ************************************************************* */

     /* Edge accessors
     *
     * Preferred access methods:
     * - first_edge(RRNodeId)
     * - last_edge(RRNodeId)
     * - edge_range(RRNodeId)
     * - edge_sink_node(RREdgeId)
     * - edge_switch(RREdgeId)
     *
     * Legacy/deprecated access methods:
     * - edges(RRNodeId)
     * - configurable_edges(RRNodeId)
     * - non_configurable_edges(RRNodeId)
     * - num_edges(RRNodeId)
     * - num_configurable_edges(RRNodeId)
     * - num_non_configurable_edges(RRNodeId)
     * - edge_id(RRNodeId, t_edge_size)
     * - edge_sink_node(RRNodeId, t_edge_size)
     * - edge_switch(RRNodeId, t_edge_size)
     *
     * Only call these methods after partition_edges has been invoked. */

    inline edge_idx_range edges(const RRNodeId& node) const {
        return primary_rr_graph_->edges(node);
    }
    /* Edges are configurable if they have a switch that is configurable vtr/libs/libarchfpga/src/physical_types.cpp:83 */
    inline edge_idx_range configurable_edges(const RRNodeId& node) const {
        return primary_rr_graph_->configurable_edges(node);
    }
    inline edge_idx_range non_configurable_edges(const RRNodeId& node) const {
        return primary_rr_graph_->non_configurable_edges(node);
    }

    inline t_edge_size num_edges(const RRNodeId& node) const {
        return primary_rr_graph_->num_edges(node);
    }
    inline t_edge_size num_configurable_edges(const RRNodeId& node) const{
        return primary_rr_graph_->num_configurable_edges(node);
    }
    inline t_edge_size num_non_configurable_edges(const RRNodeId& node) const {
        return primary_rr_graph_->num_non_configurable_edges(node);
    }

    // Get the first and last RREdgeId for the specified RRNodeId.
    //
    // The edges belonging to RRNodeId is [first_edge, last_edge), excluding
    // last_edge.
    //
    // If first_edge == last_edge, then a RRNodeId has no edges.
    inline RREdgeId first_edge(const RRNodeId& node) const {
        return primary_rr_graph_->first_edge(node);
    }

    // Return the first_edge of the next rr_node, which is one past the edge
    // id range for the node we care about.
    //
    // This implies we have one dummy rr_node at the end of first_edge_, and
    // we always allocate that dummy node. We also assume that the edges have
    // been sorted by rr_node, which is true after partition_edges().
    inline RREdgeId last_edge(const RRNodeId& node) const {
        return primary_rr_graph_->last_edge(node);
    }

    // Returns a range of RREdgeId's belonging to RRNodeId id.
    //
    // If this range is empty, then RRNodeId id has no edges.
    inline vtr::StrongIdRange<RREdgeId> edge_range(const RRNodeId node) const {
        //return primary_rr_graph_->edge_range(node);
        return primary_rr_graph_->edge_range(node);
    }

    inline std::vector<t_dest_switch> edge_range_direct(RRNodeId node) const {
        return primary_rr_graph_->edge_range_direct(node);
    }

    inline std::vector<t_edge_with_id> edge_range_with_id_direct(RRNodeId node) const {
        return primary_rr_graph_->edge_range_with_id_direct(node);
    }

    inline std::vector<t_edge_struct> non_configurable_edge_range_direct(RRNodeId node) const {
        return primary_rr_graph_->non_configurable_edge_range_direct(node);
    }

    inline std::vector<t_edge_with_id> non_configurable_edge_with_id_range_direct(RRNodeId node) const {
        return primary_rr_graph_->non_configurable_edge_with_id_range_direct(node);
    }

    inline std::vector<RRNodeId> edge_range_dest_direct(RRNodeId node) const {
        return primary_rr_graph_->edge_range_dest_direct(node);
    }
    
    inline bool directconnect_exists(RRNodeId src_rr_node, RRNodeId dest_rr_node) const {
        return primary_rr_graph_->directconnect_exists(src_rr_node, dest_rr_node);
    }

    inline short edge_switch_in_node(RRNodeId node, const RREdgeId& edge_id) const {
        return primary_rr_graph_->edge_switch_in_node(node, edge_id);
    }

    inline RRNodeId edge_sink_node_in_node(RRNodeId node, const RREdgeId& edge_id) const {
        return primary_rr_graph_->edge_sink_node_in_node(node, edge_id);
    }

    // Retrieve the RREdgeId for iedge'th edge in RRNodeId.
    //
    // This method should generally not be used, and instead first_edge and
    // last_edge should be used.
    inline RREdgeId edge_id(const RRNodeId& node, t_edge_size iedge) const {
        return primary_rr_graph_->edge_id(node, iedge);
    }

    // Get the destination node for the specified edge.
    inline RRNodeId edge_sink_node(const RREdgeId& edge) const {
        return primary_rr_graph_->edge_sink_node(edge);
    }

    // Call the `apply` function with the edge id, source, and sink nodes of every edge.
    inline void for_each_edge(std::function<void(RREdgeId, RRNodeId, RRNodeId)> apply) const {
        primary_rr_graph_->for_each_edge(apply);
    }

    // Call the `apply` function with the edge id, source, and sink nodes of every edge.
    inline void for_each_edge_direct(std::function<void(RREdgeId, RRNodeId, RRNodeId, short)> apply) const {
        primary_rr_graph_->for_each_edge_direct(apply);
    }

    // Call the `apply` function with the edge id, source, and sink nodes of every edge.
    inline void for_each_edge_sink_direct(std::function<void(RREdgeId, RRNodeId)> apply) const {
        primary_rr_graph_->for_each_edge_sink_direct(apply);
    }




    // Call the `apply` function with the edge id, source, and sink nodes of every edge.
    inline void for_each_edge_no_src(std::function<void(RREdgeId, RRNodeId)> apply) const {
        primary_rr_graph_->for_each_edge_no_src(apply);
    }

    // Get the destination node for the iedge'th edge from specified RRNodeId.
    //
    // This method should generally not be used, and instead first_edge and
    // last_edge should be used.
    inline RRNodeId edge_sink_node(const RRNodeId& node, t_edge_size iedge) const {
        return primary_rr_graph_->edge_sink_node(node, iedge);
    }

    // Get the switch used for the specified edge.
    inline short edge_switch(const RREdgeId& edge) const {
        return primary_rr_graph_->edge_switch(edge);
    }

    // Get the switch used for the iedge'th edge from specified RRNodeId.
    //
    // This method should generally not be used, and instead first_edge and
    // last_edge should be used.
    inline short edge_switch(const RRNodeId& node, t_edge_size iedge) const {
        return primary_rr_graph_->edge_switch(node, iedge);
    }


/* Return the fast look-up data structure for queries from client functions */
    const RRSpatialLookup& node_lookup() const {
        return node_lookup_;
    }







    /* -- Internal data storage -- */
    /* Note: only read-only object or data structures are allowed!!! */
  private:
    /* node-level storage including edge storages */
    const t_rr_graph_storage& node_storage_;
    /* Fast look-up for rr nodes */
    const RRSpatialLookup& node_lookup_;
    /* Folded look-up for rr nodes */
    RRGraphViewInterface* primary_rr_graph_;
};

#endif

