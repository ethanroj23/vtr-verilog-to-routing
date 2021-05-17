#ifndef RR_GRAPH_VIEW_H
#define RR_GRAPH_VIEW_H

#include "rr_graph_storage.h"
#include "rr_spatial_lookup.h"

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
class RRGraphView {
    /* -- Constructors -- */
  public:
    /* See detailed comments about the data structures in the internal data storage section of this file */
    RRGraphView(const t_rr_graph_storage& node_storage,
                const RRSpatialLookup& node_lookup);

    /* Disable copy constructors and copy assignment operator
     * This is to avoid any duplication of the object
     * as it is only interface allowed to access routing resource graph
     */
    RRGraphView(const RRGraphView&) = delete;
    void operator=(const RRGraphView&) = delete;

    /* -- Accessors -- */
    /* TODO: The accessors may be turned into private later if they are replacable by 'questionin' 
     * kind of accessors
     */
  public:
    /* Get the type of a routing resource node */
    t_rr_type node_type(RRNodeId node) const;

    /* Return the fast look-up data structure for queries from client functions */
    const RRSpatialLookup& node_lookup() const;

    /* Get the xhigh of a routing resource node */
    short node_xhigh(RRNodeId node) const;

    /* Get the xlow of a routing resource node */
    short node_xlow(RRNodeId node) const;

    /* Get the yhigh of a routing resource node */
    short node_yhigh(RRNodeId node) const;

    /* Get the ylow of a routing resource node */
    short node_ylow(RRNodeId node) const;

    /* Get the number of edges of a routing resource node */
    t_edge_size node_num_edges(RRNodeId node) const;

    /* Get the capacity of a routing resource node */
    short node_capacity(RRNodeId node) const;

    /* Get the ptc_num of a routing resource node */
    short node_ptc_num(RRNodeId node) const;

    /* Get the direction of a routing resource node */
    e_direction node_direction(RRNodeId node) const;

    /* Get the fan_in of a routing resource node */
    t_edge_size node_fan_in(RRNodeId node) const;

    /* Get the configurable edges of a routing resource node */
    edge_idx_range node_configurable_edges(RRNodeId node) const;

    /* Get the non configurable edges of a routing resource node */
    edge_idx_range node_non_configurable_edges(RRNodeId node) const;

    /* Get the type string of a routing resource node */
    const char* node_type_string(RRNodeId node) const;

    /* Get the cost index of a routing resource node */
    short node_cost_index(RRNodeId node) const;

    /* Get the Resistance of a routing resource node */
    float node_R(RRNodeId node) const;

    /* Get Capacitance index of a routing resource node */
    float node_C(RRNodeId node) const;

    /* Validate the edges */
    bool node_validate() const;

    /* Return the number of nodes */
    size_t node_length() const;




    /* -- Internal data storage -- */
    /* Note: only read-only object or data structures are allowed!!! */
  private:
    /* node-level storage including edge storages */
    const t_rr_graph_storage& node_storage_;
    /* Fast look-up for rr nodes */
    const RRSpatialLookup& node_lookup_;
};

#endif
