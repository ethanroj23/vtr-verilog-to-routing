#ifndef RR_GRAPH_VIEW_INTERFACE_H
#define RR_GRAPH_VIEW_INTERFACE_H

#include "vpr_types.h"

class RRGraphViewInterface {
   public:

      /* All of the following functions must be implemented by any RRGraph that wants to be used by RRGraphView */

      /* Node Methods */
      virtual t_rr_type node_type(RRNodeId node) const = 0;
      virtual const char* node_type_string(RRNodeId node) const=0;
      virtual int16_t node_rc_index(RRNodeId node) const=0;
      virtual float node_C(RRNodeId node) const=0;
      virtual float node_R(RRNodeId node) const=0;

      virtual short node_xlow(RRNodeId node) const=0;
      virtual short node_ylow(RRNodeId node) const=0;
      virtual short node_xhigh(RRNodeId node) const=0;
      virtual short node_yhigh(RRNodeId node) const=0;

      virtual short node_capacity(RRNodeId node) const=0;
      virtual short node_cost_index(RRNodeId node) const=0;

      virtual Direction node_direction(RRNodeId node) const=0;
      virtual const std::string& node_direction_string(RRNodeId node) const=0;

      virtual short node_ptc_num(RRNodeId id) const=0;
      virtual short node_pin_num(RRNodeId id) const=0;
      virtual short node_track_num(RRNodeId id) const=0;
      virtual short node_class_num(RRNodeId id) const=0;

      virtual t_edge_size fan_in(RRNodeId node) const=0;
      virtual t_edge_size node_fan_in(RRNodeId node) const=0;

      virtual void prefetch_node(RRNodeId id) const=0;


      virtual bool is_node_on_specific_side(RRNodeId node, e_side side) const=0;
      virtual const char* node_side_string(RRNodeId node) const=0;

      /* Edge Methods */
      virtual edge_idx_range edges(const RRNodeId& node) const=0;
      virtual void for_each_edge(std::function<void(RREdgeId, RRNodeId, RRNodeId)> apply) const=0;

      virtual edge_idx_range configurable_edges(const RRNodeId& node) const=0;
      virtual edge_idx_range non_configurable_edges(const RRNodeId& node) const=0;
      virtual t_edge_size num_edges(const RRNodeId& node) const=0;
      virtual t_edge_size num_configurable_edges(const RRNodeId& node) const=0;
      virtual t_edge_size num_non_configurable_edges(const RRNodeId& node) const=0;

      virtual RREdgeId first_edge(const RRNodeId& node) const=0;
      virtual RREdgeId last_edge(const RRNodeId& node) const=0;

      virtual vtr::StrongIdRange<RREdgeId> edge_range(const RRNodeId node) const=0;

      virtual RRNodeId edge_sink_node(const RREdgeId& edge) const=0;
      virtual RRNodeId edge_sink_node(const RRNodeId& node, t_edge_size iedge) const=0;

      virtual short edge_switch(const RREdgeId& edge) const=0;
      virtual short edge_switch(const RRNodeId& node, t_edge_size iedge) const=0;

      virtual RREdgeId edge_id(const RRNodeId& node, t_edge_size iedge) const=0;

      /* Other Methods */
      virtual const char* rr_graph_name() const=0;
      virtual size_t size() const=0;
      virtual size_t edge_count() const=0;
      virtual bool empty() const=0;
      virtual int memory_used() const=0;


      /* destructor */
      virtual ~RRGraphViewInterface() {}
};


#endif
