#ifndef RR_GRAPH_VIEW_INTERFACE_H
#define RR_GRAPH_VIEW_INTERFACE_H

#include "vpr_types.h"

class RRGraphViewInterface {
   public:

      /* All of the following functions must be implemented by any RRGraph that wants to be used by RRGraphView */

      virtual t_rr_type node_type(RRNodeId node) const = 0;
      virtual const char* node_type_string(RRNodeId node) const=0;
      virtual short node_capacity(RRNodeId node) const=0;
      virtual Direction node_direction(RRNodeId node) const=0;
      virtual const std::string& node_direction_string(RRNodeId node) const=0;
      virtual float node_C(RRNodeId node) const=0;
      virtual float node_R(RRNodeId node) const=0;
      virtual int16_t node_rc_index(RRNodeId node) const=0;
      //virtual t_edge_size node_fan_in(RRNodeId node) const;
      virtual short node_xlow(RRNodeId node) const=0;
      virtual short node_xhigh(RRNodeId node) const=0;
      virtual short node_ylow(RRNodeId node) const=0;
      virtual short node_yhigh(RRNodeId node) const=0;
      virtual short node_cost_index(RRNodeId node) const=0;
      virtual bool is_node_on_specific_side(RRNodeId node, e_side side) const=0;
      virtual const char* node_side_string(RRNodeId node) const=0;
      virtual const char* rr_graph_name() const=0;
      virtual size_t size() const=0;
      virtual bool empty() const=0;
      virtual int memory_used() const=0;

      /* destructor */
      virtual ~RRGraphViewInterface() {}
};


#endif
