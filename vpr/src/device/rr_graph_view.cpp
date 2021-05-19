#include "rr_graph_view.h"

RRGraphView::RRGraphView(const t_rr_graph_storage& node_storage,
                         const RRSpatialLookup& node_lookup)
    : node_storage_(node_storage)
    , node_lookup_(node_lookup) {
}

t_rr_type RRGraphView::node_type(RRNodeId node) const {
    return node_storage_.node_type(node);
}

const RRSpatialLookup& RRGraphView::node_lookup() const {
    return node_lookup_;
}

short RRGraphView::node_xhigh(RRNodeId node) const {
    return node_storage_.node_xhigh(node);
}

short RRGraphView::node_xlow(RRNodeId node) const {
    return node_storage_.node_xlow(node);
}

short RRGraphView::node_yhigh(RRNodeId node) const {
    return node_storage_.node_yhigh(node);
}

short RRGraphView::node_ylow(RRNodeId node) const {
    return node_storage_.node_ylow(node);
}

t_edge_size RRGraphView::node_num_edges(RRNodeId node) const {
    return node_storage_.num_edges(node);
}

short RRGraphView::node_capacity(RRNodeId node) const {
    return node_storage_.node_capacity(node);
}

short RRGraphView::node_ptc_num(RRNodeId node) const {
    return node_storage_.node_ptc_num(node);
}

e_direction RRGraphView::node_direction(RRNodeId node) const {
    return node_storage_.node_direction(node);
}

t_edge_size RRGraphView::node_fan_in(RRNodeId node) const {
    return node_storage_.fan_in(node);
}

edge_idx_range RRGraphView::node_configurable_edges(RRNodeId node) const {
    return node_storage_.configurable_edges(node);
}

edge_idx_range RRGraphView::node_non_configurable_edges(RRNodeId node) const {
    return node_storage_.non_configurable_edges(node);
}

const char* RRGraphView::node_type_string(RRNodeId node) const {
    return node_storage_.node_type_string(node);
}

short RRGraphView::node_cost_index(RRNodeId node) const {
    return node_storage_.node_cost_index(node);
}

float RRGraphView::node_R(RRNodeId node) const {
    return node_storage_.node_R(node);
}

float RRGraphView::node_C(RRNodeId node) const {
    return node_storage_.node_C(node);
}

bool RRGraphView::node_validate() const {
    return node_storage_.validate();
}

short RRGraphView::node_edge_switch(RREdgeId edge) const {
    return node_storage_.edge_switch(edge);
}

RRNodeId RRGraphView::node_edge_sink_node(RREdgeId edge) const {
    return node_storage_.edge_sink_node(edge);
}

short RRGraphView::node_track_num(RRNodeId node) const {
    return node_storage_.node_track_num(node);
}

short RRGraphView::node_length(RRNodeId node) const {
    return std::max(
        RRGraphView::node_yhigh(node) - RRGraphView::node_ylow(node),
        RRGraphView::node_xhigh(node) - RRGraphView::node_xlow(node));
}

const char* RRGraphView::node_direction_string(RRNodeId node) const {
    return node_storage_.node_direction_string(node);
}

short RRGraphView::node_pin_num(RRNodeId node) const {
    return node_storage_.node_pin_num(node);
}

short RRGraphView::node_class_num(RRNodeId node) const {
    return node_storage_.node_class_num(node);
}

bool RRGraphView::edge_is_configurable(RRNodeId node, RREdgeId edge) const {
    auto edge_range = RRGraphView::node_configurable_edges(node);
    for (auto current_edge : edge_range){
        if (edge == RREdgeId(current_edge)){
            return true;
        }
    }
    return false;
}
