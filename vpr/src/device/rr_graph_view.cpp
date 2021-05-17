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