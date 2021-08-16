#include "rr_graph_view.h"

RRGraphView::RRGraphView(const t_rr_graph_storage& node_storage,
                         const RRSpatialLookup& node_lookup,
                         RRGraphViewInterface* primary_rr_graph)
    : node_storage_(node_storage)
    , node_lookup_(node_lookup){
    primary_rr_graph_ = primary_rr_graph;
}

