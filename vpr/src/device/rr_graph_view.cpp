#include "rr_graph_view.h"

RRGraphView::RRGraphView(const t_rr_graph_storage& node_storage,
                         const RRSpatialLookup& node_lookup,
                         const FoldedRRGraph& folded_rr_graph)
    : node_storage_(node_storage)
    , node_lookup_(node_lookup)
    , folded_rr_graph_(folded_rr_graph) {
}

void RRGraphView::print_stats(){
    VTR_LOG("RRGraphView Stats:\nnode_type: %d\nnode_capacity: %d\nnode_direction: %d\nnode_direction_string: %d\nnode_fan_in: %d\n",
                function_calls_.node_type, function_calls_.node_capacity, function_calls_.node_direction, function_calls_.node_direction_string,
                    function_calls_.node_fan_in);

}
