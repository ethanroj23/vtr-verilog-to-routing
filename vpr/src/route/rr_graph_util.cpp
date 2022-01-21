#include <queue>
#include <random>
#include <algorithm>

#include "vtr_memory.h"
#include "vtr_time.h"

#include "vpr_types.h"
#include "vpr_error.h"

#include "globals.h"
#include "rr_graph_util.h"

int seg_index_of_cblock(t_rr_type from_rr_type, int to_node) {
    /* Returns the segment number (distance along the channel) of the connection *
     * box from from_rr_type (CHANX or CHANY) to to_node (IPIN).                 */

    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    if (from_rr_type == CHANX)
        return (rr_graph.node_xlow(RRNodeId(to_node)));
    else
        /* CHANY */
        return (rr_graph.node_ylow(RRNodeId(to_node)));
}

int seg_index_of_sblock(int from_node, int to_node) {
    /* Returns the segment number (distance along the channel) of the switch box *
     * box from from_node (CHANX or CHANY) to to_node (CHANX or CHANY).  The     *
     * switch box on the left side of a CHANX segment at (i,j) has seg_index =   *
     * i-1, while the switch box on the right side of that segment has seg_index *
     * = i.  CHANY stuff works similarly.  Hence the range of values returned is *
     * 0 to device_ctx.grid.width()-1 (if from_node is a CHANX) or 0 to device_ctx.grid.height()-1 (if from_node is a CHANY).   */

    t_rr_type from_rr_type, to_rr_type;

    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    from_rr_type = rr_graph.node_type(RRNodeId(from_node));
    to_rr_type = rr_graph.node_type(RRNodeId(to_node));

    if (from_rr_type == CHANX) {
        if (to_rr_type == CHANY) {
            return (rr_graph.node_xlow(RRNodeId(to_node)));
        } else if (to_rr_type == CHANX) {
            if (rr_graph.node_xlow(RRNodeId(to_node)) > rr_graph.node_xlow(RRNodeId(from_node))) { /* Going right */
                return (rr_graph.node_xhigh(RRNodeId(from_node)));
            } else { /* Going left */
                return (rr_graph.node_xhigh(RRNodeId(to_node)));
            }
        } else {
            VPR_FATAL_ERROR(VPR_ERROR_ROUTE,
                            "in seg_index_of_sblock: to_node %d is of type %d.\n",
                            to_node, to_rr_type);
            return OPEN; //Should not reach here once thrown
        }
    }
    /* End from_rr_type is CHANX */
    else if (from_rr_type == CHANY) {
        if (to_rr_type == CHANX) {
            return (rr_graph.node_ylow(RRNodeId(to_node)));
        } else if (to_rr_type == CHANY) {
            if (rr_graph.node_ylow(RRNodeId(to_node)) > rr_graph.node_ylow(RRNodeId(from_node))) { /* Going up */
                return (rr_graph.node_yhigh(RRNodeId(from_node)));
            } else { /* Going down */
                return (rr_graph.node_yhigh(RRNodeId(to_node)));
            }
        } else {
            VPR_FATAL_ERROR(VPR_ERROR_ROUTE,
                            "in seg_index_of_sblock: to_node %d is of type %d.\n",
                            to_node, to_rr_type);
            return OPEN; //Should not reach here once thrown
        }
    }
    /* End from_rr_type is CHANY */
    else {
        VPR_FATAL_ERROR(VPR_ERROR_ROUTE,
                        "in seg_index_of_sblock: from_node %d is of type %d.\n",
                        from_node, from_rr_type);
        return OPEN; //Should not reach here once thrown
    }
}

// Reorder RRNodeId's using one of these algorithms:
//   - DONT_REORDER: The identity reordering (does nothing.)
//   - DEGREE_BFS: Order by degree primarily, and BFS traversal order secondarily.
//   - RANDOM_SHUFFLE: Shuffle using the specified seed. Great for testing.
// The DEGREE_BFS algorithm was selected because it had the best performance of seven
// existing algorithms here: https://github.com/SymbiFlow/vtr-rrgraph-reordering-tool
// It might be worth further research, as the DEGREE_BFS algorithm is simple and
// makes some arbitrary choices, such as the starting node.
// Nonetheless, it does improve performance ~7% for the SymbiFlow Xilinx Artix 7 graph.
//
// NOTE: Re-ordering will invalidate any references to rr_graph nodes, so this
//       should generally be called before creating such references.
void reorder_rr_graph_nodes(const t_router_opts& router_opts) {
    auto& device_ctx = g_vpr_ctx.mutable_device();
    auto& graph = device_ctx.rr_nodes;
    const auto& rr_graph = device_ctx.rr_graph;
    size_t v_num = graph.size();

    if (router_opts.reorder_rr_graph_nodes_algorithm == DONT_REORDER) return;
    if (router_opts.reorder_rr_graph_nodes_threshold < 0 || v_num < (size_t)router_opts.reorder_rr_graph_nodes_threshold) return;

    vtr::ScopedStartFinishTimer timer("Reordering rr_graph nodes");

    vtr::vector<RRNodeId, RRNodeId> src_order(v_num); // new id -> old id
    size_t cur_idx = 0;
    for (RRNodeId& n : src_order) { // Initialize to [0, 1, 2 ...]
        n = RRNodeId(cur_idx++);
    }

    // This method works well. The intution is that highly connected nodes are enumerated first (together),
    // and since there will be a lot of nodes with the same degree, they are then ordered based on some
    // distance from the starting node.
    if (router_opts.reorder_rr_graph_nodes_algorithm == DEGREE_BFS) {
        vtr::vector<RRNodeId, size_t> bfs_idx(v_num);
        vtr::vector<RRNodeId, size_t> degree(v_num);
        std::queue<RRNodeId> que;

        // Compute both degree (in + out) and an index based on the BFS traversal
        cur_idx = 0;
        for (size_t i = 0; i < v_num; ++i) {
            if (bfs_idx[RRNodeId(i)]) continue;
            que.push(RRNodeId(i));
            bfs_idx[RRNodeId(i)] = cur_idx++;
            while (!que.empty()) {
                RRNodeId u = que.front();
                que.pop();
                degree[u] += rr_graph.num_edges(u);

                if( strcmp(rr_graph.rr_graph_name(), "FoldedPerTileRRGraph") == 0 ){ //ESR1
                    std::vector<t_dest_switch> edges;
                    rr_graph.edge_range_direct(u, edges);
                    for (auto edge : edges){
                        RRNodeId v; // ESR2
                        v = edge.dest;
                        degree[v]++;
                        if (bfs_idx[v]) continue;
                        bfs_idx[v] = cur_idx++;
                        que.push(v);
                    }
                }
                else{
                    for (RREdgeId edge = rr_graph.first_edge(u); edge < rr_graph.last_edge(u); edge = RREdgeId(size_t(edge) + 1)) {
                        RRNodeId v; // ESR2
                        v = rr_graph.edge_sink_node(edge);
                        degree[v]++;
                        if (bfs_idx[v]) continue;
                        bfs_idx[v] = cur_idx++;
                        que.push(v);
                    }
                }
            }
        }

        // Sort by degree primarily, and BFS order secondarily
        sort(src_order.begin(), src_order.end(),
             [&](auto a, auto b) -> bool {
                 auto deg_a = degree[a];
                 auto deg_b = degree[b];
                 return deg_a > deg_b || (deg_a == deg_b && bfs_idx[a] < bfs_idx[b]);
             });
    } else if (router_opts.reorder_rr_graph_nodes_algorithm == RANDOM_SHUFFLE) {
        std::mt19937 g(router_opts.reorder_rr_graph_nodes_seed);
        std::shuffle(src_order.begin(), src_order.end(), g);
    }
    vtr::vector<RRNodeId, RRNodeId> dest_order(v_num);
    cur_idx = 0;
    for (auto u : src_order)
        dest_order[u] = RRNodeId(cur_idx++);

    graph.reorder(dest_order, src_order);

    // update rr_node_indices, a map to optimize rr_index lookups
    for (auto& grid : device_ctx.rr_node_indices) {
        for (size_t x = 0; x < grid.dim_size(0); x++) {
            for (size_t y = 0; y < grid.dim_size(1); y++) {
                for (size_t s = 0; s < grid.dim_size(2); s++) {
                    for (auto& node : grid[x][y][s]) {
                        if (node != OPEN) {
                            node = size_t(dest_order[RRNodeId(node)]);
                        }
                    }
                }
            }
        }
    }

    device_ctx.rr_node_metadata.remap_keys([&](int node) { return size_t(dest_order[RRNodeId(node)]); });
    device_ctx.rr_edge_metadata.remap_keys([&](std::tuple<int, int, short> edge) {
        return std::make_tuple(size_t(dest_order[RRNodeId(std::get<0>(edge))]),
                               size_t(dest_order[RRNodeId(std::get<1>(edge))]),
                               std::get<2>(edge));
    });
}

vtr::vector<RRNodeId, std::vector<RREdgeId>> get_fan_in_list() {
    auto& rr_graph = g_vpr_ctx.device().rr_graph;

    vtr::vector<RRNodeId, std::vector<RREdgeId>> node_fan_in_list;

    node_fan_in_list.resize(rr_graph.size(), std::vector<RREdgeId>(0));
    node_fan_in_list.shrink_to_fit();

    //Walk the graph and increment fanin on all dwnstream nodes
    // rr_graph.for_each_edge(
    //     [&](RREdgeId edge, __attribute__((unused)) RRNodeId src, RRNodeId sink) {
    //         node_fan_in_list[sink].push_back(edge);
    //     });
    
    // rr_graph.for_each_edge_no_src(
    //     [&](RREdgeId edge, RRNodeId sink) {
    //         node_fan_in_list[sink].push_back(edge);
    //     });
    if( strcmp(rr_graph.rr_graph_name(), "FoldedPerTileRRGraph") == 0 ){ // ESRworks
        rr_graph.for_each_edge_sink_direct( // replicates the above function, but for FoldedPerTileRRGraph
            [&](RREdgeId edge, RRNodeId sink) {
                node_fan_in_list[sink].push_back(edge);
            });
    }
    else{
        rr_graph.for_each_edge(
        [&](RREdgeId edge, __attribute__((unused)) RRNodeId src, RRNodeId sink) {
            node_fan_in_list[sink].push_back(edge);
        });
    }

        

    return node_fan_in_list;
}
