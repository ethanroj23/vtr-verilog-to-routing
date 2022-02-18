import os

# Indices into flat_graph object
NODE = 0
EDGE = 1
EDGE_COUNT = 2

XLOW = 0
YLOW = 1
XHIGH = 2
YHIGH = 3
TYPE = 4
R = 5
C = 6
CAPACITY = 7
SIDE = 8
PTC = 9
DIRECTION = 10
SEGMENT = 11

# Indices into folded_graph object
F_NODE_TO_PATTERN = 0
F_NODE_PATTERNS = 1
F_EDGES = 2

# Size constants
FLAT_NODE_BYTES = 16 # bytes per node
FLAT_EDGE_BYTES = 10 # bytes per edge

def fold_save_metrics(flat_graph, graph_name):
    folded_graph = fold(flat_graph)
    save(folded_graph, graph_name)
    metrics(flat_graph, folded_graph, graph_name)

def fold(graph):
    np = {} # node_patterns
    node_to_pattern = [] # mapping from node_id -> node pattern idx
    p_idx = 0
    for i, node in enumerate(graph[NODE]):
        cur_p = (node[XLOW], node[YLOW], node[XHIGH], node[YHIGH], node[R],
                node[C], node[CAPACITY], node[SEGMENT], node[TYPE])
        if cur_p not in np:
            np[cur_p] = p_idx
            p_idx += 1
        node_to_pattern.append(np[cur_p])
    
    folded_graph = [node_to_pattern, np, graph[EDGE]]
    return folded_graph

def save(graph, graph_name):
    save_file = f'{os.getcwd()}/folded_graphs/{name()}_{graph_name}.xml'
    # print(f'Saving graph to {save_file}')



def metrics(flat_graph, folded_graph, graph_name):
    MiB = 1024*1024
    flat_nodes_size = len(flat_graph[NODE])*FLAT_NODE_BYTES/MiB
    flat_size = (len(flat_graph[NODE])*FLAT_NODE_BYTES + flat_graph[EDGE_COUNT]*FLAT_EDGE_BYTES)/MiB
    
    np_count = len(folded_graph[F_NODE_PATTERNS])
    node_count = len(folded_graph[F_NODE_TO_PATTERN])
    edge_count = flat_graph[EDGE_COUNT]
    folded_nodes_size = (node_count*4+np_count*FLAT_NODE_BYTES)/MiB
    folded_size = (np_count*FLAT_NODE_BYTES + node_count*4 + edge_count*FLAT_EDGE_BYTES)/MiB

    print(f'\n{graph_name} metrics [{name()}]:\n\t' \
          f'Node patterns: {np_count} ({np_count*FLAT_NODE_BYTES/MiB:.2f} MiB) [{100*np_count/node_count:.1f}%]\n\t' \
          f'Node to patterns: {node_count} ({node_count*4/MiB:.2f} MiB)\n\t' \
          f'Nodes: {node_count} ({flat_nodes_size:.2f} -> {folded_nodes_size:.2f} MiB) [{100*folded_nodes_size/flat_nodes_size:.1f}%]\n\t' \
          f'Edges: {edge_count} ({edge_count*FLAT_EDGE_BYTES/MiB:.2f} MiB)\n\t'
          f'Total Size: {flat_size:.2f} -> {folded_size:.2f} MiB [{100*folded_size/flat_size:.1f}%]')



def name():
    return 'nodes_all_attr'

if __name__ == '__main__':
    print(f"This folding method is '{name()}'")
    print(f"To fold an rr_graph with this method, run the following command from the directory one level up")
    print(f"\tpython fold_rr_graph.py {name()} <flat_graph_name>")