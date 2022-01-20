# FoldedPerTileRRGraph 
This folding method represents the RRGraph in VTR in a reduced manner using edge patterns. The folding takes place inside `build_graph()` of `folded_per_tile_rr_graph.cpp` and proceeds in the following manner:
## Folding Process
1. `tile_to_node` creation (x, y, tile_idx -> `RRNodeId`)  
    a. Iterate over original RRGraph nodes and add `RRNodeId` to `tile_to_node`  
    b. Sort nodes in each tile by certain characteristics (length, type, cost_index)
2. Find node and edge patterns  
    a. Every node has a list of edges  
    b. Each edge has `dx`, `dy`, `switch_id`, and `tile_idx`
3. Store edge_patterns and index into them for each node

## Access Examples

To access a given RRNode's edges:      
      
      x_y_idx = node_coords_[node];
      folded_edges = shared_edges_[x_y_idx.node_pattern_idx];
      for (cur_edge : folded_edges){
        return_edges.push_back({
          tile_to_node_[x_y_idx.xlow + cur_edge.dx][x_y_idx.ylow + cur_edge.dy][cur_edge.tile_idx], // dest
          cur_edge.switch_id // switch
        });
      }

<br/><br/>
The following names are used in reporting results for convenience

| category |vtr1|vtr3|vtr4|vtr6|vtr5|
|---|---|---|---|---|---|
| architecture         |EArch.xml|k6_frac_N10_frac_chain_mem32K_40nm.xml|k6_frac_N10_frac_chain_mem32K_40nm.xml|stratixiv_arch.timing.xml|stratixiv_arch.timing.xml|
| circuit |tseng.blif|arm_core.pre-vpr.blif| LU32PEEng.pre-vpr.blif|cholesky_mc_stratixiv_arch_timing.pre-vpr.blif|des90_stratixiv_arch_timing.pre-vpr.blif|


<br/><br/>
## Runtime in the form original / folded
| category |vtr1|vtr3|vtr4|vtr6|vtr5|
|---|---|---|---|---|---|
| routing            | 0.25 / 0.24| 17.74 / 16.18| 181.34 / 165.37| 319.5 / 336.72| 455.59 / 453.19|
| lookahead          | 0.29 / 0.23| 5.08 / 4.07| 54.1 / 42.37| 213.43 / 190.09| 570.37 / 494.97|
| packing            | 0.81 / 0.83| 20.06 / 20.56|        | 103.24 / 102.03| 248.0 / 245.47|
| placement          | 0.47 / 0.49| 13.38 / 13.98| 0.08 / 0.08| 166.89 / 186.68| 294.14 / 327.65|
| max_rss            | 46.5 / 48.3| 262.4 / 268.4| 1745.1 / 1769.0| 2973.2 / 3397.9| 4283.0 / 5052.4|
| entire flow        | 2.2 / 2.21| 62.28 / 60.88| 277.38 / 251.01| 1232.51 / 1248.51| 2638.87 / 2628.5|
<br/><br/>
## Memory Usage in the form original / folded
| category |vtr1|vtr3|vtr4|vtr6|vtr5|
|---|---|---|---|---|---|
| nodes              | 15052 / 15052| 208464 / 208464| 1688524 / 1688524| 3364308 / 3364308| 7260478 / 7260478|
| edges              | 114130 / 105196| 1924506 / 270358| 18950277 / 400187| 36744890 / 26305066| 91152197 / 65750765|
| size (MiB)               | 1.32 / 1.15| 21.53 / 6.83| 206.49 / 41.70| 401.76 / 277.69| 980.08 / 667.82|



<br/><br/>
## Cache miss percentages in the form original / folded
| category |vtr1|vtr3|vtr4|vtr6|vtr5|
|---|---|---|---|---|---|
| cache refs         | 2.193 / 1.935| 5.676 / 5.773| 19.683 / 14.564| 22.115 / 21.479| 26.424 / 23.991|
| L1-dcache accesses | 3.26 / 3.57| 3.54 / 4.22| 1.72 / 3.07| 2.49 / 3.83| 2.64 / 4.12|
| LL-cache accesses  | 1.17 / 1.46| 2.53 / 2.51| 12.4 / 8.96| 15.04 / 14.8| 19.89 / 18.27|


<br/><br/>
## Relative difference. (Negative means folded was smaller)
| category |vtr1|vtr3|vtr4|vtr6|vtr5|
|---|---|---|---|---|---|
| cache refs         | -11.76 | 1.71   | -26.01 | -2.88  | -9.21  |
| L1-dcache accesses | 9.51   | 19.21  | 78.49  | 53.82  | 56.06  |
| LL-cache accesses  | 24.79  | -0.79  | -27.74 | -1.60  | -8.14  |
| routing            | -4.00  | -8.79  | -8.81  | 5.39   | -0.53  |
| lookahead          | -20.69 | -19.88 | -21.68 | -10.94 | -13.22 |
| packing            | 2.47   | 2.49   |        | -1.17  | -1.02  |
| placement          | 4.26   | 4.48   | 0.00   | 11.86  | 11.39  |
| max_rss            | 3.87   | 2.29   | 1.37   | 14.28  | 17.96  |
| entire flow        | 0.45   | -2.25  | -9.51  | 1.30   | -0.39  |
| nodes              | 0.00   | 0.00   | 0.00   | 0.00   | 0.00   |
| edges              | -7.83  | -85.95 | -97.89 | -28.41 | -27.87 |
| size               | -12.97 | -68.26 | -79.80 | -30.88 | -31.86 |





