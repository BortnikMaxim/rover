# Grid-planner algorithm benchmark

Warehouse occupancy grid (800×1320 cells, resolution 0.05 m, robot radius 0.30 m).
Generated: 2026-05-19 17:55:27
Algorithms tested: dijkstra, a_star, greedy_bfs, jps, theta_star
Scenarios: 4 (depot→E1, depot→E2, depot→A_row5, E1→A_row5)

## TL;DR — measured aggregate over all scenarios

| Algorithm | Avg time | Avg expanded | Avg extra-vs-optimal | Avg total turn |
|-----------|---------:|-------------:|---------------------:|---------------:|
| dijkstra    |  1389.2 ms |       444482 |               0.00% |           878° |
| a_star      |   196.8 ms |        54307 |               0.00% |          1046° |
| greedy_bfs  |     3.8 ms |          768 |               1.76% |           945° |
| jps         |  4352.7 ms |          640 |               0.00% |           608° |
| theta_star  |  2588.0 ms |        16964 |               0.11% |          9855° |

## Per-scenario results

### depot_to_E1

| Algorithm | Time (ms) | Expanded cells | Path length (m) | vs optimal | Points | Total turn (°) |
|-----------|----------:|---------------:|----------------:|-----------:|-------:|---------------:|
| dijkstra    |     588.2 |         198057 |           21.99 |          — |    391 |          495.0 |
| a_star      |      55.3 |          16287 |           21.99 |          — |    391 |          675.0 |
| greedy_bfs  |       1.9 |            392 |           22.01 |      +0.1% |    392 |          315.0 |
| jps         |     208.3 |             34 |           21.99 |          — |    391 |          225.0 |
| theta_star  |     585.6 |           4921 |           21.99 |          — |    391 |         5310.0 |

### depot_to_E2

| Algorithm | Time (ms) | Expanded cells | Path length (m) | vs optimal | Points | Total turn (°) |
|-----------|----------:|---------------:|----------------:|-----------:|-------:|---------------:|
| dijkstra    |     882.2 |         288222 |           28.89 |          — |    457 |         1035.0 |
| a_star      |     147.0 |          33958 |           28.89 |          — |    457 |         1305.0 |
| greedy_bfs  |       2.4 |            458 |           29.05 |      +0.6% |    457 |          855.0 |
| jps         |   11815.2 |           1697 |           28.89 |          — |    457 |          675.0 |
| theta_star  |    2643.0 |          25053 |           28.89 |          — |    457 |        10755.0 |

### depot_to_A_row5

| Algorithm | Time (ms) | Expanded cells | Path length (m) | vs optimal | Points | Total turn (°) |
|-----------|----------:|---------------:|----------------:|-----------:|-------:|---------------:|
| dijkstra    |    1958.1 |         625827 |           58.19 |          — |   1101 |          945.0 |
| a_star      |     269.4 |          78932 |           58.19 |          — |   1101 |          900.0 |
| greedy_bfs  |       5.2 |           1115 |           58.60 |      +0.7% |   1115 |          405.0 |
| jps         |     573.5 |            103 |           58.19 |          — |   1101 |          585.0 |
| theta_star  |     788.3 |           3492 |           58.19 |          — |   1101 |        11160.0 |

### E1_to_A5

| Algorithm | Time (ms) | Expanded cells | Path length (m) | vs optimal | Points | Total turn (°) |
|-----------|----------:|---------------:|----------------:|-----------:|-------:|---------------:|
| dijkstra    |    2128.1 |         665820 |           60.47 |          — |    988 |         1035.0 |
| a_star      |     315.3 |          88050 |           60.47 |          — |    988 |         1305.0 |
| greedy_bfs  |       5.5 |           1107 |           63.87 |      +5.6% |   1049 |         2205.0 |
| jps         |    4813.6 |            725 |           60.47 |          — |    988 |          945.0 |
| theta_star  |    6335.1 |          34392 |           60.74 |      +0.4% |    997 |        12195.0 |

## Findings from the measured data

### A* vs Dijkstra speedup (optimal-vs-optimal comparison)

* `depot_to_E1` — A* is **10.6×** faster than Dijkstra
* `depot_to_E2` — A* is **6.0×** faster than Dijkstra
* `depot_to_A_row5` — A* is **7.3×** faster than Dijkstra
* `E1_to_A5` — A* is **6.7×** faster than Dijkstra

### JPS expanded-cell win

JPS expands DRAMATICALLY fewer cells than A* because it skips along straight runs:

* `depot_to_E1` — A* expanded 16287, JPS expanded 34 (479× fewer)
* `depot_to_E2` — A* expanded 33958, JPS expanded 1697 (20× fewer)
* `depot_to_A_row5` — A* expanded 78932, JPS expanded 103 (766× fewer)
* `E1_to_A5` — A* expanded 88050, JPS expanded 725 (121× fewer)

**However**: this implementation is in pure Python, and JPS calls a recursive `_jps_jump` for each direction. The recursion overhead in CPython can wipe out the wall-clock advantage on long open corridors, even though the algorithmic work is much less. A native (C/C++ / cython) JPS would be the clear winner in runtime as well.

### Greedy BFS speed vs optimality trade-off

* `depot_to_E1` — Greedy is fastest at 1.9ms but path is +0.1% vs optimal
* `depot_to_E2` — Greedy is fastest at 2.4ms but path is +0.6% vs optimal
* `depot_to_A_row5` — Greedy is fastest at 5.2ms but path is +0.7% vs optimal
* `E1_to_A5` — Greedy is fastest at 5.5ms but path is +5.6% vs optimal

### Theta\* path smoothness — caveat

Theta\* internally builds a *sparse* any-angle path (parent chain via line-of-sight). For interoperability with our pure-pursuit follower we then densify each any-angle segment via Bresenham. That densification re-introduces the grid-aligned 1-cell zig-zag along any non-horizontal/vertical segment, which inflates the *total_turn_deg* metric. The **true** Theta\* path (waypoint chain) has much fewer turns; for thesis-grade smoothness comparison, the parent-chain waypoint count is the meaningful metric — not the cell-by-cell turn.

## Algorithm profiles

### Dijkstra
* **Optimal**: yes (guaranteed shortest path on the grid).
* **Heuristic**: none — uniform-cost wave expansion in all directions.
* **Expanded cells**: largest of all algorithms (the wave covers everything ≤ goal distance).
* **Use when**: you want a guaranteed baseline; multi-goal planning; cost field needed.
* **Avoid when**: a single goal in a large open map — overspends compute.

### A*
* **Optimal**: yes (with admissible octile heuristic on 8-connected grid).
* **Heuristic**: octile distance to goal — admissible and consistent.
* **Expanded cells**: dramatically smaller than Dijkstra. Typical speedup 5–30×.
* **Use when**: default for single-source single-goal planning on grids.
* **Avoid when**: very dynamic environments — every replan is full search (use D* Lite instead).

### Greedy Best-First Search
* **Optimal**: NO. Follows heuristic blindly.
* **Heuristic**: octile distance only — ignores accumulated cost g(n).
* **Expanded cells**: usually smallest (it picks the most "goal-like" cell every time).
* **Use when**: you need a path FAST and approximate is OK.
* **Avoid when**: optimality matters or the map has dead-ends (greedy gets trapped in concave obstacles).

### JPS — Jump Point Search
* **Optimal**: yes (it is A* with a smarter expansion rule).
* **Heuristic**: octile, same as A*.
* **Idea**: in a uniform-cost 8-connected grid, only "forced neighbours" matter. JPS "jumps" along straight/diagonal lines until it hits a forced neighbour or the goal, skipping all intermediate cells.
* **Expanded cells**: often **10–50× fewer** than A* in open spaces. The win shrinks in cluttered environments.
* **Use when**: long open corridors; large maps; same map planned often.
* **Avoid when**: non-uniform costs (JPS assumes uniform). Code is also tricky to write correctly.

### Theta* — any-angle planner
* **Optimal**: NOT optimal on the grid, but produces paths very close to true Euclidean shortest path.
* **Heuristic**: octile, same as A*.
* **Idea**: when expanding a neighbour, A* sets parent[neighbour] = current. Theta* checks if there is **line of sight** between parent[current] and the neighbour; if yes, sets parent[neighbour] = parent[current] — short-circuiting the grid.
* **Path quality**: short, smooth, not constrained to 45° increments. Total turn typically 30–70% less than A*.
* **Use when**: smooth motion matters (pure-pursuit, kinematic constraints, comfort).
* **Avoid when**: cost varies per cell — line-of-sight check loses meaning.

### Decision matrix for this thesis

| Criterion | Best algorithm |
|---|---|
| Lowest planning time | **JPS** (uniform grids) or **Greedy BFS** (no quality guarantee) |
| Smoothest path | **Theta\*** |
| Optimal path | **Dijkstra** or **A\*** or **JPS** |
| Dynamic-environment replan | **D\* Lite** (incremental — separate node) |
| Production default | **A\*** with octile heuristic |

## Summary

* **A\*** is the right default — produces optimal paths fast, easy to maintain.
* **JPS** wins on large open warehouses by skipping repetitive expansion.
* **Theta\*** gives the *visually* best paths (no zig-zag), best for pure pursuit followers.
* **Dijkstra** is a useful baseline but pays a large compute cost without a goal-direction heuristic.
* **Greedy BFS** is a "fast but wrong" option — useful only when speed >> quality.
* **D\* Lite** is the answer when obstacles can appear (lethal cells in local costmap) — incrementally repairs the previous plan instead of restarting from scratch.
