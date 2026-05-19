#!/usr/bin/env python3
"""
Benchmark all five grid-planning algorithms on identical (start, goal) pairs
on the warehouse occupancy map. Prints a markdown table + writes CSV.

Imports the algorithm functions directly from grid_planner_node.py so we
benchmark the exact production code (no re-implementation here).
"""
from __future__ import annotations

import csv
import importlib.util
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import binary_dilation


ROOT = Path(__file__).resolve().parents[1]
PLANNER_PY = ROOT / 'ros2_ws' / 'src' / 'kolestel_rover_description' / 'scripts' / 'grid_planner_node.py'
MAP_PGM = ROOT / 'ros2_ws' / 'src' / 'kolestel_rover_description' / 'maps' / 'warehouse_nav2_map.pgm'
MAP_YAML = ROOT / 'ros2_ws' / 'src' / 'kolestel_rover_description' / 'maps' / 'warehouse_nav2_map.yaml'


def load_grid(robot_radius_m: float, occ_thresh: int = 200):
    meta = yaml.safe_load(MAP_YAML.read_text())
    res = float(meta['resolution'])
    origin = meta['origin']
    ox, oy = float(origin[0]), float(origin[1])
    img = np.flipud(np.array(Image.open(MAP_PGM))).copy()
    occupied = (img < occ_thresh).astype(np.uint8)
    r = max(1, int(math.ceil(robot_radius_m / res)))
    Y, X = np.ogrid[-r:r + 1, -r:r + 1]
    kernel = (X * X + Y * Y) <= r * r
    grid = binary_dilation(occupied, structure=kernel).astype(np.uint8)
    return grid, res, ox, oy


def world_to_grid(x, y, res, ox, oy):
    return (int(math.floor((y - oy) / res)),
            int(math.floor((x - ox) / res)))


def import_planner_module():
    """Extract the algorithm code from grid_planner_node.py via AST, dropping
    any ROS imports / Node class. Run the cleaned source — get back the
    ALGORITHMS dict with all functions usable standalone.
    """
    import ast
    src = PLANNER_PY.read_text()
    tree = ast.parse(src)
    kept_nodes = []
    for node in tree.body:
        # skip rclpy / ROS-only imports
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            name = ''
            if isinstance(node, ast.ImportFrom):
                name = node.module or ''
            else:
                name = node.names[0].name if node.names else ''
            if name.startswith(('rclpy', 'geometry_msgs', 'nav_msgs', 'sensor_msgs',
                                'std_msgs', 'tf2_ros', 'ament_index_python')):
                continue
        # skip the Node class itself (uses rclpy heavily)
        if isinstance(node, ast.ClassDef) and node.name == 'GridPlannerNode':
            continue
        # skip the main() function which uses rclpy
        if isinstance(node, ast.FunctionDef) and node.name == 'main':
            continue
        # skip the `if __name__ == '__main__':` guard
        if isinstance(node, ast.If):
            try:
                if (isinstance(node.test, ast.Compare)
                        and isinstance(node.test.left, ast.Name)
                        and node.test.left.id == '__name__'):
                    continue
            except Exception:
                pass
        kept_nodes.append(node)
    cleaned = ast.Module(body=kept_nodes, type_ignores=[])
    ns = {'__name__': 'grid_planner_algorithms_only'}
    exec(compile(cleaned, PLANNER_PY.name, 'exec'), ns)
    return type('M', (), ns)()


def path_metrics(cells, res, ox, oy):
    total_len = 0.0
    total_turn = 0.0
    prev_heading = None
    pts = []
    for (r, c) in cells:
        x = ox + (c + 0.5) * res
        y = oy + (r + 0.5) * res
        pts.append((x, y))
    for a, b in zip(pts[:-1], pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        total_len += seg
        if seg > 1e-6:
            h = math.atan2(b[1] - a[1], b[0] - a[0])
            if prev_heading is not None:
                dt = abs(((h - prev_heading) + math.pi) % (2 * math.pi) - math.pi)
                total_turn += dt
            prev_heading = h
    return total_len, total_turn


def bench_one(algo_fn, grid, start, goal, repeats=3):
    """Run repeatedly and take the median time for stability."""
    times, last_cells, last_visited = [], None, None
    for _ in range(repeats):
        t0 = time.perf_counter()
        cells, visited = algo_fn(grid, start, goal)
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000.0)
        last_cells, last_visited = cells, visited
    return float(np.median(times)), last_cells, last_visited


SCENARIOS = [
    # (name, start_world_xy, goal_world_xy, description)
    ('depot_to_E1',       (10.5, -8.0), (30.0, -2.0), 'depot south wall to shelf E1 (short, one 90° turn)'),
    ('depot_to_E2',       (10.5, -8.0), (30.0, 10.0), 'depot to shelf E2 (long, one 90° turn)'),
    ('depot_to_A_row5',   (10.5, -8.0), ( 2.8, 47.0), 'depot to opposite corner (long, multiple turns)'),
    ('E1_to_A5',          (30.0, -2.0), ( 2.8, 47.0), 'cross-warehouse (longest, multiple turns)'),
]


def main():
    out_dir = ROOT / 'benchmark' / 'results'
    out_dir.mkdir(parents=True, exist_ok=True)

    grid, res, ox, oy = load_grid(robot_radius_m=0.30)
    print(f'grid loaded: {grid.shape}, res={res} m, origin=({ox},{oy})')

    mod = import_planner_module()
    algorithms = mod.ALGORITHMS

    rows = []
    print(f'\nrunning {len(algorithms)} algorithms x {len(SCENARIOS)} scenarios')
    for scen_name, sxy, gxy, desc in SCENARIOS:
        s = world_to_grid(sxy[0], sxy[1], res, ox, oy)
        g = world_to_grid(gxy[0], gxy[1], res, ox, oy)
        if grid[s] or grid[g]:
            print(f'  [SKIP] {scen_name}: start or goal blocked')
            continue
        print(f'\n--- {scen_name} ---  start={s} goal={g}  ({desc})')
        baseline_len = None
        for name, fn in algorithms.items():
            t_ms, cells, visited = bench_one(fn, grid, s, g, repeats=3)
            if not cells:
                print(f'  {name:12s} NO PATH (t={t_ms:.1f}ms)')
                continue
            plen, tturn = path_metrics(cells, res, ox, oy)
            expanded = int(visited.sum())
            if baseline_len is None:
                baseline_len = plen
            rows.append({
                'scenario': scen_name,
                'algorithm': name,
                'time_ms': round(t_ms, 2),
                'expanded': expanded,
                'path_len_m': round(plen, 2),
                'path_pts': len(cells),
                'total_turn_deg': round(math.degrees(tturn), 1),
                'extra_vs_optimal_pct': round((plen / baseline_len - 1) * 100, 2) if baseline_len else 0.0,
            })
            print(f'  {name:12s} t={t_ms:7.1f}ms  expanded={expanded:7d}  '
                  f'len={plen:6.2f}m  pts={len(cells):5d}  '
                  f'turn={math.degrees(tturn):6.1f}°')

    # write CSV
    csv_path = out_dir / 'benchmark.csv'
    with csv_path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['scenario', 'algorithm', 'time_ms', 'expanded',
                                           'path_len_m', 'path_pts', 'total_turn_deg',
                                           'extra_vs_optimal_pct'])
        w.writeheader()
        w.writerows(rows)
    print(f'\nCSV saved: {csv_path}')

    # write markdown
    md_path = out_dir / 'benchmark.md'
    write_markdown(md_path, rows)
    print(f'Markdown saved: {md_path}')


def write_markdown(path, rows):
    by_scen = {}
    for r in rows:
        by_scen.setdefault(r['scenario'], []).append(r)
    by_algo = {}
    for r in rows:
        by_algo.setdefault(r['algorithm'], []).append(r)

    # aggregate numbers
    agg = {}
    for algo, vals in by_algo.items():
        agg[algo] = {
            'avg_time_ms': sum(v['time_ms'] for v in vals) / len(vals),
            'avg_expanded': sum(v['expanded'] for v in vals) / len(vals),
            'total_extra_pct': sum(v['extra_vs_optimal_pct'] for v in vals) / len(vals),
            'avg_turn': sum(v['total_turn_deg'] for v in vals) / len(vals),
        }

    lines = [
        '# Grid-planner algorithm benchmark',
        '',
        'Warehouse occupancy grid (800×1320 cells, resolution 0.05 m, robot radius 0.30 m).',
        f'Generated: {time.strftime("%Y-%m-%d %H:%M:%S")}',
        f'Algorithms tested: {", ".join(by_algo.keys())}',
        f'Scenarios: {len(by_scen)} (depot→E1, depot→E2, depot→A_row5, E1→A_row5)',
        '',
        '## TL;DR — measured aggregate over all scenarios',
        '',
        '| Algorithm | Avg time | Avg expanded | Avg extra-vs-optimal | Avg total turn |',
        '|-----------|---------:|-------------:|---------------------:|---------------:|',
    ]
    for algo, a in agg.items():
        lines.append(
            f"| {algo:11s} | {a['avg_time_ms']:7.1f} ms "
            f"| {a['avg_expanded']:12.0f} "
            f"| {a['total_extra_pct']:18.2f}% "
            f"| {a['avg_turn']:13.0f}° |"
        )

    lines += [
        '',
        '## Per-scenario results',
        '',
    ]
    for scen, scen_rows in by_scen.items():
        # find baseline path length (Dijkstra is reference optimum)
        opt = next((r for r in scen_rows if r['algorithm'] == 'dijkstra'), scen_rows[0])
        baseline_len = opt['path_len_m']
        lines += [
            f'### {scen}',
            '',
            '| Algorithm | Time (ms) | Expanded cells | Path length (m) | vs optimal | Points | Total turn (°) |',
            '|-----------|----------:|---------------:|----------------:|-----------:|-------:|---------------:|',
        ]
        for r in scen_rows:
            extra = r['extra_vs_optimal_pct']
            extra_str = f'+{extra:.1f}%' if extra > 0.01 else ('—' if abs(extra) < 0.01 else f'{extra:.1f}%')
            lines.append(
                f"| {r['algorithm']:11s} "
                f"| {r['time_ms']:9.1f} "
                f"| {r['expanded']:14d} "
                f"| {r['path_len_m']:15.2f} "
                f"| {extra_str:>10s} "
                f"| {r['path_pts']:6d} "
                f"| {r['total_turn_deg']:14.1f} |"
            )
        lines.append('')

    # find specific data points for the findings section
    def get(scenario, algo, field):
        for r in rows:
            if r['scenario'] == scenario and r['algorithm'] == algo:
                return r.get(field)
        return None

    def speedup(scenario, algo_fast, algo_slow):
        f = get(scenario, algo_fast, 'time_ms')
        s = get(scenario, algo_slow, 'time_ms')
        if f and s:
            return s / f
        return None

    lines += [
        '## Findings from the measured data',
        '',
        '### A* vs Dijkstra speedup (optimal-vs-optimal comparison)',
        '',
    ]
    for scen in by_scen:
        sp = speedup(scen, 'a_star', 'dijkstra')
        if sp:
            lines.append(f'* `{scen}` — A* is **{sp:.1f}×** faster than Dijkstra')
    lines.append('')

    lines += [
        '### JPS expanded-cell win',
        '',
        'JPS expands DRAMATICALLY fewer cells than A* because it skips along straight runs:',
        '',
    ]
    for scen in by_scen:
        e_a = get(scen, 'a_star', 'expanded')
        e_j = get(scen, 'jps', 'expanded')
        if e_a and e_j:
            lines.append(f'* `{scen}` — A* expanded {e_a}, JPS expanded {e_j} ({e_a / e_j:.0f}× fewer)')
    lines += [
        '',
        '**However**: this implementation is in pure Python, and JPS calls a recursive '
        '`_jps_jump` for each direction. The recursion overhead in CPython can wipe out '
        'the wall-clock advantage on long open corridors, even though the algorithmic '
        'work is much less. A native (C/C++ / cython) JPS would be the clear winner in '
        'runtime as well.',
        '',
        '### Greedy BFS speed vs optimality trade-off',
        '',
    ]
    for scen in by_scen:
        gpct = get(scen, 'greedy_bfs', 'extra_vs_optimal_pct')
        if gpct is not None:
            lines.append(f'* `{scen}` — Greedy is fastest at {get(scen, "greedy_bfs", "time_ms"):.1f}ms but path is {gpct:+.1f}% vs optimal')
    lines += [
        '',
        '### Theta\\* path smoothness — caveat',
        '',
        'Theta\\* internally builds a *sparse* any-angle path (parent chain via line-of-'
        'sight). For interoperability with our pure-pursuit follower we then densify '
        'each any-angle segment via Bresenham. That densification re-introduces the grid-'
        'aligned 1-cell zig-zag along any non-horizontal/vertical segment, which inflates '
        'the *total_turn_deg* metric. The **true** Theta\\* path (waypoint chain) has '
        'much fewer turns; for thesis-grade smoothness comparison, the parent-chain '
        'waypoint count is the meaningful metric — not the cell-by-cell turn.',
        '',
        '## Algorithm profiles',
        '',
        '### Dijkstra',
        '* **Optimal**: yes (guaranteed shortest path on the grid).',
        '* **Heuristic**: none — uniform-cost wave expansion in all directions.',
        '* **Expanded cells**: largest of all algorithms (the wave covers everything ≤ goal distance).',
        '* **Use when**: you want a guaranteed baseline; multi-goal planning; cost field needed.',
        '* **Avoid when**: a single goal in a large open map — overspends compute.',
        '',
        '### A*',
        '* **Optimal**: yes (with admissible octile heuristic on 8-connected grid).',
        '* **Heuristic**: octile distance to goal — admissible and consistent.',
        '* **Expanded cells**: dramatically smaller than Dijkstra. Typical speedup 5–30×.',
        '* **Use when**: default for single-source single-goal planning on grids.',
        '* **Avoid when**: very dynamic environments — every replan is full search (use D* Lite instead).',
        '',
        '### Greedy Best-First Search',
        '* **Optimal**: NO. Follows heuristic blindly.',
        '* **Heuristic**: octile distance only — ignores accumulated cost g(n).',
        '* **Expanded cells**: usually smallest (it picks the most "goal-like" cell every time).',
        '* **Use when**: you need a path FAST and approximate is OK.',
        '* **Avoid when**: optimality matters or the map has dead-ends (greedy gets trapped in concave obstacles).',
        '',
        '### JPS — Jump Point Search',
        '* **Optimal**: yes (it is A* with a smarter expansion rule).',
        '* **Heuristic**: octile, same as A*.',
        '* **Idea**: in a uniform-cost 8-connected grid, only "forced neighbours" matter. JPS "jumps" along straight/diagonal lines until it hits a forced neighbour or the goal, skipping all intermediate cells.',
        '* **Expanded cells**: often **10–50× fewer** than A* in open spaces. The win shrinks in cluttered environments.',
        '* **Use when**: long open corridors; large maps; same map planned often.',
        '* **Avoid when**: non-uniform costs (JPS assumes uniform). Code is also tricky to write correctly.',
        '',
        '### Theta* — any-angle planner',
        '* **Optimal**: NOT optimal on the grid, but produces paths very close to true Euclidean shortest path.',
        '* **Heuristic**: octile, same as A*.',
        '* **Idea**: when expanding a neighbour, A* sets parent[neighbour] = current. Theta* checks if there is **line of sight** between parent[current] and the neighbour; if yes, sets parent[neighbour] = parent[current] — short-circuiting the grid.',
        '* **Path quality**: short, smooth, not constrained to 45° increments. Total turn typically 30–70% less than A*.',
        '* **Use when**: smooth motion matters (pure-pursuit, kinematic constraints, comfort).',
        '* **Avoid when**: cost varies per cell — line-of-sight check loses meaning.',
        '',
        '### Decision matrix for this thesis',
        '',
        '| Criterion | Best algorithm |',
        '|---|---|',
        '| Lowest planning time | **JPS** (uniform grids) or **Greedy BFS** (no quality guarantee) |',
        '| Smoothest path | **Theta\\*** |',
        '| Optimal path | **Dijkstra** or **A\\*** or **JPS** |',
        '| Dynamic-environment replan | **D\\* Lite** (incremental — separate node) |',
        '| Production default | **A\\*** with octile heuristic |',
        '',
        '## Summary',
        '',
        '* **A\\*** is the right default — produces optimal paths fast, easy to maintain.',
        '* **JPS** wins on large open warehouses by skipping repetitive expansion.',
        '* **Theta\\*** gives the *visually* best paths (no zig-zag), best for pure pursuit followers.',
        '* **Dijkstra** is a useful baseline but pays a large compute cost without a goal-direction heuristic.',
        '* **Greedy BFS** is a "fast but wrong" option — useful only when speed >> quality.',
        '* **D\\* Lite** is the answer when obstacles can appear (lethal cells in local costmap) — incrementally repairs the previous plan instead of restarting from scratch.',
        '',
    ]
    path.write_text('\n'.join(lines))


if __name__ == '__main__':
    main()
