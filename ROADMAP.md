# Multi-Robot Warehouse Order-Picking Simulator — Roadmap

**Language:** C++ (17 or later)
**Purpose:** DSA + LLD resume project. A pipeline that solves four nested optimization
problems, each from a different DSA family.

---

## 1. The mental model

Strip away the warehouse story and this is **one simulation pipeline solving four nested
sub-problems**:

| # | Sub-problem                | Question                                             | DSA family                          |
|---|----------------------------|------------------------------------------------------|-------------------------------------|
| 1 | Single-robot pathfinding   | How does one robot get A → B on the grid?            | Graphs (BFS / Dijkstra / A*)        |
| 2 | Multi-robot coordination   | How do N robots move at once without colliding?      | Graphs on a *space-time* grid       |
| 3 | Order → robot assignment   | Which robot takes which order?                        | Greedy → bipartite matching         |
| 4 | Item visiting order        | Best pickup sequence for one order's many items?     | DP (Held–Karp TSP)                  |

**Global objective:** minimize total time/distance across *all* robots. This is NP-hard as a
whole — the system is a pipeline that uses **exact** algorithms on tractable sub-problems
(per-order TSP via DP) and **principled heuristics** on the intractable coordination layer
(prioritized space-time planning).

> Resume framing: *"Not one algorithm — a pipeline combining exact DP (Held–Karp TSP per
> order) with heuristic multi-agent pathfinding (prioritized space-time A*)."*

---

## 2. Architecture

```
SimulationEngine   tick loop, owns the clock
   |
   +-- Warehouse    grid + shelves + packing station (the graph)
   +-- Robot        id, pos, assigned order, planned path
   +-- Order        item list, priority/deadline
   +-- TaskAssigner order -> robot                          (Phase 2)
   +-- PathPlanner  A -> B path on grid, swappable interface (Phase 1)
   +-- RoutePlanner visit-order for one order's items (TSP)  (Phase 3)
   +-- Scheduler    space-time reservation + collision resolution (Phase 1-2)
   +-- Metrics      records everything for before/after comparisons
```

Two modules that are easy to forget but are the backbone:

- **SimulationEngine (tick loop).** Discrete-time clock. Each tick, every robot advances one
  cell along its reserved path; the Scheduler validates no collisions. This loop is the spine
  everything plugs into.
- **Metrics collector.** Build this in **Phase 1**. Tracks makespan, total distance, per-robot
  utilization, average order completion time, throughput. Without day-one measurement you
  cannot make the "exact DP cut picking time by X%" resume claim.

---

## 3. Phased roadmap (mapped to syllabus)

### Phase 1 — Foundation  *(needs only Graphs — start now)*
- [ ] Grid `Warehouse`: free cells, shelves/obstacles (blocked), packing station.
- [ ] `PathPlanner` interface + BFS implementation (shortest path in steps).
- [ ] `SimulationEngine` tick loop (advance clock, move robots one cell/tick).
- [ ] ASCII visualization of the grid per tick.
- [ ] `Metrics` collector (makespan, distance, utilization).
- [ ] `Scheduler` with **space-time reservation table**: nodes are `(x, y, t)`. Robots reserve
      `(cell, t)` pairs; others plan around them. This is **Cooperative A*** (Silver, 2005).
- [ ] Handle **both** collision types: **vertex** (same cell at tick t) and **edge/swap**
      (two robots trade places between t and t+1).

**Deliverable:** several robots crossing the grid to fixed targets, zero collisions, metrics printed.
**Interview line:** *"Multi-agent pathfinding via prioritized planning on a space-time graph."*

### Phase 2 — Assignment  *(after Greedy)*
- [ ] `TaskAssigner`: greedy nearest-available-robot <-> nearest-order (~30-40 lines).
- [ ] (Stretch) swap greedy for the **Hungarian algorithm** (optimal assignment) — a second
      "heuristic -> optimal" story that parallels the TSP one.

**Interview line:** *"Greedy assignment, with an optimal bipartite-matching upgrade path."*

### Phase 3 — Algorithmic centerpiece  *(after DP)*
- [ ] `RoutePlanner`: for one order with k scattered items, find the optimal pickup sequence
      (start -> all items -> packing station).
- [ ] **Held–Karp TSP**: bitmask DP over subsets. Time `O(2^k · k^2)`, space `O(2^k · k)`.
      Exact for k <= ~12-15.
- [ ] **Composition insight (LLD gold):** the TSP needs a distance matrix between pickup
      points — those distances come from **Phase 1's BFS** grid shortest paths. Phase 3 *calls*
      Phase 1. Clean module composition is exactly what the LLD round rewards.
- [ ] Keep the Phase-2 greedy nearest-neighbor route as a baseline; run both, let `Metrics`
      compute the % improvement -> your headline resume number.

**Interview line:** *"Replaced greedy nearest-neighbor with exact Held–Karp DP, cutting
per-order picking distance by X%."*

### Phase 4 — OOP / LLD polish  *(after OOP)*
- [ ] Refactor into clean classes; `PathPlanner` as an interface (abstract base class) so
      BFS / A* / Cooperative A* swap with zero changes elsewhere. Same for `TaskAssigner`
      (greedy vs. Hungarian) and `RoutePlanner` (greedy vs. Held–Karp). **Strategy pattern**
      throughout.

**Interview line:** *"Strategy pattern lets me hot-swap planning algorithms; this is where
I'd extend for real-time replanning."*

### Stretch goals (standout)
- [ ] A* with a congestion-cost heuristic.
- [ ] **Conflict-Based Search (CBS)** — modern gold-standard MAPF solver.
- [ ] Dynamic order arrivals (orders appear mid-simulation).
- [ ] Real GUI (SFML / SDL) instead of ASCII.

---

## 4. The one dependency rule

Build strictly in order. **Phase 3's TSP is worthless without Phase 1's pathfinder** (it needs
grid distances), and everything needs the Phase 1 tick loop + metrics. Do not skip ahead.

---

## 5. Suggested C++ project layout

```
warehouse-sim/
  CMakeLists.txt
  include/
    Warehouse.hpp      Robot.hpp      Order.hpp
    PathPlanner.hpp    (abstract) + BfsPlanner.hpp, AStarPlanner.hpp
    Scheduler.hpp      TaskAssigner.hpp   RoutePlanner.hpp
    SimulationEngine.hpp   Metrics.hpp   Types.hpp (Cell, Position, tick_t)
  src/
    *.cpp
  tests/
    (per-module unit tests)
  main.cpp             (wires up a scenario and runs the sim)
  README.md            (the resume-facing write-up + the X% numbers)
```

---

## 6. Progress log

- 2026-07-19: Roadmap created. Language chosen: C++. Nothing built yet.
