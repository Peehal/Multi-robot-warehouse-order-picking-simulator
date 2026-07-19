#pragma once

#include "PathPlanner.hpp"

namespace sim {

// Cooperative A* / space-time BFS: searches over (x, y, t) states instead of
// (x, y), so the same unweighted-shortest-path algorithm naturally avoids
// both vertex collisions (two robots in the same cell at the same tick) and
// edge/swap collisions (two robots trading cells between consecutive ticks).
class BfsPlanner : public PathPlanner {
public:
    explicit BfsPlanner(tick_t horizon) : horizon_(horizon) {}

    PlanResult plan(const Warehouse& warehouse, Position start, Position goal, tick_t startTime,
                    const ReservationTable& reservations) const override;

private:
    tick_t horizon_;
};

}  // namespace sim
