#pragma once

#include <vector>

#include "ReservationTable.hpp"
#include "Types.hpp"
#include "Warehouse.hpp"

namespace sim {

// waypoints[i] is the robot's position at tick (startTime + i).
struct PlannedPath {
    std::vector<Position> waypoints;
    tick_t startTime = 0;
};

// A no-path-found result is distinguished from a real path by `found`, since
// this codebase's compiler predates std::optional's library support.
struct PlanResult {
    bool found = false;
    PlannedPath path;
};

class PathPlanner {
public:
    virtual ~PathPlanner() = default;

    virtual PlanResult plan(const Warehouse& warehouse, Position start, Position goal,
                            tick_t startTime, const ReservationTable& reservations) const = 0;
};

}  // namespace sim
