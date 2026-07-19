#pragma once

#include <memory>

#include "PathPlanner.hpp"
#include "ReservationTable.hpp"
#include "Robot.hpp"
#include "Warehouse.hpp"

namespace sim {

// Prioritized planning: robots are planned one at a time, in call order, and
// each new plan avoids every reservation already made by earlier robots. It
// is a heuristic (not globally optimal) way to handle multi-robot
// coordination, but it turns collision avoidance into a sequence of ordinary
// space-time BFS searches.
class Scheduler {
public:
    Scheduler(const Warehouse& warehouse, std::unique_ptr<PathPlanner> planner, tick_t horizon);

    bool planRoute(Robot& robot, Position goal, tick_t currentTime);

private:
    const Warehouse& warehouse_;
    std::unique_ptr<PathPlanner> planner_;
    ReservationTable reservations_;
    tick_t horizon_;
};

}  // namespace sim
