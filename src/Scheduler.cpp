#include "Scheduler.hpp"

using namespace std;

namespace sim {

Scheduler::Scheduler(const Warehouse& warehouse, unique_ptr<PathPlanner> planner,
                      tick_t horizon)
    : warehouse_(warehouse), planner_(move(planner)), horizon_(horizon) {}

bool Scheduler::planRoute(Robot& robot, Position goal, tick_t currentTime) {
    PlanResult result =
        planner_->plan(warehouse_, robot.position(), goal, currentTime, reservations_);
    if (!result.found) return false;

    const auto& waypoints = result.path.waypoints;
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        tick_t t = result.path.startTime + static_cast<tick_t>(i);
        reservations_.reserveVertex(waypoints[i], t, robot.id());
        reservations_.reserveEdge(waypoints[i], waypoints[i + 1], t, robot.id());
    }

    tick_t arrivalTick = result.path.startTime + static_cast<tick_t>(waypoints.size()) - 1;
    reservations_.reserveGoalForever(goal, arrivalTick, horizon_, robot.id());

    robot.assignPath(result.path);
    return true;
}

}  // namespace sim
