#pragma once

#include "PathPlanner.hpp"
#include "Types.hpp"

namespace sim {

class Robot {
public:
    Robot(int id, Position start);

    int id() const { return id_; }
    Position position() const { return position_; }
    bool hasPath() const { return hasPath_; }
    bool isDone() const { return completionTick_ != kNoTick; }

    void assignPath(const PlannedPath& path);

    // Advances the robot to its waypoint for tick `t`, tracking distance and
    // completion. Robots without an assigned path, or past their last
    // waypoint, simply stay put.
    void step(tick_t t);

    tick_t distanceTraveled() const { return distanceTraveled_; }
    tick_t completionTick() const { return completionTick_; }

private:
    int id_;
    Position position_;
    bool hasPath_ = false;
    PlannedPath path_;
    tick_t distanceTraveled_ = 0;
    tick_t completionTick_ = kNoTick;
};

}  // namespace sim
