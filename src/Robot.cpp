#include "Robot.hpp"

using namespace std;

namespace sim {

Robot::Robot(int id, Position start) : id_(id), position_(start) {}

void Robot::assignPath(const PlannedPath& path) {
    path_ = path;
    hasPath_ = true;
    completionTick_ = kNoTick;
    distanceTraveled_ = 0;
}

void Robot::step(tick_t t) {
    if (!hasPath_) return;

    size_t idx = static_cast<size_t>(t - path_.startTime);
    if (idx >= path_.waypoints.size()) return;

    Position prev = position_;
    position_ = path_.waypoints[idx];
    if (position_ != prev) ++distanceTraveled_;

    if (idx == path_.waypoints.size() - 1 && completionTick_ == kNoTick) {
        completionTick_ = t;
    }
}

}  // namespace sim
