#pragma once

#include <vector>

#include "Robot.hpp"
#include "Types.hpp"

namespace sim {

struct Metrics {
    tick_t makespan = 0;
    tick_t totalDistance = 0;
    std::vector<tick_t> perRobotDistance;
    std::vector<tick_t> perRobotCompletion;

    void computeFrom(const std::vector<Robot>& robots);
    void print() const;
};

}  // namespace sim
