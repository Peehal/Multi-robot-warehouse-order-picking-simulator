#include "Metrics.hpp"

#include <algorithm>
#include <iostream>

namespace sim {

void Metrics::computeFrom(const std::vector<Robot>& robots) {
    perRobotDistance.clear();
    perRobotCompletion.clear();
    makespan = 0;
    totalDistance = 0;

    for (const auto& robot : robots) {
        perRobotDistance.push_back(robot.distanceTraveled());
        perRobotCompletion.push_back(robot.completionTick());

        totalDistance += robot.distanceTraveled();
        if (robot.completionTick() > makespan) makespan = robot.completionTick();
    }
}

void Metrics::print() const {
    std::cout << "\n--- Metrics ---\n";
    for (size_t i = 0; i < perRobotDistance.size(); ++i) {
        std::cout << "Robot " << i << ": distance=" << perRobotDistance[i]
                   << " completedAtTick=" << perRobotCompletion[i] << "\n";
    }
    std::cout << "Makespan (ticks): " << makespan << "\n";
    std::cout << "Total distance across all robots: " << totalDistance << "\n";
}

}  // namespace sim
