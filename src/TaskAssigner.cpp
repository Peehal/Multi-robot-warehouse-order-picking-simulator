#include "TaskAssigner.hpp"

#include <algorithm>
#include <cstdlib>
#include <limits>

using namespace std;

namespace sim {

int GreedyTaskAssigner::manhattan(Position a, Position b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

int GreedyTaskAssigner::distanceToOrder(Position robotPos, const Order& order) {
    int best = numeric_limits<int>::max();
    for (const auto& item : order.items) {
        best = min(best, manhattan(robotPos, item));
    }
    return best;
}

vector<Assignment> GreedyTaskAssigner::assign(const vector<Robot>& robots,
                                               const vector<Order>& orders) const {
    vector<bool> robotTaken(robots.size(), false);
    vector<bool> orderTaken(orders.size(), false);
    vector<Assignment> assignments;

    size_t rounds = min(robots.size(), orders.size());
    for (size_t step = 0; step < rounds; ++step) {
        int bestRobotIdx = -1;
        int bestOrderIdx = -1;
        int bestDist = numeric_limits<int>::max();

        for (size_t r = 0; r < robots.size(); ++r) {
            if (robotTaken[r]) continue;
            for (size_t o = 0; o < orders.size(); ++o) {
                if (orderTaken[o]) continue;
                int d = distanceToOrder(robots[r].position(), orders[o]);
                if (d < bestDist) {
                    bestDist = d;
                    bestRobotIdx = static_cast<int>(r);
                    bestOrderIdx = static_cast<int>(o);
                }
            }
        }

        if (bestRobotIdx == -1) break;

        robotTaken[bestRobotIdx] = true;
        orderTaken[bestOrderIdx] = true;
        assignments.push_back(Assignment{robots[bestRobotIdx].id(), orders[bestOrderIdx].id});
    }

    return assignments;
}

}  // namespace sim
