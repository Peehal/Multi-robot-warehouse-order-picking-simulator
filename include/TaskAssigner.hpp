#pragma once

#include <vector>

#include "Order.hpp"
#include "Robot.hpp"

namespace sim {

struct Assignment {
    int robotId;
    int orderId;
};

// Greedy nearest-available-robot <-> nearest-order: repeatedly picks the
// globally cheapest remaining (robot, order) pair, assigns it, and removes
// both from the pool -- the textbook greedy pattern of a locally optimal
// choice at each step with no backtracking.
//
// Distance is Manhattan distance to an order's nearest item, used only as a
// cheap proxy to decide *who* should take *which* order. This is deliberately
// not the exact grid distance from Phase 1's BFS: computing a real shortest
// path for every candidate (robot, order) pair here would be wasted work
// once an order has more than one item, since the actual travel cost depends
// on the pickup sequence -- that sequencing problem is exactly what Phase
// 3's Held-Karp RoutePlanner solves.
class GreedyTaskAssigner {
public:
    std::vector<Assignment> assign(const std::vector<Robot>& robots,
                                    const std::vector<Order>& orders) const;

private:
    static int manhattan(Position a, Position b);
    static int distanceToOrder(Position robotPos, const Order& order);
};

}  // namespace sim
