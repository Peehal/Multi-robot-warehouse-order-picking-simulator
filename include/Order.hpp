#pragma once

#include <vector>

#include "Types.hpp"

namespace sim {

// Phase 2's Order already carries a full item list so the type doesn't need
// to change shape in Phase 3, but until the Held-Karp RoutePlanner exists to
// sequence multi-item pickups, demo orders are kept single-item: reaching
// the one item is a complete A -> B trip Phase 1's Scheduler can already
// execute.
struct Order {
    int id;
    std::vector<Position> items;
};

}  // namespace sim
