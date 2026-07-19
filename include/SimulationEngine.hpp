#pragma once

#include <vector>

#include "Metrics.hpp"
#include "Robot.hpp"
#include "Warehouse.hpp"

namespace sim {

class SimulationEngine {
public:
    SimulationEngine(const Warehouse& warehouse, std::vector<Robot>& robots);

    // Advances the tick loop until every robot is done or maxTicks is
    // reached. Prints the grid each tick when verbose is true.
    void run(tick_t maxTicks, bool verbose);

    const Metrics& metrics() const { return metrics_; }

    // True only if no two robots ever shared a cell or swapped cells across
    // the whole run. Checked independently of the ASCII view, which would
    // silently hide a same-cell collision (it only draws one robot per cell).
    bool collisionFree() const { return collisionFree_; }

private:
    const Warehouse& warehouse_;
    std::vector<Robot>& robots_;
    Metrics metrics_;
    bool collisionFree_ = true;

    bool allDone() const;
    void printGrid(tick_t t) const;
    void checkCollisions(tick_t t, const std::vector<Position>& prevPositions);
};

}  // namespace sim
