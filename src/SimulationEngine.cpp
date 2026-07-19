#include "SimulationEngine.hpp"

#include <iostream>

namespace sim {

SimulationEngine::SimulationEngine(const Warehouse& warehouse, std::vector<Robot>& robots)
    : warehouse_(warehouse), robots_(robots) {}

bool SimulationEngine::allDone() const {
    for (const auto& robot : robots_) {
        if (!robot.isDone()) return false;
    }
    return true;
}

void SimulationEngine::printGrid(tick_t t) const {
    std::cout << "\n-- tick " << t << " --\n";
    for (int y = 0; y < warehouse_.height(); ++y) {
        for (int x = 0; x < warehouse_.width(); ++x) {
            char ch = '.';
            switch (warehouse_.cellType(x, y)) {
                case CellType::Shelf:
                    ch = '#';
                    break;
                case CellType::Station:
                    ch = 'S';
                    break;
                case CellType::Free:
                    ch = '.';
                    break;
            }
            for (const auto& robot : robots_) {
                if (robot.position() == Position{x, y}) {
                    ch = static_cast<char>('0' + robot.id());
                    break;
                }
            }
            std::cout << ch << ' ';
        }
        std::cout << "\n";
    }
}

void SimulationEngine::checkCollisions(tick_t t, const std::vector<Position>& prevPositions) {
    for (size_t i = 0; i < robots_.size(); ++i) {
        for (size_t j = i + 1; j < robots_.size(); ++j) {
            Position a = robots_[i].position();
            Position b = robots_[j].position();
            if (a == b) {
                std::cout << "COLLISION at tick " << t << ": robot " << robots_[i].id()
                          << " and robot " << robots_[j].id() << " both at (" << a.x << ","
                          << a.y << ")\n";
                collisionFree_ = false;
            }
            if (prevPositions[i] == b && prevPositions[j] == a && a != b) {
                std::cout << "SWAP COLLISION at tick " << t << ": robot " << robots_[i].id()
                          << " and robot " << robots_[j].id() << " traded cells\n";
                collisionFree_ = false;
            }
        }
    }
}

void SimulationEngine::run(tick_t maxTicks, bool verbose) {
    for (tick_t t = 0; t <= maxTicks; ++t) {
        std::vector<Position> prevPositions;
        prevPositions.reserve(robots_.size());
        for (const auto& robot : robots_) prevPositions.push_back(robot.position());

        for (auto& robot : robots_) {
            robot.step(t);
        }

        checkCollisions(t, prevPositions);
        if (verbose) printGrid(t);
        if (allDone()) break;
    }
    metrics_.computeFrom(robots_);

    std::cout << "\n--- Collision check ---\n";
    std::cout << (collisionFree_ ? "PASS: no collisions detected across the run.\n"
                                  : "FAIL: collisions detected, see above.\n");
}

}  // namespace sim
