#include <iostream>
#include <memory>
#include <vector>

#include "BfsPlanner.hpp"
#include "Robot.hpp"
#include "Scheduler.hpp"
#include "SimulationEngine.hpp"
#include "Warehouse.hpp"

// Scenario 1: a single-cell bridge at (2,1) connects the top row to the
// bottom row. Robot 0 and Robot 1 both need to cross it in opposite
// directions, which forces the Scheduler's reservation table to make one of
// them wait rather than collide.
//
//   row0(y=0): . . . . .
//   row1(y=1): # # . # #
//   row2(y=2): . . . . .
void runBridgeScenario() {
    std::cout << "\n========== Scenario 1: single-cell bridge ==========\n";

    sim::Warehouse warehouse(5, 3);
    warehouse.setShelf(0, 1);
    warehouse.setShelf(1, 1);
    warehouse.setShelf(3, 1);
    warehouse.setShelf(4, 1);
    warehouse.setStation(0, 0);

    const sim::tick_t horizon = 50;
    sim::Scheduler scheduler(
        warehouse, std::unique_ptr<sim::PathPlanner>(new sim::BfsPlanner(horizon)), horizon);

    std::vector<sim::Robot> robots;
    robots.emplace_back(0, sim::Position{0, 0});
    robots.emplace_back(1, sim::Position{4, 2});
    robots.emplace_back(2, sim::Position{4, 0});

    std::vector<sim::Position> goals = {
        sim::Position{0, 2},
        sim::Position{4, 0},
        sim::Position{0, 0},
    };

    bool allPlanned = true;
    for (size_t i = 0; i < robots.size(); ++i) {
        bool ok = scheduler.planRoute(robots[i], goals[i], 0);
        if (!ok) {
            std::cerr << "Robot " << robots[i].id() << " could not find a path!\n";
            allPlanned = false;
        }
    }
    if (!allPlanned) {
        std::cout << "Scenario infeasible: not all robots could be planned. Skipping run.\n";
        return;
    }

    sim::SimulationEngine engine(warehouse, robots);
    engine.run(30, /*verbose=*/true);
    engine.metrics().print();
}

// Scenario 2: a single-wide corridor with two robots starting at opposite
// ends heading straight at each other, plus a one-cell passing bay at (5,1)
// near robot 1's start. A strict 1-wide corridor with no passing bay is
// provably unsolvable -- two robots crossing in opposite directions must
// occupy the same cell at the same tick somewhere in the middle, no matter
// how long either one waits.
//
// The bay's position matters: a bay at the exact midpoint (column 3) does
// NOT help, because both robots are equidistant from it and always arrive at
// its single entrance at the same tick -- neither can duck in ahead of the
// other. The bay must sit off-center, near whichever robot needs to yield,
// so that robot can reach it early, wait out the other robot's entire
// crossing, then proceed once the corridor is clear.
//
// Robot 0 is planned first (no reservations exist yet) and always takes the
// direct route regardless of the bay's position -- there is nothing yet to
// avoid. Only the second-planned robot (1) ever needs the bay, so it is
// placed one cell from robot 1's start.
//
//   row0(y=0): . . . . . . .
//   row1(y=1): # # # # # . #   (passing bay only at column 5)
void runHeadOnCorridorScenario() {
    std::cout << "\n========== Scenario 2: head-on corridor with passing bay ==========\n";

    sim::Warehouse warehouse(7, 2);
    for (int x = 0; x < 7; ++x) {
        if (x != 5) warehouse.setShelf(x, 1);
    }

    const sim::tick_t horizon = 50;
    sim::Scheduler scheduler(
        warehouse, std::unique_ptr<sim::PathPlanner>(new sim::BfsPlanner(horizon)), horizon);

    std::vector<sim::Robot> robots;
    robots.emplace_back(0, sim::Position{0, 0});
    robots.emplace_back(1, sim::Position{6, 0});

    std::vector<sim::Position> goals = {
        sim::Position{6, 0},
        sim::Position{0, 0},
    };

    bool allPlanned = true;
    for (size_t i = 0; i < robots.size(); ++i) {
        bool ok = scheduler.planRoute(robots[i], goals[i], 0);
        if (!ok) {
            std::cerr << "Robot " << robots[i].id() << " could not find a path!\n";
            allPlanned = false;
        }
    }
    if (!allPlanned) {
        std::cout << "Scenario infeasible: not all robots could be planned. Skipping run.\n";
        return;
    }

    sim::SimulationEngine engine(warehouse, robots);
    engine.run(30, /*verbose=*/true);
    engine.metrics().print();
}

// Scenario 3: goal is fully boxed in by shelves -- planRoute must report
// failure rather than crash or hang.
void runUnreachableGoalScenario() {
    std::cout << "\n========== Scenario 3: unreachable goal ==========\n";

    sim::Warehouse warehouse(5, 5);
    warehouse.setShelf(2, 1);
    warehouse.setShelf(1, 2);
    warehouse.setShelf(2, 2);
    warehouse.setShelf(3, 2);
    warehouse.setShelf(2, 3);

    const sim::tick_t horizon = 50;
    sim::Scheduler scheduler(
        warehouse, std::unique_ptr<sim::PathPlanner>(new sim::BfsPlanner(horizon)), horizon);

    std::vector<sim::Robot> robots;
    robots.emplace_back(0, sim::Position{0, 0});

    bool ok = scheduler.planRoute(robots[0], sim::Position{2, 2}, 0);
    std::cout << (ok ? "UNEXPECTED: path was found to a boxed-in cell\n"
                      : "EXPECTED: planRoute correctly reported no path (goal is boxed in)\n");
}

int main() {
    runBridgeScenario();
    runHeadOnCorridorScenario();
    runUnreachableGoalScenario();
    return 0;
}
