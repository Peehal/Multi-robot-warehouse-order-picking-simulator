#include "BfsPlanner.hpp"
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

using namespace std;

namespace sim {

namespace {

struct State {
    Position pos;
    tick_t t;
    bool operator==(const State& other) const { return pos == other.pos && t == other.t; }
};

struct StateHash {
    size_t operator()(const State& s) const noexcept {
        size_t h = hash<Position>()(s.pos);
        h = h * 31 + hash<tick_t>()(s.t);
        return h;
    }
};

}  // namespace

PlanResult BfsPlanner::plan(const Warehouse& warehouse, Position start, Position goal,
                             tick_t startTime, const ReservationTable& reservations) const {
    PlanResult result;

    if (start == goal) {
        result.found = true;
        result.path.startTime = startTime;
        result.path.waypoints.push_back(start);
        return result;
    }

    static const int dx[5] = {0, 1, -1, 0, 0};
    static const int dy[5] = {0, 0, 0, 1, -1};

    queue<State> frontier;
    unordered_map<State, State, StateHash> cameFrom;
    unordered_set<State, StateHash> visited;

    State startState{start, startTime};
    frontier.push(startState);
    visited.insert(startState);

    const tick_t deadline = startTime + horizon_;
    State goalState{start, startTime};
    bool found = false;

    while (!frontier.empty() && !found) {
        State cur = frontier.front();
        frontier.pop();
        if (cur.t >= deadline) continue;

        for (int dir = 0; dir < 5 && !found; ++dir) {
            Position next{cur.pos.x + dx[dir], cur.pos.y + dy[dir]};
            tick_t nextTick = cur.t + 1;

            if (!warehouse.inBounds(next.x, next.y)) continue;
            if (warehouse.isBlocked(next.x, next.y)) continue;
            if (reservations.isVertexReserved(next, nextTick)) continue;
            // Edge/swap check: another robot moving next -> cur during this
            // same tick would collide with us moving cur -> next head-on.
            if (dir != 0 && reservations.isEdgeReserved(next, cur.pos, cur.t)) continue;

            State nextState{next, nextTick};
            if (visited.count(nextState)) continue;

            visited.insert(nextState);
            cameFrom[nextState] = cur;

            if (next == goal) {
                goalState = nextState;
                found = true;
                break;
            }
            frontier.push(nextState);
        }
    }

    if (!found) return result;

    vector<Position> path;
    State cur = goalState;
    while (true) {
        path.push_back(cur.pos);
        if (cur.t == startTime) break;
        cur = cameFrom[cur];
    }
    reverse(path.begin(), path.end());

    result.found = true;
    result.path.startTime = startTime;
    result.path.waypoints = move(path);
    return result;
}

}  // namespace sim
