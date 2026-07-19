#pragma once

#include <unordered_map>

#include "Types.hpp"

namespace sim {

// Tracks who occupies which cell at which tick (vertex reservations) and who
// moves along which edge during which tick (edge reservations), so a BFS
// planning a new robot's route can avoid both head-on and swap collisions.
class ReservationTable {
public:
    bool isVertexReserved(Position pos, tick_t t) const;
    bool isEdgeReserved(Position from, Position to, tick_t t) const;

    void reserveVertex(Position pos, tick_t t, int robotId);
    void reserveEdge(Position from, Position to, tick_t t, int robotId);

    // Keeps a robot's final resting cell blocked for all future ticks up to
    // horizon so later robots don't plan a route through an occupied cell.
    void reserveGoalForever(Position pos, tick_t fromTime, tick_t horizon, int robotId);

private:
    struct VertexKey {
        int x, y;
        tick_t t;
        bool operator==(const VertexKey& other) const {
            return x == other.x && y == other.y && t == other.t;
        }
    };
    struct VertexKeyHash {
        size_t operator()(const VertexKey& k) const noexcept {
            size_t h = std::hash<int>()(k.x);
            h = h * 31 + std::hash<int>()(k.y);
            h = h * 31 + std::hash<sim::tick_t>()(k.t);
            return h;
        }
    };

    struct EdgeKey {
        int x1, y1, x2, y2;
        tick_t t;
        bool operator==(const EdgeKey& other) const {
            return x1 == other.x1 && y1 == other.y1 && x2 == other.x2 && y2 == other.y2 &&
                   t == other.t;
        }
    };
    struct EdgeKeyHash {
        size_t operator()(const EdgeKey& k) const noexcept {
            size_t h = std::hash<int>()(k.x1);
            h = h * 31 + std::hash<int>()(k.y1);
            h = h * 31 + std::hash<int>()(k.x2);
            h = h * 31 + std::hash<int>()(k.y2);
            h = h * 31 + std::hash<sim::tick_t>()(k.t);
            return h;
        }
    };

    std::unordered_map<VertexKey, int, VertexKeyHash> vertexReservations_;
    std::unordered_map<EdgeKey, int, EdgeKeyHash> edgeReservations_;
};

}  // namespace sim
