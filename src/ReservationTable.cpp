#include "ReservationTable.hpp"

namespace sim {

bool ReservationTable::isVertexReserved(Position pos, tick_t t) const {
    VertexKey key{pos.x, pos.y, t};
    return vertexReservations_.find(key) != vertexReservations_.end();
}

bool ReservationTable::isEdgeReserved(Position from, Position to, tick_t t) const {
    EdgeKey key{from.x, from.y, to.x, to.y, t};
    return edgeReservations_.find(key) != edgeReservations_.end();
}

void ReservationTable::reserveVertex(Position pos, tick_t t, int robotId) {
    VertexKey key{pos.x, pos.y, t};
    vertexReservations_[key] = robotId;
}

void ReservationTable::reserveEdge(Position from, Position to, tick_t t, int robotId) {
    EdgeKey key{from.x, from.y, to.x, to.y, t};
    edgeReservations_[key] = robotId;
}

void ReservationTable::reserveGoalForever(Position pos, tick_t fromTime, tick_t horizon,
                                           int robotId) {
    for (tick_t t = fromTime; t <= horizon; ++t) {
        reserveVertex(pos, t, robotId);
    }
}

}  // namespace sim
