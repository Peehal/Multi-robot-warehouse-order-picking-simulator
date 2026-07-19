#pragma once

#include <cstddef>
#include <functional>

namespace sim {

using tick_t = int;

constexpr tick_t kNoTick = -1;

struct Position {
    int x;
    int y;

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }
};

enum class CellType { Free, Shelf, Station };

}  // namespace sim

namespace std {
template <>
struct hash<sim::Position> {
    size_t operator()(const sim::Position& p) const noexcept {
        size_t h = std::hash<int>()(p.x);
        h = h * 31 + std::hash<int>()(p.y);
        return h;
    }
};
}  // namespace std
