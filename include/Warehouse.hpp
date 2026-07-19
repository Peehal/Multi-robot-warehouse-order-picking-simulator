#pragma once

#include <vector>

#include "Types.hpp"

namespace sim {

class Warehouse {
public:
    Warehouse(int width, int height);

    int width() const { return width_; }
    int height() const { return height_; }

    void setShelf(int x, int y);
    void setStation(int x, int y);

    bool inBounds(int x, int y) const;
    bool isBlocked(int x, int y) const;
    CellType cellType(int x, int y) const;

    Position station() const { return station_; }

private:
    int width_;
    int height_;
    std::vector<CellType> grid_;
    Position station_{0, 0};

    int index(int x, int y) const { return y * width_ + x; }
};

}  // namespace sim
