#include "Warehouse.hpp"

#include <stdexcept>

namespace sim {

Warehouse::Warehouse(int width, int height)
    : width_(width), height_(height), grid_(static_cast<size_t>(width * height), CellType::Free) {}

void Warehouse::setShelf(int x, int y) {
    if (!inBounds(x, y)) throw std::out_of_range("setShelf: position out of bounds");
    grid_[index(x, y)] = CellType::Shelf;
}

void Warehouse::setStation(int x, int y) {
    if (!inBounds(x, y)) throw std::out_of_range("setStation: position out of bounds");
    grid_[index(x, y)] = CellType::Station;
    station_ = Position{x, y};
}

bool Warehouse::inBounds(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
}

bool Warehouse::isBlocked(int x, int y) const {
    return cellType(x, y) == CellType::Shelf;
}

CellType Warehouse::cellType(int x, int y) const {
    return grid_[index(x, y)];
}

}  // namespace sim
