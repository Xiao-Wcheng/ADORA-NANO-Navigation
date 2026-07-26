#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include "map_loader.h"

struct Point {
    int x;
    int y;

    Point() : x(0), y(0) {}
    Point(int x_, int y_) : x(x_), y(y_) {}

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

struct PointHash {
    std::size_t operator()(const Point& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

class AStar {
public:
    AStar(const MapLoader& map);

    std::vector<Point> findPath(const Point& start, const Point& goal);
    double getPathLength() const { return path_length_; }
    double getComputeTime() const { return compute_time_; }
    bool pathFound() const { return path_found_; }

private:
    const MapLoader& map_;
    double path_length_;
    double compute_time_;
    bool path_found_;
};

#endif // ASTAR_H
