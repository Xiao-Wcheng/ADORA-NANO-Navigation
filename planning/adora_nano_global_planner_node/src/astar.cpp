#include "astar.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace {

using Graph = boost::adjacency_list<
    boost::vecS, boost::vecS, boost::undirectedS, boost::no_property,
    boost::property<boost::edge_weight_t, double>>;
using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

struct GoalFound {};

class GoalVisitor : public boost::default_astar_visitor {
public:
    explicit GoalVisitor(Vertex goal) : goal_(goal) {}

    template <class GraphType>
    void examine_vertex(Vertex vertex, const GraphType&) {
        if (vertex == goal_) throw GoalFound{};
    }

private:
    Vertex goal_;
};

class OctileHeuristic : public boost::astar_heuristic<Graph, double> {
public:
    OctileHeuristic(int width, Point goal) : width_(width), goal_(goal) {}

    double operator()(Vertex vertex) const {
        const int x = static_cast<int>(vertex) % width_;
        const int y = static_cast<int>(vertex) / width_;
        const int dx = std::abs(x - goal_.x);
        const int dy = std::abs(y - goal_.y);
        const int diagonal = std::min(dx, dy);
        const int cardinal = std::max(dx, dy) - diagonal;
        return static_cast<double>(cardinal) + std::sqrt(2.0) * diagonal;
    }

private:
    int width_;
    Point goal_;
};

bool Free(const MapLoader &map, int x, int y) {
    return map.isValid(x, y) && !map.isOccupied(x, y);
}

Vertex ToVertex(int x, int y, int width) {
    return static_cast<Vertex>(y * width + x);
}

Point ToPoint(Vertex vertex, int width) {
    return {static_cast<int>(vertex) % width, static_cast<int>(vertex) / width};
}

}  // namespace

AStar::AStar(const MapLoader& map)
    : map_(map), path_length_(0.0), compute_time_(0.0), path_found_(false) {}

std::vector<Point> AStar::findPath(const Point& start, const Point& goal) {
    const auto started = std::chrono::high_resolution_clock::now();
    path_length_ = 0.0;
    compute_time_ = 0.0;
    path_found_ = false;

    const int width = map_.getWidth();
    const int height = map_.getHeight();
    if (width <= 0 || height <= 0 || !Free(map_, start.x, start.y) ||
        !Free(map_, goal.x, goal.y)) {
        return {};
    }

    Graph graph(static_cast<std::size_t>(width * height));
    constexpr int edge_dx[] = {1, 0, 1, 1};
    constexpr int edge_dy[] = {0, 1, 1, -1};
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (!Free(map_, x, y)) continue;
            for (int index = 0; index < 4; ++index) {
                const int nx = x + edge_dx[index];
                const int ny = y + edge_dy[index];
                if (!Free(map_, nx, ny)) continue;
                const bool diagonal = edge_dx[index] != 0 && edge_dy[index] != 0;
                if (diagonal &&
                    (!Free(map_, x + edge_dx[index], y) ||
                     !Free(map_, x, y + edge_dy[index]))) {
                    continue;
                }
                boost::add_edge(ToVertex(x, y, width), ToVertex(nx, ny, width),
                                diagonal ? std::sqrt(2.0) : 1.0, graph);
            }
        }
    }

    const Vertex start_vertex = ToVertex(start.x, start.y, width);
    const Vertex goal_vertex = ToVertex(goal.x, goal.y, width);
    std::vector<Vertex> predecessor(boost::num_vertices(graph));
    std::vector<double> distance(boost::num_vertices(graph),
                                 std::numeric_limits<double>::infinity());
    predecessor[start_vertex] = start_vertex;

    try {
        boost::astar_search(
            graph, start_vertex, OctileHeuristic(width, goal),
            boost::predecessor_map(predecessor.data())
                .distance_map(distance.data())
                .weight_map(boost::get(boost::edge_weight, graph))
                .visitor(GoalVisitor(goal_vertex)));
    } catch (const GoalFound&) {
        std::vector<Point> path;
        Vertex current = goal_vertex;
        path.push_back(ToPoint(current, width));
        while (current != start_vertex) {
            const Vertex previous = predecessor[current];
            if (previous == current || previous >= boost::num_vertices(graph)) return {};
            current = previous;
            path.push_back(ToPoint(current, width));
        }
        std::reverse(path.begin(), path.end());
        path_found_ = true;
        path_length_ = distance[goal_vertex];
        const auto ended = std::chrono::high_resolution_clock::now();
        compute_time_ = std::chrono::duration<double, std::milli>(ended - started).count();
        return path;
    }

    const auto ended = std::chrono::high_resolution_clock::now();
    compute_time_ = std::chrono::duration<double, std::milli>(ended - started).count();
    return {};
}
