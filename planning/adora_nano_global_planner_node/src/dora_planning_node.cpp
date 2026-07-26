// DORA Path Planning Node
extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <nlohmann/json.hpp>

#include "map_loader.h"
#include "astar.h"
#include "nav2_simple_smoother.h"
#include "self_filter.h"

using json = nlohmann::json;
using namespace std;

// Global variables
MapLoader map_loader;
string map_yaml_path;
constexpr const char* algorithm_type = "boost_astar";
bool map_loaded = false;
bool reload_map_on_plan = false;
int inflation_radius_cells = 0;
int max_nearest_free_radius_cells = 8;
bool enable_path_smoothing = true;
double dynamic_obstacle_inflation_m = 0.25;
double dynamic_obstacle_max_range = 4.0;
double lidar_x = 0.09;
double lidar_y = 0.06;
double lidar_yaw = 0.0;
int dynamic_obstacle_timeout_ms = 700;
int dynamic_scan_stride = 2;
SelfFilterConfig self_filter;

struct WorldPose {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    bool valid{false};
};

WorldPose latest_pose;
json latest_scan;
bool have_scan = false;
int64_t last_scan_ms = 0;

Point start_point;
Point goal_point;
bool start_received = false;
bool goal_received = false;

// Function declarations
int run(void *dora_context);
void process_start_point(char *data, size_t data_len);
void process_goal_point(char *data, size_t data_len);
void plan_and_publish_path(void *dora_context);
bool load_planning_map();
Point nearest_free_point(const Point& point, int max_radius_cells);
std::string point_key(const Point& point);
void process_pose(char *data, size_t data_len);
void process_scan(char *data, size_t data_len);
void rebuild_dynamic_obstacles();

int64_t now_ms()
{
    return chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
}

int main()
{
    cout << "=== DORA Path Planning Node ===" << endl;

    // Load map path from environment variable
    const char* map_path_env = std::getenv("MAP_YAML_PATH");
    if (map_path_env) {
        map_yaml_path = string(map_path_env);
    } else {
        map_yaml_path = "map/test_map.yaml";
    }

    const char* reload_env = std::getenv("RELOAD_MAP_ON_PLAN");
    if (reload_env) {
        string value(reload_env);
        reload_map_on_plan = value == "1" || value == "true" || value == "TRUE" || value == "yes";
    }

    const char* inflation_env = std::getenv("INFLATION_RADIUS_CELLS");
    if (inflation_env) {
        inflation_radius_cells = std::max(0, std::stoi(inflation_env));
    }

    const char* nearest_env = std::getenv("MAX_NEAREST_FREE_RADIUS_CELLS");
    if (nearest_env) {
        max_nearest_free_radius_cells = std::max(0, std::stoi(nearest_env));
    }
    const char* smoothing_env = std::getenv("ENABLE_PATH_SMOOTHING");
    if (smoothing_env) {
        string value(smoothing_env);
        enable_path_smoothing = value == "1" || value == "true" || value == "TRUE" || value == "yes";
    }
    if (const char* value = std::getenv("DYNAMIC_OBSTACLE_INFLATION_M")) dynamic_obstacle_inflation_m = std::max(0.0, std::stod(value));
    if (const char* value = std::getenv("DYNAMIC_OBSTACLE_MAX_RANGE")) dynamic_obstacle_max_range = std::max(0.1, std::stod(value));
    if (const char* value = std::getenv("DYNAMIC_OBSTACLE_TIMEOUT_MS")) dynamic_obstacle_timeout_ms = std::max(100, std::stoi(value));
    if (const char* value = std::getenv("DYNAMIC_SCAN_STRIDE")) dynamic_scan_stride = std::max(1, std::stoi(value));
    if (const char* value = std::getenv("LIDAR_X")) lidar_x = std::stod(value);
    if (const char* value = std::getenv("LIDAR_Y")) lidar_y = std::stod(value);
    if (const char* value = std::getenv("LIDAR_YAW")) lidar_yaw = std::stod(value);
    if (const char* value = std::getenv("FOOTPRINT_HALF_LENGTH")) self_filter.half_length = std::stod(value);
    if (const char* value = std::getenv("FOOTPRINT_HALF_WIDTH")) self_filter.half_width = std::stod(value);
    if (const char* value = std::getenv("SELF_FILTER_PADDING")) self_filter.padding = std::stod(value);

    cout << "Map path: " << map_yaml_path << endl;
    cout << "Algorithm: " << algorithm_type << endl;
    cout << "Reload map on plan: " << (reload_map_on_plan ? "true" : "false") << endl;
    cout << "Inflation radius cells: " << inflation_radius_cells << endl;
    cout << "Max nearest free radius cells: " << max_nearest_free_radius_cells << endl;
    cout << "Path smoothing: " << (enable_path_smoothing ? "true" : "false") << endl;
    cout << "Dynamic obstacle layer: inflation=" << dynamic_obstacle_inflation_m
         << "m timeout=" << dynamic_obstacle_timeout_ms << "ms max_range="
         << dynamic_obstacle_max_range << "m" << endl;

    // Load map
    if (!load_planning_map()) {
        cerr << "Failed to load map!" << endl;
        return -1;
    }

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    cout << "Exit Path Planning Node..." << endl;
    return ret;
}

int run(void *dora_context)
{
    while(true)
    {
        void *event = dora_next_event(dora_context);

        if (event == NULL)
        {
            printf("[path planning node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);

            if (strncmp(id, "start_point", 11) == 0)
            {
                char *data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                process_start_point(data, data_len);

                // If both start and goal received, plan path
                if (start_received && goal_received) {
                    plan_and_publish_path(dora_context);
                    start_received = false;
                    goal_received = false;
                }
            }
            else if (strncmp(id, "goal_point", 10) == 0)
            {
                char *data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                process_goal_point(data, data_len);

                // If both start and goal received, plan path
                if (start_received && goal_received) {
                    plan_and_publish_path(dora_context);
                    start_received = false;
                    goal_received = false;
                }
            }
            else if (strncmp(id, "CorrectedPose", 13) == 0 || strncmp(id, "Pose", 4) == 0 ||
                     strncmp(id, "Odometry", 8) == 0)
            {
                char *data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                process_pose(data, data_len);
            }
            else if (strncmp(id, "LaserScan", 9) == 0)
            {
                char *data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                process_scan(data, data_len);
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[path planning node] received stop event\n");
            break;
        }
        else
        {
            printf("[path planning node] received unexpected event: %d\n", ty);
        }
        free_dora_event(event);
    }
    return 0;
}

bool load_planning_map()
{
    if (!map_loader.loadMap(map_yaml_path)) {
        map_loaded = false;
        return false;
    }
    if (inflation_radius_cells > 0) {
        map_loader.inflateObstacles(inflation_radius_cells);
    }
    if (have_scan && latest_pose.valid && now_ms() - last_scan_ms <= dynamic_obstacle_timeout_ms) {
        rebuild_dynamic_obstacles();
    }
    map_loaded = true;
    cout << "Map loaded successfully!" << endl;
    cout << "Map size: " << map_loader.getWidth() << "x"
         << map_loader.getHeight() << endl;
    return true;
}

void process_pose(char *data, size_t data_len)
{
    try {
        const json msg = json::parse(string(data, data_len));
        const json *pose = &msg;
        if (msg.contains("pose") && msg.at("pose").is_object()) pose = &msg.at("pose");
        if (!pose->contains("x") || !pose->contains("y")) return;
        latest_pose.x = pose->value("x", 0.0);
        latest_pose.y = pose->value("y", 0.0);
        latest_pose.theta = pose->value("theta", pose->value("yaw", msg.value("theta", 0.0)));
        latest_pose.valid = true;
        if (have_scan && now_ms() - last_scan_ms <= dynamic_obstacle_timeout_ms) rebuild_dynamic_obstacles();
    } catch (const exception& e) {
        cerr << "failed to parse CorrectedPose: " << e.what() << endl;
    }
}

void process_scan(char *data, size_t data_len)
{
    try {
        latest_scan = json::parse(string(data, data_len));
        have_scan = latest_scan.contains("ranges") && latest_scan.at("ranges").is_array();
        last_scan_ms = now_ms();
        if (have_scan && latest_pose.valid) rebuild_dynamic_obstacles();
    } catch (const exception& e) {
        cerr << "failed to parse LaserScan: " << e.what() << endl;
    }
}

void rebuild_dynamic_obstacles()
{
    map_loader.clearDynamicObstacles();
    if (!map_loaded || !have_scan || !latest_pose.valid) return;
    const auto& info = map_loader.getMapInfo();
    if (info.origin.size() < 2 || info.resolution <= 0.0) return;
    const double base_c = cos(latest_pose.theta);
    const double base_s = sin(latest_pose.theta);
    const double angle_min = latest_scan.value("angle_min", 0.0);
    const double angle_increment = latest_scan.value("angle_increment", 0.0);
    const double range_min = latest_scan.value("range_min", 0.10);
    const double range_max = std::min(dynamic_obstacle_max_range, latest_scan.value("range_max", dynamic_obstacle_max_range));
    const auto& ranges = latest_scan.at("ranges");
    const int inflation_cells = static_cast<int>(ceil(dynamic_obstacle_inflation_m / info.resolution));
    for (size_t i = 0; i < ranges.size(); i += static_cast<size_t>(dynamic_scan_stride)) {
        if (!ranges.at(i).is_number()) continue;
        const double range = ranges.at(i).get<double>();
        if (!std::isfinite(range) || range < range_min || range > range_max) continue;
        const double base_angle = lidar_yaw + angle_min + static_cast<double>(i) * angle_increment;
        const double base_x = lidar_x + range * cos(base_angle);
        const double base_y = lidar_y + range * sin(base_angle);
        if (inside_robot_footprint(base_x, base_y, self_filter)) continue;
        const double world_x = latest_pose.x + base_c * base_x - base_s * base_y;
        const double world_y = latest_pose.y + base_s * base_x + base_c * base_y;
        const int grid_x = static_cast<int>(floor((world_x - info.origin[0]) / info.resolution));
        const int world_grid_y = static_cast<int>(floor((world_y - info.origin[1]) / info.resolution));
        const int image_y = map_loader.getHeight() - 1 - world_grid_y;
        if (map_loader.isValid(grid_x, image_y)) map_loader.addDynamicObstacle(grid_x, image_y, inflation_cells);
    }
}

std::string point_key(const Point& point)
{
    return std::to_string(point.x) + "," + std::to_string(point.y);
}

Point nearest_free_point(const Point& point, int max_radius_cells)
{
    if (map_loader.isValid(point.x, point.y) && !map_loader.isOccupied(point.x, point.y)) {
        return point;
    }

    std::queue<Point> queue;
    std::unordered_set<std::string> visited;
    queue.push(point);
    visited.insert(point_key(point));

    const int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

    while (!queue.empty()) {
        Point current = queue.front();
        queue.pop();

        const int chebyshev_radius = std::max(std::abs(current.x - point.x), std::abs(current.y - point.y));
        if (chebyshev_radius > max_radius_cells) {
            continue;
        }

        if (map_loader.isValid(current.x, current.y) && !map_loader.isOccupied(current.x, current.y)) {
            return current;
        }

        for (int i = 0; i < 8; ++i) {
            Point next(current.x + dx[i], current.y + dy[i]);
            const std::string key = point_key(next);
            if (visited.find(key) != visited.end()) {
                continue;
            }
            visited.insert(key);
            queue.push(next);
        }
    }

    return point;
}

void process_start_point(char *data, size_t data_len)
{
    string data_str(data, data_len);
    try {
        json j = json::parse(data_str);
        start_point.x = j["x"];
        start_point.y = j["y"];
        start_received = true;
        cout << "Received start point: (" << start_point.x << ", "
             << start_point.y << ")" << endl;
    } catch (const json::parse_error& e) {
        cerr << "JSON parse error in start_point: " << e.what() << endl;
    }
}

void process_goal_point(char *data, size_t data_len)
{
    string data_str(data, data_len);
    try {
        json j = json::parse(data_str);
        goal_point.x = j["x"];
        goal_point.y = j["y"];
        goal_received = true;
        cout << "Received goal point: (" << goal_point.x << ", "
             << goal_point.y << ")" << endl;
    } catch (const json::parse_error& e) {
        cerr << "JSON parse error in goal_point: " << e.what() << endl;
    }
}

void plan_and_publish_path(void *dora_context)
{
    if (reload_map_on_plan && !load_planning_map()) {
        cerr << "Map reload failed!" << endl;
        return;
    }

    if (!map_loaded) {
        cerr << "Map not loaded!" << endl;
        return;
    }
    if (!have_scan || now_ms() - last_scan_ms > dynamic_obstacle_timeout_ms) {
        map_loader.clearDynamicObstacles();
    }

    cout << "\n=== Planning path ===" << endl;
    cout << "Start: (" << start_point.x << ", " << start_point.y << ")" << endl;
    cout << "Goal: (" << goal_point.x << ", " << goal_point.y << ")" << endl;

    Point planning_start = nearest_free_point(start_point, max_nearest_free_radius_cells);
    Point planning_goal = nearest_free_point(goal_point, max_nearest_free_radius_cells);

    if (!(planning_start == start_point)) {
        cout << "Adjusted start to nearest free point: (" << planning_start.x << ", "
             << planning_start.y << ")" << endl;
    }
    if (!(planning_goal == goal_point)) {
        cout << "Adjusted goal to nearest free point: (" << planning_goal.x << ", "
             << planning_goal.y << ")" << endl;
    }

    vector<Point> path;
    double path_length = 0.0;
    double compute_time = 0.0;
    bool path_found = false;

    AStar planner(map_loader);
    path = planner.findPath(planning_start, planning_goal);
    path_length = planner.getPathLength();
    compute_time = planner.getComputeTime();
    path_found = planner.pathFound();

    const size_t raw_waypoint_count = path.size();
    vector<GridPathPoint> output_path;
    if (path_found && enable_path_smoothing) {
        output_path = Nav2SimpleSmoother(map_loader).smooth(path);
    } else {
        output_path.reserve(path.size());
        for (const auto& point : path) {
            output_path.push_back(
                {static_cast<double>(point.x), static_cast<double>(point.y)});
        }
    }

    // Print results
    cout << "Algorithm: " << algorithm_type << endl;
    cout << "Path found: " << (path_found ? "YES" : "NO") << endl;
    if (path_found) {
        cout << "Path length: " << path_length << " grid units" << endl;
        cout << "Waypoints: " << raw_waypoint_count << " -> " << output_path.size() << endl;
        cout << "Compute time: " << compute_time << " ms" << endl;
    }

    // Create JSON output
    json j_path;
    j_path["algorithm"] = algorithm_type;
    j_path["path_found"] = path_found;
    j_path["path_length"] = path_length;
    j_path["compute_time_ms"] = compute_time;
    j_path["num_waypoints"] = output_path.size();
    j_path["raw_num_waypoints"] = raw_waypoint_count;
    j_path["path_smoothed"] = enable_path_smoothing && path_found;
    j_path["path_smoother"] =
        enable_path_smoothing && path_found ? "nav2_simple_smoother_port" : "none";
    j_path["dynamic_obstacle_cells"] = map_loader.getDynamicObstacleCount();
    j_path["dynamic_scan_age_ms"] = have_scan ? now_ms() - last_scan_ms : -1;
    j_path["requested_start"] = {{"x", start_point.x}, {"y", start_point.y}};
    j_path["requested_goal"] = {{"x", goal_point.x}, {"y", goal_point.y}};
    j_path["planning_start"] = {{"x", planning_start.x}, {"y", planning_start.y}};
    j_path["planning_goal"] = {{"x", planning_goal.x}, {"y", planning_goal.y}};

    json waypoints_array = json::array();
    for (const auto& point : output_path) {
        json waypoint;
        waypoint["x"] = point.x;
        waypoint["y"] = point.y;
        waypoints_array.push_back(waypoint);
    }
    j_path["waypoints"] = waypoints_array;

    // Serialize and send
    string json_string = j_path.dump();
    char *c_json_string = new char[json_string.length() + 1];
    strcpy(c_json_string, json_string.c_str());

    string out_id = "path";
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(),
                                   c_json_string, strlen(c_json_string));

    if (result != 0) {
        cerr << "Failed to send path output" << endl;
    } else {
        cout << "Path published successfully!" << endl;
    }

    delete[] c_json_string;
}
