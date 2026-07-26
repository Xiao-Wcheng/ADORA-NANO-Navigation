extern "C"
{
#include "node_api.h"
}

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

using json = nlohmann::json;

namespace
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct GridPoint
{
  int x{0};
  int y{0};
};

struct Config
{
  double resolution{0.05};
  int map_width{400};
  int map_height{400};
  double origin_x{-10.0};
  double origin_y{-10.0};
  double goal_distance{0.30};
  double goal_lateral{0.0};
  double goal_x{0.0};
  double goal_y{0.0};
  double goal_theta{0.0};
  bool use_absolute_goal{false};
  bool send_once{true};
  bool latch_goal{true};
  bool require_localization_match{false};
  int max_localization_lost_count{0};
  int min_send_interval_ms{1000};
};

bool HasEnv(const char *name)
{
  const char *raw = std::getenv(name);
  return raw != nullptr && std::strlen(raw) != 0;
}

double GetEnvDouble(const char *name, double fallback)
{
  const char *raw = std::getenv(name);
  if (raw == nullptr || std::strlen(raw) == 0)
  {
    return fallback;
  }
  return std::stod(raw);
}

int GetEnvInt(const char *name, int fallback)
{
  const char *raw = std::getenv(name);
  if (raw == nullptr || std::strlen(raw) == 0)
  {
    return fallback;
  }
  return std::stoi(raw);
}

bool GetEnvBool(const char *name, bool fallback)
{
  const char *raw = std::getenv(name);
  if (raw == nullptr || std::strlen(raw) == 0)
  {
    return fallback;
  }
  const std::string value(raw);
  return value == "1" || value == "true" || value == "TRUE" || value == "yes";
}

int64_t NowMs()
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

bool ReadPose(const json &msg, Pose2D *pose)
{
  if (pose == nullptr)
  {
    return false;
  }
  if (msg.contains("pose") && msg.at("pose").contains("x") && msg.at("pose").contains("y"))
  {
    const auto &p = msg.at("pose");
    pose->x = p.value("x", 0.0);
    pose->y = p.value("y", 0.0);
    pose->theta = p.value("theta", p.value("yaw", msg.value("theta", msg.value("yaw", 0.0))));
    return true;
  }
  if (msg.contains("x") && msg.contains("y"))
  {
    pose->x = msg.value("x", 0.0);
    pose->y = msg.value("y", 0.0);
    pose->theta = msg.value("theta", msg.value("yaw", 0.0));
    return true;
  }
  return false;
}

GridPoint WorldToPlannerGrid(double world_x, double world_y, const Config &config)
{
  const int grid_x = static_cast<int>(std::floor((world_x - config.origin_x) / config.resolution));
  const int grid_y = static_cast<int>(std::floor((world_y - config.origin_y) / config.resolution));
  GridPoint out;
  out.x = std::max(0, std::min(config.map_width - 1, grid_x));
  out.y = std::max(0, std::min(config.map_height - 1, config.map_height - 1 - grid_y));
  return out;
}

Pose2D GoalPoseFromCurrent(const Pose2D &pose, const Config &config)
{
  if (config.use_absolute_goal)
  {
    Pose2D goal;
    goal.x = config.goal_x;
    goal.y = config.goal_y;
    goal.theta = config.goal_theta;
    return goal;
  }

  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  Pose2D goal;
  goal.x = pose.x + c * config.goal_distance - s * config.goal_lateral;
  goal.y = pose.y + s * config.goal_distance + c * config.goal_lateral;
  goal.theta = pose.theta;
  return goal;
}

void SendJson(void *ctx, const std::string &id, const json &msg)
{
  const std::string data = msg.dump();
  const int ret = dora_send_output(ctx, const_cast<char *>(id.data()), id.size(), data.data(), data.size());
  if (ret != 0)
  {
    std::cerr << "failed to send " << id << ": " << ret << std::endl;
  }
}

Config LoadConfig()
{
  Config config;
  config.resolution = GetEnvDouble("RESOLUTION", config.resolution);
  config.map_width = GetEnvInt("MAP_WIDTH", config.map_width);
  config.map_height = GetEnvInt("MAP_HEIGHT", config.map_height);
  config.origin_x = GetEnvDouble("ORIGIN_X", config.origin_x);
  config.origin_y = GetEnvDouble("ORIGIN_Y", config.origin_y);
  config.goal_distance = GetEnvDouble("GOAL_DISTANCE", config.goal_distance);
  config.goal_lateral = GetEnvDouble("GOAL_LATERAL", config.goal_lateral);
  config.use_absolute_goal = HasEnv("GOAL_X") && HasEnv("GOAL_Y");
  config.goal_x = GetEnvDouble("GOAL_X", config.goal_x);
  config.goal_y = GetEnvDouble("GOAL_Y", config.goal_y);
  config.goal_theta = GetEnvDouble("GOAL_THETA", config.goal_theta);
  config.send_once = GetEnvBool("SEND_ONCE", config.send_once);
  config.latch_goal = GetEnvBool("LATCH_GOAL", config.latch_goal);
  config.require_localization_match = GetEnvBool("REQUIRE_LOCALIZATION_MATCH", config.require_localization_match);
  config.max_localization_lost_count = GetEnvInt("MAX_LOCALIZATION_LOST_COUNT", config.max_localization_lost_count);
  config.min_send_interval_ms = GetEnvInt("MIN_SEND_INTERVAL_MS", config.min_send_interval_ms);
  return config;
}

bool PoseReadyForGoal(const json &msg, const Config &config)
{
  if (!config.require_localization_match)
  {
    return true;
  }
  const bool localization_mode = msg.value("localization_mode", false);
  if (!localization_mode)
  {
    return true;
  }
  const bool matched = msg.value("localization_matched", false);
  const int lost_count = msg.value("localization_lost_count", config.max_localization_lost_count + 1);
  return matched && lost_count <= config.max_localization_lost_count;
}

} // namespace

int main()
{
  try
  {
    void *ctx = init_dora_context_from_env();
    if (ctx == nullptr)
    {
      throw std::runtime_error("failed to init dora context");
    }

    const Config config = LoadConfig();
    bool sent = false;
    bool goal_latched = false;
    Pose2D latched_goal;
    int64_t last_send_ms = 0;
    bool replan_requested = false;

    std::cout << "Pose goal Dora node" << std::endl;
    std::cout << "goal_mode=" << (config.use_absolute_goal ? "absolute" : "relative")
              << " goal_distance=" << config.goal_distance << " goal_lateral=" << config.goal_lateral
              << " goal_xy=(" << config.goal_x << "," << config.goal_y << ")"
              << " send_once=" << config.send_once
              << " latch_goal=" << config.latch_goal
              << " require_localization_match=" << config.require_localization_match
              << " min_send_interval_ms=" << config.min_send_interval_ms
              << " map=" << config.map_width << "x" << config.map_height
              << " origin=(" << config.origin_x << "," << config.origin_y << ")"
              << " resolution=" << config.resolution << std::endl;

    while (true)
    {
      void *event = dora_next_event(ctx);
      if (event == nullptr)
      {
        break;
      }

      enum DoraEventType ty = read_dora_event_type(event);
      if (ty == DoraEventType_Input)
      {
        char *input_id = nullptr;
        size_t input_id_len = 0;
        char *data = nullptr;
        size_t data_len = 0;
        read_dora_input_id(event, &input_id, &input_id_len);
        read_dora_input_data(event, &data, &data_len);
        const std::string id(input_id, input_id_len);

        if (id == "ReplanRequest")
        {
          replan_requested = true;
          std::cout << "pose_goal received replan request" << std::endl;
        }

        if ((id == "CorrectedPose" || id == "Odometry" || id == "Pose") &&
            (!sent || !config.send_once || replan_requested))
        {
          const int64_t now_ms = NowMs();
          if (!replan_requested && !config.send_once && last_send_ms != 0 &&
              now_ms - last_send_ms < config.min_send_interval_ms)
          {
            free_dora_event(event);
            continue;
          }

          Pose2D pose;
          const json msg = json::parse(std::string(data, data_len));
          if (!PoseReadyForGoal(msg, config))
          {
            std::cout << "pose_goal waiting for localization match" << std::endl;
            free_dora_event(event);
            continue;
          }
          if (!ReadPose(msg, &pose))
          {
            std::cerr << "received pose but could not parse it" << std::endl;
          }
          else
          {
            if (!goal_latched || !config.latch_goal)
            {
              latched_goal = GoalPoseFromCurrent(pose, config);
              goal_latched = true;
            }
            const Pose2D goal_pose = latched_goal;
            const GridPoint start = WorldToPlannerGrid(pose.x, pose.y, config);
            const GridPoint goal = WorldToPlannerGrid(goal_pose.x, goal_pose.y, config);

            SendJson(ctx, "start_point", json{{"x", start.x}, {"y", start.y}});
            SendJson(ctx, "goal_point", json{{"x", goal.x}, {"y", goal.y}});
            SendJson(ctx, "goal_pose", json{{"x", goal_pose.x},
                                             {"y", goal_pose.y},
                                             {"theta", goal_pose.theta},
                                             {"mode", config.use_absolute_goal ? "absolute" : "relative"},
                                             {"start_x", pose.x},
                                             {"start_y", pose.y},
                                             {"start_theta", pose.theta}});
            sent = true;
            replan_requested = false;
            last_send_ms = now_ms;

            std::cout << "pose=(" << pose.x << "," << pose.y << "," << pose.theta << ")"
                      << " start_grid=(" << start.x << "," << start.y << ")"
                      << " goal_world=(" << goal_pose.x << "," << goal_pose.y << ")"
                      << " goal_grid=(" << goal.x << "," << goal.y << ")" << std::endl;
          }
        }
      }
      else if (ty == DoraEventType_Stop)
      {
        free_dora_event(event);
        break;
      }

      free_dora_event(event);
    }

    free_dora_context(ctx);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "pose goal node error: " << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
