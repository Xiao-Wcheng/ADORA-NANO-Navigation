extern "C"
{
#include "node_api.h"
}

#include "supervisor_goal.hpp"
#include "supervisor_progress.hpp"

#include <chrono>
#include <cmath>
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

struct SupervisorState
{
  Pose2D pose;
  Pose2D goal;
  bool have_pose{false};
  bool have_goal{false};
  bool localization_mode{false};
  bool localization_matched{true};
  int localization_lost_count{0};
  double localization_score{0.0};
  int localization_matches{0};
  std::string local_mode{"UNKNOWN"};
  std::string local_reason{"unknown"};
  int64_t start_ms{0};
  int64_t last_pose_ms{0};
  int64_t last_local_ms{0};
  int64_t blocked_since_ms{0};
  int64_t last_log_ms{0};
  int64_t last_replan_ms{0};
  int replan_count{0};
  std::string last_nav_state{"STARTING"};
  bool terminal_latched{false};
  std::string terminal_state;
  std::string terminal_reason;
  int64_t goal_start_ms{0};
  int seq{0};
};

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

std::string GetEnvString(const char *name, const std::string &fallback)
{
  const char *raw = std::getenv(name);
  if (raw == nullptr || std::strlen(raw) == 0)
  {
    return fallback;
  }
  return raw;
}

int64_t NowMs()
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

json HeaderJson(const std::string &frame_id, int seq)
{
  auto now = std::chrono::system_clock::now();
  auto secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
  auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() - secs);

  json header;
  header["frame_id"] = frame_id;
  header["seq"] = seq;
  header["stamp"]["sec"] = secs.count();
  header["stamp"]["nanosec"] = nanos.count();
  return header;
}

bool ReadPose(const json &msg, Pose2D *pose)
{
  if (pose == nullptr)
  {
    return false;
  }
  if (msg.contains("pose") && msg["pose"].is_object())
  {
    const auto &p = msg["pose"];
    if (p.contains("x") && p.contains("y"))
    {
      pose->x = p.value("x", 0.0);
      pose->y = p.value("y", 0.0);
      pose->theta = p.value("theta", p.value("yaw", msg.value("theta", msg.value("yaw", 0.0))));
      return true;
    }
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

void SendJson(void *ctx, const std::string &output_id, const json &msg)
{
  const std::string data = msg.dump();
  int ret = dora_send_output(ctx, const_cast<char *>(output_id.data()), output_id.size(), data.data(), data.size());
  if (ret != 0)
  {
    std::cerr << "failed to send " << output_id << ": " << ret << std::endl;
  }
}

struct Config
{
  double goal_x{0.0};
  double goal_y{0.0};
  double goal_tolerance{0.10};
  double yaw_goal_tolerance{0.12};
  int nav_timeout_ms{180000};
  int pose_timeout_ms{1000};
  int blocked_timeout_ms{5000};
  int max_localization_lost_count{12};
  int replan_cooldown_ms{3000};
  int max_replan_attempts{3};
  int progress_timeout_ms{10000};
  double required_movement_radius_m{0.05};
  double required_movement_angle_rad{0.10};
};

Config LoadConfig()
{
  Config config;
  config.goal_x = GetEnvDouble("GOAL_X", config.goal_x);
  config.goal_y = GetEnvDouble("GOAL_Y", config.goal_y);
  config.goal_tolerance = GetEnvDouble("GOAL_TOLERANCE", config.goal_tolerance);
  config.yaw_goal_tolerance = GetEnvDouble("YAW_GOAL_TOLERANCE", config.yaw_goal_tolerance);
  config.nav_timeout_ms = GetEnvInt("NAV_TIMEOUT_MS", config.nav_timeout_ms);
  config.pose_timeout_ms = GetEnvInt("POSE_TIMEOUT_MS", config.pose_timeout_ms);
  config.blocked_timeout_ms = GetEnvInt("BLOCKED_TIMEOUT_MS", config.blocked_timeout_ms);
  config.max_localization_lost_count = GetEnvInt("MAX_LOCALIZATION_LOST_COUNT", config.max_localization_lost_count);
  config.replan_cooldown_ms = GetEnvInt("REPLAN_COOLDOWN_MS", config.replan_cooldown_ms);
  config.max_replan_attempts = GetEnvInt("MAX_REPLAN_ATTEMPTS", config.max_replan_attempts);
  config.progress_timeout_ms = GetEnvInt("PROGRESS_TIMEOUT_MS", config.progress_timeout_ms);
  config.required_movement_radius_m =
      GetEnvDouble("REQUIRED_MOVEMENT_RADIUS", config.required_movement_radius_m);
  config.required_movement_angle_rad =
      GetEnvDouble("REQUIRED_MOVEMENT_ANGLE", config.required_movement_angle_rad);
  return config;
}

double DistanceToGoal(const Pose2D &pose, const Pose2D &goal)
{
  const double dx = goal.x - pose.x;
  const double dy = goal.y - pose.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool IsBlockedMode(const std::string &mode, const std::string &reason)
{
  return mode == "STOP" || mode == "BLOCKED" || reason == "no_valid_trajectory" ||
         reason == "blocked_no_side" || reason == "front_blocked_stop_only" ||
         reason == "front_emergency_stop" || reason == "bypass_no_progress" || reason == "no_scan" ||
         reason == "cmd_timeout";
}

bool IsImmediateBlockedReason(const std::string &reason)
{
  return reason == "no_global_path" || reason == "collision_ahead" ||
         reason == "rotation_blocked";
}

json BuildStatus(const SupervisorState &state, const Config &config, const std::string &nav_state,
                 const std::string &reason, double goal_dist, int64_t now_ms)
{
  json j;
  j["header"] = HeaderJson("nav_supervisor", state.seq);
  j["state"] = nav_state;
  j["reason"] = reason;
  j["goal"]["x"] = state.have_goal ? state.goal.x : config.goal_x;
  j["goal"]["y"] = state.have_goal ? state.goal.y : config.goal_y;
  j["goal"]["theta"] = state.have_goal ? state.goal.theta : 0.0;
  j["have_goal"] = state.have_goal;
  j["goal_tolerance"] = config.goal_tolerance;
  j["yaw_goal_tolerance"] = config.yaw_goal_tolerance;
  j["goal_distance"] = goal_dist;
  j["goal_yaw_error"] = state.have_pose && state.have_goal ?
    CheckSupervisorGoal(
      {state.pose.x, state.pose.y, state.pose.theta},
      {state.goal.x, state.goal.y, state.goal.theta},
      {config.goal_tolerance, config.yaw_goal_tolerance}).yaw_error : -1.0;
  j["pose"]["x"] = state.pose.x;
  j["pose"]["y"] = state.pose.y;
  j["pose"]["theta"] = state.pose.theta;
  j["have_pose"] = state.have_pose;
  j["localization_mode"] = state.localization_mode;
  j["localization_matched"] = state.localization_matched;
  j["localization_lost_count"] = state.localization_lost_count;
  j["localization_score"] = state.localization_score;
  j["localization_matches"] = state.localization_matches;
  j["local_mode"] = state.local_mode;
  j["local_reason"] = state.local_reason;
  j["elapsed_ms"] = now_ms - state.start_ms;
  j["pose_age_ms"] = state.last_pose_ms == 0 ? -1 : now_ms - state.last_pose_ms;
  j["blocked_ms"] = state.blocked_since_ms == 0 ? 0 : now_ms - state.blocked_since_ms;
  j["replan_count"] = state.replan_count;
  return j;
}

bool IsTerminalState(const std::string &nav_state)
{
  return nav_state == "REACHED" || nav_state == "TIMEOUT" || nav_state == "LOCALIZATION_LOST";
}

json BuildZeroCommand(int seq, const std::string &reason)
{
  json j;
  j["header"] = HeaderJson("nav_supervisor", seq);
  j["linear"] = {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}};
  j["angular"] = {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}};
  j["state"] = "safety_stop";
  j["reason"] = reason;
  return j;
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

    const std::string output_id = GetEnvString("OUTPUT_ID", "NavigationStatus");
    const std::string replan_output_id = GetEnvString("REPLAN_OUTPUT_ID", "ReplanRequest");
    const std::string cmd_output_id = GetEnvString("CMD_OUTPUT_ID", "SafeCmdVelTwist");
    const Config config = LoadConfig();
    SupervisorState state;
    SupervisorProgress goal_progress(
      {config.required_movement_radius_m, config.required_movement_angle_rad,
       config.progress_timeout_ms});
    state.start_ms = NowMs();
    state.goal.x = config.goal_x;
    state.goal.y = config.goal_y;
    state.goal.theta = 0.0;

    std::cout << "Navigation supervisor Dora node" << std::endl;
    std::cout << "goal=(" << config.goal_x << "," << config.goal_y << ")"
              << " tolerance=" << config.goal_tolerance
              << " yaw_tolerance=" << config.yaw_goal_tolerance
              << " nav_timeout_ms=" << config.nav_timeout_ms
              << " blocked_timeout_ms=" << config.blocked_timeout_ms << std::endl;

    while (true)
    {
      void *event = dora_next_event(ctx);
      if (event == nullptr)
      {
        break;
      }

      enum DoraEventType ty = read_dora_event_type(event);
      if (ty == DoraEventType_Stop)
      {
        free_dora_event(event);
        break;
      }
      if (ty != DoraEventType_Input)
      {
        free_dora_event(event);
        continue;
      }

      char *input_id = nullptr;
      size_t input_id_len = 0;
      char *data = nullptr;
      size_t data_len = 0;
      read_dora_input_id(event, &input_id, &input_id_len);
      read_dora_input_data(event, &data, &data_len);
      const std::string id(input_id, input_id_len);
      const int64_t now_ms = NowMs();
      json local_command;
      bool have_local_command = false;

      try
      {
        if (id == "CorrectedPose" || id == "Pose" || id == "Odometry")
        {
          const json msg = json::parse(std::string(data, data_len));
          Pose2D pose;
          if (ReadPose(msg, &pose))
          {
            state.pose = pose;
            state.have_pose = true;
            state.last_pose_ms = now_ms;
            state.localization_mode = msg.value("localization_mode", state.localization_mode);
            state.localization_matched = msg.value("localization_matched", state.localization_matched);
            state.localization_lost_count = msg.value("localization_lost_count", state.localization_lost_count);
            state.localization_score = msg.value("localization_score", state.localization_score);
            state.localization_matches = msg.value("localization_matches", state.localization_matches);
          }
        }
        else if (id == "LocalPlannerStatus")
        {
          const json msg = json::parse(std::string(data, data_len));
          state.local_mode = msg.value("mode", "UNKNOWN");
          state.local_reason = msg.value("reason", "unknown");
          state.last_local_ms = now_ms;
        }
        else if (id == "GoalPose")
        {
          Pose2D goal;
          if (ReadPose(json::parse(std::string(data, data_len)), &goal))
          {
            const bool goal_changed = !state.have_goal || DistanceToGoal(state.goal, goal) > 1e-6 ||
                                      std::abs(state.goal.theta - goal.theta) > 1e-6;
            state.goal = goal;
            state.have_goal = true;
            if (goal_changed)
            {
              state.terminal_latched = false;
              state.terminal_state.clear();
              state.terminal_reason.clear();
              state.goal_start_ms = now_ms;
              state.blocked_since_ms = 0;
              state.replan_count = 0;
              state.last_replan_ms = 0;
              if (state.have_pose)
              {
                goal_progress.reset(
                  {state.pose.x, state.pose.y, state.pose.theta}, now_ms);
              }
              else
              {
                goal_progress.clear();
              }
            }
          }
        }
        else if (id == "CmdVelTwist")
        {
          local_command = json::parse(std::string(data, data_len));
          have_local_command = true;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "failed to parse " << id << ": " << e.what() << std::endl;
      }

      if (id == "tick" || id == "CorrectedPose" || id == "LocalPlannerStatus" ||
          id == "GoalPose" || id == "CmdVelTwist")
      {
        const Pose2D active_goal = state.have_goal ? state.goal : Pose2D{config.goal_x, config.goal_y, 0.0};
        const double goal_dist = (state.have_pose && state.have_goal) ? DistanceToGoal(state.pose, active_goal) : -1.0;
        const SupervisorGoalResult goal_check = CheckSupervisorGoal(
          {state.pose.x, state.pose.y, state.pose.theta},
          {active_goal.x, active_goal.y, active_goal.theta},
          {config.goal_tolerance, config.yaw_goal_tolerance});
        std::string nav_state = "RUNNING";
        std::string reason = "running";

        if (!state.have_pose || now_ms - state.last_pose_ms > config.pose_timeout_ms)
        {
          nav_state = "POSE_TIMEOUT";
          reason = "pose_timeout";
        }
        else if (!state.have_goal)
        {
          nav_state = "WAIT_GOAL";
          reason = "waiting_for_goal";
        }
        else if (goal_dist >= 0.0 && goal_check.reached)
        {
          nav_state = "REACHED";
          reason = "position_and_yaw_reached";
        }
        else if (state.goal_start_ms > 0 && now_ms - state.goal_start_ms > config.nav_timeout_ms)
        {
          nav_state = "TIMEOUT";
          reason = "nav_timeout";
        }

        if (nav_state == "RUNNING" &&
            state.localization_mode &&
            state.localization_lost_count >= config.max_localization_lost_count)
        {
          nav_state = "LOCALIZATION_LOST";
          reason = "localization_lost";
        }

        if (nav_state == "RUNNING")
        {
          if (IsImmediateBlockedReason(state.local_reason))
          {
            nav_state = "BLOCKED";
            reason = state.local_reason;
          }
          else if (IsBlockedMode(state.local_mode, state.local_reason))
          {
            if (state.blocked_since_ms == 0)
            {
              state.blocked_since_ms = now_ms;
            }
            if (now_ms - state.blocked_since_ms > config.blocked_timeout_ms)
            {
              nav_state = "BLOCKED";
              reason = "blocked_timeout";
            }
          }
          else
          {
            state.blocked_since_ms = 0;
          }
        }

        if (nav_state == "RUNNING" && goal_progress.stalled(
            {state.pose.x, state.pose.y, state.pose.theta}, now_ms))
        {
          nav_state = "BLOCKED";
          reason = "stalled_no_progress";
        }

        if (state.terminal_latched)
        {
          nav_state = state.terminal_state;
          reason = state.terminal_reason;
        }
        else if (IsTerminalState(nav_state))
        {
          state.terminal_latched = true;
          state.terminal_state = nav_state;
          state.terminal_reason = reason;
        }

        const bool motion_allowed =
          ShouldForwardLocalCommand(nav_state, state.terminal_latched);
        if (have_local_command && motion_allowed)
        {
          local_command["supervisor_state"] = nav_state;
          SendJson(ctx, cmd_output_id, local_command);
        }
        else if (!motion_allowed)
        {
          SendJson(ctx, cmd_output_id, BuildZeroCommand(state.seq, reason));
        }
        SendJson(ctx, output_id, BuildStatus(state, config, nav_state, reason, goal_dist, now_ms));
        const bool replan_due = nav_state == "BLOCKED" &&
                                state.replan_count < config.max_replan_attempts &&
                                (state.last_nav_state != "BLOCKED" || state.last_replan_ms == 0 ||
                                 now_ms - state.last_replan_ms >= config.replan_cooldown_ms);
        if (replan_due)
        {
          SendJson(ctx, replan_output_id,
                   json{{"header", HeaderJson("nav_supervisor", state.seq)},
                        {"reason", reason},
                        {"goal", {{"x", active_goal.x}, {"y", active_goal.y}, {"theta", active_goal.theta}}},
                        {"pose", {{"x", state.pose.x}, {"y", state.pose.y}, {"theta", state.pose.theta}}}});
          std::cout << "requested global replan after blocked timeout" << std::endl;
          state.last_replan_ms = now_ms;
          state.replan_count += 1;
        }
        state.last_nav_state = nav_state;

        if (now_ms - state.last_log_ms >= 500)
        {
          state.last_log_ms = now_ms;
          std::cout << "nav state=" << nav_state
                    << " reason=" << reason
                    << " goal_dist=" << goal_dist
                    << " local=" << state.local_mode << "/" << state.local_reason
                    << " loc_lost=" << state.localization_lost_count
                    << " pose=(" << state.pose.x << "," << state.pose.y << "," << state.pose.theta << ")"
                    << std::endl;
        }

        state.seq += 1;
      }

      free_dora_event(event);
    }

    free_dora_context(ctx);
  }
  catch (const std::exception &e)
  {
    std::cerr << "fatal: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
