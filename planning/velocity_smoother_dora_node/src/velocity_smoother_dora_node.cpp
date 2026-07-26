extern "C"
{
#include "node_api.h"
}

#include "nav2_velocity_smoother_port/velocity_smoother.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;
using namespace nav2_velocity_smoother_port;

namespace
{

double envDouble(const char * name, double fallback)
{
  const char * value = std::getenv(name);
  return value == nullptr || std::strlen(value) == 0 ? fallback : std::stod(value);
}

int64_t nowMs()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

Config loadConfig()
{
  Config c;
  c.frequency = envDouble("SMOOTHING_FREQUENCY", c.frequency);
  c.max_velocity = {envDouble("MAX_VX", c.max_velocity[0]),
    envDouble("MAX_VY", c.max_velocity[1]), envDouble("MAX_WZ", c.max_velocity[2])};
  c.min_velocity = {envDouble("MIN_VX", c.min_velocity[0]),
    envDouble("MIN_VY", c.min_velocity[1]), envDouble("MIN_WZ", c.min_velocity[2])};
  c.max_accel = {envDouble("MAX_ACCEL_X", c.max_accel[0]),
    envDouble("MAX_ACCEL_Y", c.max_accel[1]), envDouble("MAX_ACCEL_WZ", c.max_accel[2])};
  c.max_decel = {envDouble("MAX_DECEL_X", c.max_decel[0]),
    envDouble("MAX_DECEL_Y", c.max_decel[1]), envDouble("MAX_DECEL_WZ", c.max_decel[2])};
  c.deadband = {envDouble("DEADBAND_VX", c.deadband[0]),
    envDouble("DEADBAND_VY", c.deadband[1]), envDouble("DEADBAND_WZ", c.deadband[2])};
  c.velocity_timeout_ms = static_cast<int64_t>(
    envDouble("VELOCITY_TIMEOUT_MS", static_cast<double>(c.velocity_timeout_ms)));
  return c;
}

Twist readTwist(const json & value)
{
  Twist result;
  if (value.contains("linear")) {
    result.vx = value["linear"].value("x", 0.0);
    result.vy = value["linear"].value("y", 0.0);
  }
  if (value.contains("angular")) {
    result.wz = value["angular"].value("z", 0.0);
  }
  return result;
}

void sendTwist(void * context, const Twist & twist)
{
  const json message = {
    {"linear", {{"x", twist.vx}, {"y", twist.vy}, {"z", 0.0}}},
    {"angular", {{"x", 0.0}, {"y", 0.0}, {"z", twist.wz}}},
    {"state", "SMOOTHED"}};
  const std::string data = message.dump();
  const std::string id = "SmoothedCmdVelTwist";
  dora_send_output(context, const_cast<char *>(id.data()), id.size(), data.data(), data.size());
}

}  // namespace

int main()
{
  void * context = init_dora_context_from_env();
  if (context == nullptr) {
    return 1;
  }
  VelocitySmoother smoother(loadConfig());
  while (true) {
    void * event = dora_next_event(context);
    if (event == nullptr) {
      break;
    }
    const DoraEventType type = read_dora_event_type(event);
    if (type == DoraEventType_Stop) {
      smoother.reset();
      sendTwist(context, {});
      free_dora_event(event);
      break;
    }
    if (type == DoraEventType_Input) {
      char * raw_id = nullptr;
      size_t id_size = 0;
      char * raw_data = nullptr;
      size_t data_size = 0;
      read_dora_input_id(event, &raw_id, &id_size);
      read_dora_input_data(event, &raw_data, &data_size);
      const std::string id(raw_id, id_size);
      try {
        if (id == "CmdVelTwist") {
          smoother.setTarget(
            readTwist(json::parse(std::string(raw_data, data_size))), nowMs());
        }
        if (id == "tick" || id == "CmdVelTwist") {
          sendTwist(context, smoother.update(nowMs()));
        }
      } catch (const std::exception & error) {
        std::cerr << "velocity smoother input error: " << error.what() << std::endl;
        smoother.reset();
        sendTwist(context, {});
      }
    }
    free_dora_event(event);
  }
  return 0;
}
