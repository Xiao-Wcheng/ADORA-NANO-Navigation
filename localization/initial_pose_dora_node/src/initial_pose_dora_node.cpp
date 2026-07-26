extern "C" {
#include "node_api.h"
}

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace {

std::string EnvString(const char *name, const char *fallback) {
  const char *value = std::getenv(name);
  return value && *value ? value : fallback;
}

bool ValidPose(const std::string &line);

std::string AutoInitialPose() {
  const std::string x_text = EnvString("INITIAL_POSE_X", "");
  const std::string y_text = EnvString("INITIAL_POSE_Y", "");
  const std::string yaw_text = EnvString("INITIAL_POSE_YAW", "");
  if (x_text.empty() && y_text.empty() && yaw_text.empty()) return {};
  if (x_text.empty() || y_text.empty() || yaw_text.empty()) {
    throw std::runtime_error(
        "INITIAL_POSE_X, INITIAL_POSE_Y and INITIAL_POSE_YAW must be set together");
  }
  const std::string payload =
      "{\"x\":" + x_text + ",\"y\":" + y_text + ",\"yaw\":" + yaw_text + "}";
  if (!ValidPose(payload)) {
    throw std::runtime_error("automatic initial pose must contain finite numbers");
  }
  return payload;
}

bool ReadNumber(const std::string &json, const char *key, double *value) {
  const std::regex pattern(std::string("\\\"") + key +
                           "\\\"\\s*:\\s*([-+]?(?:[0-9]*\\.)?[0-9]+(?:[eE][-+]?[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(json, match, pattern)) return false;
  try {
    *value = std::stod(match[1].str());
  } catch (...) {
    return false;
  }
  return std::isfinite(*value);
}

bool ValidPose(const std::string &line) {
  double x = 0.0, y = 0.0, yaw = 0.0;
  return ReadNumber(line, "x", &x) && ReadNumber(line, "y", &y) &&
         ReadNumber(line, "yaw", &yaw);
}

void Send(void *context, const std::string &payload) {
  std::string id = "InitialPose";
  const int result = dora_send_output(context, id.data(), id.size(), payload.data(), payload.size());
  if (result != 0) throw std::runtime_error("InitialPose output failed: " + std::to_string(result));
}

}  // namespace

int main() {
  void *context = nullptr;
  int fifo_fd = -1;
  try {
    const std::string fifo_path = EnvString("INITIAL_POSE_FIFO", "/tmp/dora_initial_pose_fifo");
    if (mkfifo(fifo_path.c_str(), 0600) != 0 && errno != EEXIST) {
      throw std::runtime_error("mkfifo failed: " + std::string(std::strerror(errno)));
    }
    fifo_fd = open(fifo_path.c_str(), O_RDWR | O_NONBLOCK);
    if (fifo_fd < 0) throw std::runtime_error("open FIFO failed: " + std::string(std::strerror(errno)));

    context = init_dora_context_from_env();
    if (!context) throw std::runtime_error("failed to init Dora context");

    std::string buffered;
    const std::string automatic_pose = AutoInitialPose();
    bool automatic_pose_sent = false;
    std::cout << "initial pose FIFO: " << fifo_path << std::endl;
    while (true) {
      void *event = dora_next_event(context);
      if (!event) break;
      const DoraEventType type = read_dora_event_type(event);
      if (type == DoraEventType_Stop) {
        free_dora_event(event);
        break;
      }
      if (type == DoraEventType_Input) {
        if (!automatic_pose.empty() && !automatic_pose_sent) {
          Send(context, automatic_pose);
          automatic_pose_sent = true;
          std::cout << "Automatic InitialPose sent: " << automatic_pose << std::endl;
        }
        char chunk[4096];
        ssize_t count = 0;
        while ((count = read(fifo_fd, chunk, sizeof(chunk))) > 0) buffered.append(chunk, count);
        std::size_t newline = 0;
        while ((newline = buffered.find('\n')) != std::string::npos) {
          std::string line = buffered.substr(0, newline);
          buffered.erase(0, newline + 1);
          while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
          if (line.empty()) continue;
          if (!ValidPose(line)) {
            std::cerr << "ignored invalid initial pose; expected JSON x/y/yaw" << std::endl;
            continue;
          }
          Send(context, line);
          std::cout << "InitialPose sent: " << line << std::endl;
        }
      }
      free_dora_event(event);
    }

    free_dora_context(context);
    close(fifo_fd);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "initial pose Dora node error: " << error.what() << std::endl;
    if (context) free_dora_context(context);
    if (fifo_fd >= 0) close(fifo_fd);
    return 1;
  }
}
