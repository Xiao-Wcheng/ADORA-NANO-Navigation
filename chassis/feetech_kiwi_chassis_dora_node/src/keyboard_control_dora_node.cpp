extern "C" {
#include "node_api.h"
}

#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace {

double EnvDouble(const char *name, double fallback) {
  const char *value = std::getenv(name);
  return value && *value ? std::stod(value) : fallback;
}

std::string EnvString(const char *name, const char *fallback) {
  const char *value = std::getenv(name);
  return value && *value ? value : fallback;
}

double NowSeconds() {
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

std::string Twist(const std::string &command, double linear, double angular) {
  double x = 0.0, y = 0.0, yaw = 0.0;
  if (command == "w") x = linear;
  else if (command == "s") x = -linear;
  else if (command == "q") y = linear;
  else if (command == "e") y = -linear;
  else if (command == "a") yaw = angular;
  else if (command == "d") yaw = -angular;

  std::ostringstream out;
  out << "{\"linear\":{\"x\":" << x << ",\"y\":" << y
      << ",\"z\":0},\"angular\":{\"x\":0,\"y\":0,\"z\":" << yaw << "}}";
  return out.str();
}

void Send(void *context, const std::string &payload) {
  std::string id = "CmdVelTwist";
  const int result = dora_send_output(context, id.data(), id.size(), payload.data(), payload.size());
  if (result != 0) std::cerr << "keyboard output failed: " << result << std::endl;
}

bool ValidCommand(const std::string &command) {
  return command == "w" || command == "s" || command == "q" || command == "e" ||
         command == "a" || command == "d" || command == "stop";
}

}  // namespace

int main() {
  void *context = nullptr;
  int fifo_fd = -1;
  try {
    const std::string fifo_path = EnvString("KEYBOARD_FIFO", "/tmp/feetech_kiwi_keyboard_fifo");
    const double linear = EnvDouble("KEY_LINEAR_SPEED", 0.04);
    const double angular = EnvDouble("KEY_ANGULAR_SPEED", 0.10);
    const double timeout = EnvDouble("KEY_COMMAND_TIMEOUT", 0.50);

    if (mkfifo(fifo_path.c_str(), 0600) != 0 && errno != EEXIST) {
      throw std::runtime_error("mkfifo failed: " + std::string(std::strerror(errno)));
    }
    fifo_fd = open(fifo_path.c_str(), O_RDWR | O_NONBLOCK);
    if (fifo_fd < 0) throw std::runtime_error("open FIFO failed: " + std::string(std::strerror(errno)));

    context = init_dora_context_from_env();
    if (!context) throw std::runtime_error("failed to init Dora context");

    std::string buffered;
    std::string latest = "stop";
    double received_at = 0.0;
    std::cout << "keyboard FIFO: " << fifo_path << std::endl;

    while (true) {
      void *event = dora_next_event(context);
      if (!event) break;
      const DoraEventType type = read_dora_event_type(event);
      if (type == DoraEventType_Stop) {
        free_dora_event(event);
        break;
      }
      if (type == DoraEventType_Input) {
        char chunk[4096];
        ssize_t count = 0;
        while ((count = read(fifo_fd, chunk, sizeof(chunk))) > 0) buffered.append(chunk, count);

        std::size_t newline = 0;
        while ((newline = buffered.find('\n')) != std::string::npos) {
          std::string command = buffered.substr(0, newline);
          buffered.erase(0, newline + 1);
          while (!command.empty() && (command.back() == '\r' || command.back() == ' ')) command.pop_back();
          if (ValidCommand(command)) {
            latest = command;
            received_at = NowSeconds();
          }
        }
        const std::string command = (NowSeconds() - received_at <= timeout) ? latest : "stop";
        Send(context, Twist(command, linear, angular));
      }
      free_dora_event(event);
    }
    Send(context, Twist("stop", linear, angular));
    free_dora_context(context);
    close(fifo_fd);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "keyboard Dora node error: " << error.what() << std::endl;
    if (context) free_dora_context(context);
    if (fifo_fd >= 0) close(fifo_fd);
    return 1;
  }
}
