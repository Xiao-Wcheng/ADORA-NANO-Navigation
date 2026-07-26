extern "C"
{
#include "node_api.h"
}

#include "ord_lidar_driver.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

using json = nlohmann::json;

namespace
{

constexpr double kPi = 3.14159265358979323846;

double DegToRad(double degrees)
{
  return degrees * kPi / 180.0;
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
  std::string value(raw);
  for (char &ch : value)
  {
    ch = static_cast<char>(std::tolower(ch));
  }
  return value == "1" || value == "true" || value == "yes" || value == "on";
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

json HeaderJson(const std::string &frame_id, int seq,
                const std::chrono::system_clock::time_point &stamp)
{
  auto secs = std::chrono::duration_cast<std::chrono::seconds>(stamp.time_since_epoch());
  auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch() - secs);

  json header;
  header["frame_id"] = frame_id;
  header["seq"] = seq;
  header["stamp"]["sec"] = secs.count();
  header["stamp"]["nanosec"] = nanos.count();
  return header;
}

bool InAngleWindow(double angle_deg, double min_deg, double max_deg)
{
  if (min_deg <= max_deg)
  {
    return angle_deg >= min_deg && angle_deg <= max_deg;
  }
  return angle_deg >= min_deg || angle_deg <= max_deg;
}

json BuildLaserScanJson(const full_scan_data_st &scan_data,
                        const std::string &frame_id,
                        int seq,
                        double angle_min_deg,
                        double angle_max_deg,
                        double range_min,
                        double range_max,
                        bool clockwise,
                        double scan_time_s,
                        int target_count,
                        const std::chrono::system_clock::time_point &scan_start_stamp)
{
  target_count = std::max(2, target_count);
  std::vector<double> ranges(static_cast<size_t>(target_count), 0.0);
  std::vector<double> intensities(static_cast<size_t>(target_count), 0.0);
  std::vector<double> best_angle_error(static_cast<size_t>(target_count),
                                       std::numeric_limits<double>::infinity());

  const double configured_span_deg = angle_max_deg >= angle_min_deg
                                       ? angle_max_deg - angle_min_deg
                                       : (360.0 - angle_min_deg) + angle_max_deg;
  const double angle_increment_deg = configured_span_deg / static_cast<double>(target_count);

  for (int i = 0; i < scan_data.vailtidy_point_num; ++i)
  {
    double angle_deg = scan_data.data[i].angle;
    if (!clockwise)
    {
      angle_deg = 360.0 - angle_deg;
      if (angle_deg >= 360.0)
      {
        angle_deg -= 360.0;
      }
    }

    if (!InAngleWindow(angle_deg, angle_min_deg, angle_max_deg))
    {
      continue;
    }

    double relative_deg = angle_deg - angle_min_deg;
    if (relative_deg < 0.0)
    {
      relative_deg += 360.0;
    }
    int index = static_cast<int>(std::llround(relative_deg / angle_increment_deg));
    index %= target_count;
    const double bin_angle_deg = static_cast<double>(index) * angle_increment_deg;
    const double angle_error = std::abs(relative_deg - bin_angle_deg);
    if (angle_error >= best_angle_error[static_cast<size_t>(index)])
    {
      continue;
    }

    const double range_m = static_cast<double>(scan_data.data[i].distance) * 0.001;
    best_angle_error[static_cast<size_t>(index)] = angle_error;
    if (range_m >= range_min && range_m <= range_max)
    {
      ranges[static_cast<size_t>(index)] = range_m;
      intensities[static_cast<size_t>(index)] = static_cast<double>(scan_data.data[i].intensity);
    }
  }

  const double angle_increment = DegToRad(configured_span_deg) / static_cast<double>(target_count);

  json msg;
  msg["header"] = HeaderJson(frame_id, seq, scan_start_stamp);
  msg["angle_min"] = DegToRad(angle_min_deg);
  msg["angle_max"] = DegToRad(angle_min_deg) + angle_increment * static_cast<double>(target_count - 1);
  msg["angle_increment"] = angle_increment;
  msg["scan_time"] = scan_time_s;
  msg["time_increment"] = ranges.empty() ? 0.0 : scan_time_s / static_cast<double>(ranges.size());
  msg["range_min"] = range_min;
  msg["range_max"] = range_max;
  msg["ranges"] = ranges;
  msg["intensities"] = intensities;
  msg["point_count"] = target_count;
  msg["raw_point_count"] = scan_data.vailtidy_point_num;
  msg["rotation_speed_hz"] = scan_data.speed;
  return msg;
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

    const std::string serial_port = GetEnvString("SERIAL_PORT", "/dev/oradar");
    const int serial_baud = GetEnvInt("SERIAL_BAUD", 230400);
    const std::string frame_id = GetEnvString("FRAME_ID", "lidar");
    const std::string output_id = GetEnvString("OUTPUT_ID", "LaserScan");
    const double angle_min_deg = GetEnvDouble("ANGLE_MIN_DEG", 0.0);
    const double angle_max_deg = GetEnvDouble("ANGLE_MAX_DEG", 360.0);
    const double range_min = GetEnvDouble("RANGE_MIN", 0.05);
    const double range_max = GetEnvDouble("RANGE_MAX", 12.0);
    const bool clockwise = GetEnvBool("CLOCKWISE", false);
    const int motor_speed_hz = GetEnvInt("MOTOR_SPEED_HZ", 0);
    const int connect_retry_ms = GetEnvInt("CONNECT_RETRY_MS", 1000);
    const int grab_timeout_ms = GetEnvInt("GRAB_TIMEOUT_MS", 100);
    const int target_point_count = GetEnvInt("TARGET_POINT_COUNT", 450);

    ordlidar::OrdlidarDriver device(ORADAR_TYPE_SERIAL, ORADAR_MS200);
    device.SetSerialPort(serial_port, static_cast<uint32_t>(serial_baud));

    std::cout << "MS200 Dora node" << std::endl;
    std::cout << "serial=" << serial_port << " baud=" << serial_baud << std::endl;
    std::cout << "frame_id=" << frame_id << " output=" << output_id << std::endl;
    std::cout << "angle=[" << angle_min_deg << ", " << angle_max_deg << "] range=["
              << range_min << ", " << range_max << "] clockwise=" << clockwise << std::endl;

    bool connected = false;
    bool speed_configured = false;
    auto last_connect_attempt = std::chrono::steady_clock::time_point{};
    int seq = 0;
    int tick_count = 0;
    int grab_ok_count = 0;
    int grab_empty_count = 0;
    int grab_fail_count = 0;
    int scan_sent_count = 0;

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
        tick_count += 1;
        auto now = std::chrono::steady_clock::now();
        if (!connected &&
            (last_connect_attempt.time_since_epoch().count() == 0 ||
             std::chrono::duration_cast<std::chrono::milliseconds>(now - last_connect_attempt).count() >= connect_retry_ms))
        {
          last_connect_attempt = now;
          if (access(serial_port.c_str(), F_OK) != 0)
          {
            std::cout << "MS200 waiting for " << serial_port << std::endl;
            free_dora_event(event);
            continue;
          }

          connected = device.Connect();
          if (connected)
          {
            std::cout << "MS200 connected" << std::endl;
            if (device.Activate())
            {
              std::cout << "MS200 activated" << std::endl;
            }
            else
            {
              std::cerr << "MS200 activate failed; scan data may be unavailable" << std::endl;
            }
          }
          else
          {
            std::cout << "MS200 waiting for " << serial_port << std::endl;
          }
        }

        if (connected && !speed_configured)
        {
          if (motor_speed_hz > 0)
          {
            const double current_speed = device.GetRotationSpeed();
            if (current_speed < motor_speed_hz * 0.9 || current_speed > motor_speed_hz * 1.1)
            {
              device.SetRotationSpeed(motor_speed_hz);
            }
          }
          speed_configured = true;
        }

        if (connected)
        {
          full_scan_data_st scan_data;
          const auto scan_start_stamp = std::chrono::system_clock::now();
          const auto scan_start = std::chrono::steady_clock::now();
          const bool ok = device.GrabFullScanBlocking(scan_data, grab_timeout_ms);
          const auto scan_end = std::chrono::steady_clock::now();
          if (ok && scan_data.vailtidy_point_num > 0)
          {
            grab_ok_count += 1;
            const double scan_time_s = std::chrono::duration<double>(scan_end - scan_start).count();
            SendJson(ctx, output_id, BuildLaserScanJson(scan_data,
                                                        frame_id,
                                                        seq++,
                                                        angle_min_deg,
                                                        angle_max_deg,
                                                        range_min,
                                                        range_max,
                                                        clockwise,
                                                        scan_time_s,
                                                        target_point_count,
                                                        scan_start_stamp));
            scan_sent_count += 1;
          }
          else if (ok)
          {
            grab_empty_count += 1;
          }
          else
          {
            grab_fail_count += 1;
          }
        }

        if (tick_count % 100 == 0)
        {
          std::cout << "MS200 stats: ticks=" << tick_count
                    << " ok=" << grab_ok_count
                    << " empty=" << grab_empty_count
                    << " fail=" << grab_fail_count
                    << " sent=" << scan_sent_count << std::endl;
        }
      }
      else if (ty == DoraEventType_Stop)
      {
        free_dora_event(event);
        break;
      }

      free_dora_event(event);
    }

    std::cout << "MS200 final stats: ticks=" << tick_count
              << " ok=" << grab_ok_count
              << " empty=" << grab_empty_count
              << " fail=" << grab_fail_count
              << " sent=" << scan_sent_count << std::endl;
    if (connected)
    {
      std::cout << "MS200 stopping measurement" << std::endl;
      device.Deactive();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      std::cout << "MS200 disconnecting" << std::endl;
      device.Disconnect();
      connected = false;
      std::cout << "MS200 stopped" << std::endl;
    }
    free_dora_context(ctx);
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "fatal: " << e.what() << std::endl;
    return 1;
  }
}
