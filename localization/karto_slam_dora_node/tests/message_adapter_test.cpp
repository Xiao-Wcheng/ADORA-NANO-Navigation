#include "karto_dora/message_adapter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  const std::string odom = R"({"header":{"stamp":{"sec":10,"nanosec":500000000}},"pose":{"pose":{"position":{"x":1.0,"y":2.0},"orientation":{"z":0.7071067811865475,"w":0.7071067811865476}}},"twist":{"twist":{"linear":{"x":0.2,"y":0.1},"angular":{"z":0.3}}}})";
  const auto parsed_odom = karto_dora::ParseOdometry(odom);
  assert(std::abs(parsed_odom.stamp - 10.5) < 1e-9);
  assert(std::abs(parsed_odom.pose.yaw - 1.5707963267948966) < 1e-9);

  const std::string scan = R"({"header":{"frame_id":"lidar","stamp":{"sec":20,"nanosec":0}},"angle_min":0.0,"angle_max":3.0,"angle_increment":1.0,"scan_time":0.4,"time_increment":0.1,"range_min":0.1,"range_max":12.0,"ranges":[1.0,2.0,0.0,4.0]})";
  const auto parsed_scan = karto_dora::ParseLaserScan(scan);
  assert(parsed_scan.ranges.size() == 4);
  assert(std::abs(parsed_scan.start_stamp - 20.0) < 1e-9);
  assert(std::abs(parsed_scan.end_stamp - 20.4) < 1e-9);

  bool rejected = false;
  try {
    (void)karto_dora::ParseLaserScan(R"({"ranges":[1.0]})");
  } catch (const karto_dora::MessageError &) {
    rejected = true;
  }
  assert(rejected);
  const auto initial = karto_dora::ParseInitialPose(R"({"x":0.5,"y":-0.2,"yaw":1.57})");
  assert(std::abs(initial.x - 0.5) < 1e-9);
  assert(std::abs(initial.y + 0.2) < 1e-9);
  assert(std::abs(initial.yaw - 1.57) < 1e-9);
  rejected = false;
  try {
    (void)karto_dora::ParseInitialPose(R"({"x":0.0,"y":0.0,"yaw":"bad"})");
  } catch (const karto_dora::MessageError &) {
    rejected = true;
  }
  assert(rejected);
  std::cout << "message_adapter_test PASS\n";
}
