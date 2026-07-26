#include "ord_lidar_driver.h"

#include <cstdlib>
#include <iostream>

int main()
{
  if (ordlidar::BoundedSerialReadSize(64, 1024) != 64) {
    std::cerr << "small serial read was changed\n";
    return 1;
  }
  if (ordlidar::BoundedSerialReadSize(4096, 1024) != 1024) {
    std::cerr << "oversized serial read can overflow stack buffer\n";
    return 1;
  }
  if (ordlidar::BoundedSerialReadSize(1, 0) != 0) {
    std::cerr << "zero-capacity read was not rejected\n";
    return 1;
  }
  std::cout << "serial_read_bounds_test PASS\n";
}
