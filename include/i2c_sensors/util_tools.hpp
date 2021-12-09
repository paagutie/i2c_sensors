#ifndef UTIL_TOOLS_HPP
#define UTIL_TOOLS_HPP

/**
 * util_tools.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       08/07/2020
 */


#include <cstdint>
#include <math.h>

namespace util_tools {

// IMU data structure
struct imu_t {
  float ax, ay, az, gx, gy, gz;
  void clear()
  {
    ax = 0.0;
    ay = 0.0;
    az = 0.0;
    gx = 0.0;
    gy = 0.0;
    gz = 0.0;
  }
};

// Gyro calibration structure
struct point_t {
  float x, y, z;
  void clear()
  {
     x = 0.0;
     y = 0.0;
     z = 0.0;
  }
};

// Attitude structure
struct attitude_t {
  float roll, pitch, yaw;
  void clear()
  {
     roll = 0.0;
     pitch = 0.0;
     yaw = 0.0;
  }
};

struct barometer_t{
  float depth;
  float temperature;
  float pressure;
};



bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}



} //end_namespace

#endif //UTIL_TOOLS_HPP
