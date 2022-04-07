#ifndef I2C_SENSORS_HPP
#define I2C_SENSORS_HPP

/**
 * i2c_sensors.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       18/03/2021
 */

#include <cstdint>

#include <iostream>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// ROS 2 Headers
#include <chrono>
#include <memory>
#include <functional>


#include "rclcpp/rclcpp.hpp"
#include "i2c_sensors/util_tools.hpp"
#include "i2c_sensors/BNO055.h"
#include "i2c_sensors/MS5837.hpp"
#include "uuv_msgs/msg/barometer.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

#define ARDUINO_ADDR         0x40  
#define ARDUINO_RESET        0x1E

#define BYTES_TO_SEND 1

#define BYTES_TO_RECEIVE_IMU  24
#define BYTES_BY_DATA_IMU      4
#define NUM_DATA_IMU           6

#define BYTES_TO_RECEIVE_EULER  12
#define BYTES_BY_DATA_EULER      4
#define NUM_DATA_EULER           3

#define CMD_BAROMETER    0x01
#define CMD_BNO055       0x02


namespace navigation_module{



class I2C_SENSORS: public rclcpp::Node
{
public:
    static constexpr float Pa = 100.0f;
    static constexpr float LOW_BATTERY = 11.5f;
    uint8_t ready = 0;
    uint8_t error = 0;


    I2C_SENSORS();
    ~I2C_SENSORS();

    void read_barometer();
    void read_BNO055();
    void close_i2c();
    void Int2bytes(int value, uint8_t *bytes);

    //Class sensors
    Adafruit_BNO055 bno;
    MS5837 ms5837;

private:
    int n_writ;
    uint8_t *calib_data;
    char *filename;
    int count_baro = 0;
    std::string i2c_address;
    bool useBNO055;
    bool useMS5837;
    
    int loops = 0;
    bool readSensorQuality = false;
    bool writeCalibration = false;

    bool barometer_ready_ = false;
    double depth_adjustment_ = 0.0;
    double current_depth = 0.0;
    double depth = 0.0;



    uint8_t *buffer;
    util_tools::imu_t imu_data;
    util_tools::attitude_t euler_data;
    

    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_parameters_;
    //rclcpp::TimerBase::SharedPtr timer_barometer;
    //rclcpp::Subscription<rov_msgs::msg::Control>::SharedPtr sub_control_;
    rclcpp::Publisher<uuv_msgs::msg::Barometer>::SharedPtr barometer_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_; 
    
    std::chrono::high_resolution_clock::time_point last_time;

    void timerCallback();
    void get_parameters();
    //void timerCallbackBarometer();
    //void topic_callback(const rov_msgs::msg::Control::SharedPtr msg);


};

} // namespace
#endif //I2C_SENSORS_H
