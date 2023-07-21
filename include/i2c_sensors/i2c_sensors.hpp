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
#include "i2c_sensors/BlueRobotics_KellerLD_Library/include/KellerLD.h"
#include "i2c_sensors/Jetson_INA219/include/INA219.h"
#include "uuv_msgs/msg/barometer.hpp"
#include "uuv_msgs/msg/battery.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


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
    static constexpr float LOW_BATTERY = 12.5f;
    uint8_t ready = 0;
    uint8_t error = 0;

    std::string imu_frame_id;
    std::string mag_frame_id;
    std::string euler_frame_id;
    std::string pressure_frame_id;
    std::string wattmeter_frame_id;
    std::string wattmeter_topic_name;


    I2C_SENSORS();
    ~I2C_SENSORS();

    void read_barometer();
    void read_BNO055();
    void read_ina219();
    void close_i2c();
    void Int2bytes(int value, uint8_t *bytes);

    //Class sensors
    std::unique_ptr<Adafruit_BNO055> bno;
    std::unique_ptr<MS5837> ms5837;
    std::unique_ptr<KellerLD> kellerLD;
    std::unique_ptr<INA219_IIC> ina219;

private:
    int n_writ;
    uint8_t *default_calib;
    char *filename;
    int count_baro = 0;
    std::string i2c_address;
    bool useBNO055;
    bool useMS5837;
    bool useKellerLD;
    bool useINA219;
    bool ms5837WaitingForData = false;
    bool kellerWaitingForData = false;

    // Rates
    int ina219_rate = 10;
    int bno055_rate = 100;
    int kellerLD_rate = 20;
    
    int loops = 0;
    bool writeCalibration = false;
    bool calibrationFlag = false;

    bool barometer_ready_ = false;
    double depth_adjustment_ = 0.0;
    double current_depth = 0.0;
    double depth = 0.0;
    double shunt_resistor_value = 0.0;
    double shunt_volt_offset = 0.0;
    double battery_capacity = 0.0;

    uint8_t *buffer;
    util_tools::imu_t imu_data;
    util_tools::attitude_t euler_data;
    std_msgs::msg::Bool imu_calib_status_msgs;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_parameters_;
    //rclcpp::TimerBase::SharedPtr timer_barometer;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imu_calib_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_calib_pub_;
    rclcpp::Publisher<uuv_msgs::msg::Barometer>::SharedPtr barometer_pub_;
    rclcpp::Publisher<uuv_msgs::msg::Battery>::SharedPtr battery_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;  
    
    std::chrono::high_resolution_clock::time_point last_time;
    std::chrono::high_resolution_clock::time_point last_time_ina219;

    void timerCallback();
    void call_imu_calibration(const std_msgs::msg::Bool::SharedPtr status);

};

} // namespace
#endif //I2C_SENSORS_H
