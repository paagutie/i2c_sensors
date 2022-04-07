#ifndef COMPOSITION__I2C_COMPONENT_HPP_
#define COMPOSITION__I2C_COMPONENT_HPP_


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
#include <memory>
#include <functional>

#include <chrono>
#include "i2c_sensors/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"


#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"


#include "i2c_sensors/util_tools.hpp"
#include "i2c_sensors/BNO055.h"
#include "i2c_sensors/MS5837.hpp"
#include "uuv_msgs/msg/barometer.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"


#define ARDUINO_ADDR            0x40  
#define ARDUINO_RESET           0x1E
#define BYTES_TO_SEND           1
#define BYTES_TO_RECEIVE_IMU    24
#define BYTES_BY_DATA_IMU       4
#define NUM_DATA_IMU            6
#define BYTES_TO_RECEIVE_EULER  12
#define BYTES_BY_DATA_EULER     4
#define NUM_DATA_EULER          3
#define CMD_BAROMETER           0x01
#define CMD_BNO055              0x02


namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace composition
{

class LifecycleI2CSensors : public rclcpp_lifecycle::LifecycleNode
{
public:
  static constexpr float Pa = 100.0f;
  static constexpr float LOW_BATTERY = 11.5f;
  uint8_t ready = 0;
  uint8_t error = 0;
    
  COMPOSITION_PUBLIC
  explicit LifecycleI2CSensors(const rclcpp::NodeOptions & options);
  ~LifecycleI2CSensors();
  
  //Class sensors
  Adafruit_BNO055 bno;
  MS5837 ms5837;
  
  /// Transition callback for state configuring
  /**
   * on_configure callback is being called when the lifecycle node
   * enters the "configuring" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "unconfigured".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  
  
  /// Transition callback for state activating
  /**
   * on_activate callback is being called when the lifecycle node
   * enters the "activating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "active" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "active"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  
  
  /// Transition callback for state deactivating
  /**
   * on_deactivate callback is being called when the lifecycle node
   * enters the "deactivating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "active".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "active"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  
  /// Transition callback for state cleaningup
  /**
   * on_cleanup callback is being called when the lifecycle node
   * enters the "cleaningup" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "unconfigured" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  
  /// Transition callback for state shutting down
  /**
   * on_shutdown callback is being called when the lifecycle node
   * enters the "shuttingdown" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "finalized" state or stays
   * in its current state.
   * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
   * TRANSITION_CALLBACK_FAILURE transitions to current state
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  
  //ROS2 Parameters callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  
  

protected:
  void on_timer();

private:
  size_t count_;
  int n_writ;
  uint8_t *calib_data;
  char *filename;
  int count_baro = 0;
    
  int loops = 0;
  bool writeCalibration = false;

  bool barometer_ready_ = false;
  double depth_adjustment_ = 0.0;
  double current_depth = 0.0;
  double depth = 0.0;

  std::string i2c_address;
  bool useBNO055;
  bool useMS5837;


  uint8_t *buffer;
  util_tools::imu_t imu_data;
  util_tools::attitude_t euler_data;


    
  std::chrono::steady_clock::time_point first_time_loss;
  std::chrono::steady_clock::time_point first_time_error;
  
  std::chrono::high_resolution_clock::time_point last_time;
  std::chrono::steady_clock::time_point last_time_parameters;
    

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<uuv_msgs::msg::Barometer>> barometer_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>> euler_pub_;
  
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  //rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::shared_ptr<rclcpp::TimerBase> timer_parameters_;
  
  bool read_barometer();
  bool read_BNO055();
  void close_i2c();
  void Int2bytes(int value, uint8_t *bytes);
  void set_new_calibration_parameters();
  
  void set_parameter(bool &value, bool &out, 
       rcl_interfaces::msg::SetParametersResult &result);
  
};

}  // namespace composition

#endif  // COMPOSITION__I2C_COMPONENT_HPP_
