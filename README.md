# i2c_sensors 
ROS2 Plugin for the use of I2C sensors.
## Description
This repository contains a plugin for the use of I2C sensors such as the BNO055 IMU (Adafruit Breakout) and the BlueRobotics Bar30 pressure sensor (MS5837-30BA from TE Connectivity).
For use in our systems (ARM Processors), arduino libraries such as the [Adafruit_BNO055](https://github.com/adafruit/Adafruit_BNO055) and [BlueRobotics_MS5837_Library](https://github.com/bluerobotics/BlueRobotics_MS5837_Library) were adapted and new functions were added.

## Requirements
- [ROS2](https://docs.ros.org/en/galactic/Installation.html) - Galactic
- Ubuntu 20.04
- [uuv_msgs](https://github.com/MARUM-MUtTI/uuv_msgs)
- Tested on the NVIDIA Jetson Nano board.

## Installation
- Clone the repositories and compile them:
```
$ source /opt/ros/galactic/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/MARUM-MUtTI/uuv_msgs.git
$ git clone https://github.com/MARUM-MUtTI/i2c_sensors.git
$ cd ..
$ colcon build
```

### Usage
There are two ways to use this package. The first one uses a node written in c++. This version allow to run a node in a separate process with the benefits of process/fault isolation as well as easier debugging. The second one uses [Lifecycle](https://index.ros.org/p/lifecycle/github-ros2-demos/) for node management and [composition](https://docs.ros.org/en/foxy/Tutorials/Composition.html) to increase efficiency. Thus it's possible to have more control over the I2C Bus configuration needed for communication. 


#### C++ node 
- To use the C++ node: 
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 run i2c_sensors i2c_sensors_node
```
#### Lifecycle management 
ROS 2 introduces the concept of managed nodes, also called LifecycleNodes. Managed nodes contain a state machine with a set of predefined states. These states can be changed by invoking a transition id which indicates the succeeding consecutive state.

- The node must first be launched using composition. This allows multiple nodes to be executed in a single process with lower overhead and, optionally, more efficient communication (see [Intra Process Communication](https://docs.ros.org/en/foxy/Tutorials/Intra-Process-Communication.html)). The idea of using composition is to be able to make use of its advantages when integrating more than one node, which is the case of a robotic system.

- Open a terminal to launch a container for using composition:
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 launch i2c_sensors composition.launch.py
```
- Then in a new terminal the initial options can be viewed using Lifecycle. To know the available transitions:
```
$ source /opt/ros/galactic/setup.bash
$ ros2 lifecycle list /i2c_sensors_node

- configure [1]
	Start: unconfigured
	Goal: configuring
- shutdown [5]
	Start: unconfigured
	Goal: shuttingdown
```

- Now it's possible to configure the node:
```
$ ros2 lifecycle set /i2c_sensors_node configure
```
- To know the current transition state use:
```
$ ros2 lifecycle get /i2c_sensors_node

inactive [2]
```

#### Available transitions for this node using Lifecycle management
```
$ ros2 lifecycle set /i2c_sensors_node activate
$ ros2 lifecycle set /i2c_sensors_node deactivate
$ ros2 lifecycle set /i2c_sensors_node cleanup
$ ros2 lifecycle set /i2c_sensors_node shutdown
```

#### IMU Sensor calibration
- To just read the quality of the IMU sensor data readout, open a new terminal:

**Note:** To perform sensor calibration using the **Lifecycle management method** after the sensor has been configured, it's necessary to **deactivate** and clear (**cleanup**) the sensor data, set the parameters (**read_sensor_quality** and/or **write_calibration**) and reconfigure (**configure**).
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 param set /i2c_sensors_node read_sensor_quality true
```

- To write the new calibration data, in the same terminal use:
```
$ ros2 param set /i2c_sensors_node write_calibration true
```

## ROS2 Topics 
- `barometer/data`
- `imu/data`
- `euler/data`

