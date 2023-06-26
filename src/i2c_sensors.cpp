/**
 * i2c_sensors.cpp
 *
 * @author     Pablo Gutiérrez
 * @date       18/03/2021
 */

#include "i2c_sensors/i2c_sensors.hpp"


namespace navigation_module{

int file;

I2C_SENSORS::I2C_SENSORS():
Node("i2c_sensors_node")   
{
    default_calib = new uint8_t[22];
    filename = new char[15];

    //ROS2 Parameters
    this->declare_parameter<std::string>("i2c_bus_address", "/dev/i2c-1");
    i2c_address = this->get_parameter("i2c_bus_address").as_string();
    RCLCPP_INFO(get_logger(), "I2C_BUS_ADDRESS: '%s'", i2c_address.c_str());

    this->declare_parameter<bool>("wattmeter.use_ina219", false);
    this->declare_parameter<double>("wattmeter.shunt_resistor_value", 0.01);
    this->declare_parameter<double>("wattmeter.shunt_volt_offset", -0.03);
    this->declare_parameter<double>("wattmeter.battery_capacity", 37.2);
    this->declare_parameter<std::string>("wattmeter.frame_id", "base_link/wattmeter0");
    this->declare_parameter<std::string>("wattmeter.topic_name", "battery/data");
    this->declare_parameter<bool>("pressure_sensor.use_ms5837", false);
    this->declare_parameter<bool>("pressure_sensor.use_kellerLD", true);
    this->declare_parameter<std::string>("pressure_sensor.frame_id", "base_link/depth");
    this->declare_parameter<bool>("bno055_sensor.use_bno055", true);
    this->declare_parameter<std::string>("bno055_sensor.imu_frame_id", "base_link/imu_sensor");
    this->declare_parameter<std::string>("bno055_sensor.mag_frame_id", "base_link/magnetometer");
    this->declare_parameter<std::string>("bno055_sensor.euler_frame_id", "base_link/euler");

    useINA219 = this->get_parameter("wattmeter.use_ina219").as_bool();
    shunt_resistor_value = this->get_parameter("wattmeter.shunt_resistor_value").as_double();
    shunt_volt_offset = this->get_parameter("wattmeter.shunt_volt_offset").as_double();
    wattmeter_frame_id = this->get_parameter("wattmeter.frame_id").as_string();
    wattmeter_topic_name = this->get_parameter("wattmeter.topic_name").as_string();
    battery_capacity = this->get_parameter("wattmeter.battery_capacity").as_double();
    useMS5837 = this->get_parameter("pressure_sensor.use_ms5837").as_bool();
    useKellerLD = this->get_parameter("pressure_sensor.use_kellerLD").as_bool();
    pressure_frame_id = this->get_parameter("pressure_sensor.frame_id").as_string();
    useBNO055 = this->get_parameter("bno055_sensor.use_bno055").as_bool();
    imu_frame_id = this->get_parameter("bno055_sensor.imu_frame_id").as_string();
    mag_frame_id = this->get_parameter("bno055_sensor.mag_frame_id").as_string();
    euler_frame_id = this->get_parameter("bno055_sensor.euler_frame_id").as_string();

    //IMU BNO055 Calibration data
    /*
    --- Accelerometer Offset registers ---
    ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,

    --- Magnetometer Offset registers ---
    MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
    MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,

    --- Gyroscope Offset registers ---
    GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
    GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,

    --- Radius registers ---
    ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
    ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
    MAG_RADIUS_LSB_ADDR                                     = 0X69,
    MAG_RADIUS_MSB_ADDR                                     = 0X6A
    */


    this->declare_parameter("imu_sensor.bno055_calib_params", std::vector<long int>({234, 255, 221, 255, 217, 255, //--- Accelerometer Offset registers ---
                                                                                     154, 255, 125, 1,   159, 255, //--- Magnetometer Offset registers ---
                                                                                     0,   0,   254, 255, 0,   0,   //--- Gyroscope Offset registers ---
                                                                                     232, 3,   86,  3}));          //--- Radius registers ---

    std::vector<long int> calibration_params = this->get_parameter("imu_sensor.bno055_calib_params").as_integer_array();
    for(uint8_t i=0; i< (uint8_t)calibration_params.size(); i++)
        default_calib[i] = (uint8_t)calibration_params[i];

    sprintf(filename,(char*)i2c_address.c_str());
    file = open(filename, O_RDWR);
 
    //ROS2 QoS
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth
            ),
        qos_profile);

    if(useMS5837)
    {
        ms5837 = std::make_unique<MS5837>();

        RCLCPP_INFO(get_logger(), "MS5837 pressure sensor has been selected.");
        //int addr = 0x76; The I2C address
        if (ioctl(file, I2C_SLAVE, MS5837_ADDR) < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to MS5837.");
            //ERROR HANDLING; you can check errno to see what went wrong 
            exit(1);
        }

        ms5837->setPort(file);
        
        rclcpp::Time time_begin = this->now();
        while (!ms5837->init()) {
            RCLCPP_INFO(this->get_logger(), "Are SDA/SCL connected correctly?");
            RCLCPP_INFO(this->get_logger(), "BlueRobotics Bar30: White=SDA, Green=SCL");

            rclcpp::Time time_end = this->now();
            rclcpp::Duration duration = time_end - time_begin;
            if(duration.seconds() >= 30.0)
            {
                RCLCPP_ERROR(this->get_logger(), "Error when initializing the MS5837 sensor.");
                exit(2);
            }
            usleep(2000000);
        }

        barometer_pub_ = this->create_publisher<uuv_msgs::msg::Barometer>("barometer/data", qos);
    }
    else if(useKellerLD)
    {
        RCLCPP_INFO(get_logger(), "KellerLD pressure sensor has been selected.");
        if (ioctl(file, I2C_SLAVE, LD_ADDR) < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to KellerLD.");
            exit(1);
        }

        kellerLD = std::make_unique<KellerLD>();
        kellerLD->setI2CPort(file);
        kellerLD->init();
        
        rclcpp::Time time_begin = this->now();
        while (!kellerLD->isInitialized()) {
            RCLCPP_INFO(this->get_logger(), "Are SDA/SCL connected correctly?");
            RCLCPP_INFO(this->get_logger(), "BlueRobotics Bar100: White=SDA, Green=SCL");

            rclcpp::Time time_end = this->now();
            rclcpp::Duration duration = time_end - time_begin;
            if(duration.seconds() >= 30.0)
            {
                RCLCPP_ERROR(this->get_logger(), "Error when initializing the Bar100 sensor.");
                exit(2);
            }

            kellerLD->init();
            usleep(2000000);
        }

        barometer_pub_ = this->create_publisher<uuv_msgs::msg::Barometer>("barometer/data", qos);

    }

    if(useBNO055)
    {
        bno = std::make_unique<Adafruit_BNO055>();

        RCLCPP_INFO(get_logger(), "The IMU sensor has been selected.");
        //int addr = 0x28; /* The I2C address */
        if (ioctl(file, I2C_SLAVE, BNO055_ADDRESS_A) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to BNO055!");
            exit(1);
        }

        bno->setI2CPort(file);

        rclcpp::Time time_begin = this->now();
        while(!bno->begin(bno->OPERATION_MODE_NDOF)){
            RCLCPP_INFO(this->get_logger(), "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");

            rclcpp::Time time_end = this->now();
            rclcpp::Duration duration = time_end - time_begin;
            if(duration.seconds() >= 30.0)
            {
              RCLCPP_ERROR(this->get_logger(), "Error when initializing the BNO055 sensor.");
              exit(2);
            }

            usleep(2000000);
        }
        
        bno->setExtCrystalUse(true);
        //Set default calibration
        bno->setSensorOffsets(default_calib);
        usleep(2000000); // hold on

        euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("euler/data", qos);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag/data", qos);
        imu_calib_sub_ = this->create_subscription<std_msgs::msg::Bool>("imu/calib/cmd", 5, std::bind(&I2C_SENSORS::call_imu_calibration, this, _1));
        imu_calib_pub_ = this->create_publisher<std_msgs::msg::Bool>("imu/calib/status", 5);
    }

    if(useINA219)
    {
        ina219 = std::make_unique<INA219_IIC>(file, INA219_I2C_ADDRESS4);
        RCLCPP_INFO(get_logger(), "The INA219 sensor has been selected.");

        rclcpp::Time time_begin = this->now();
        while(!ina219->scan()){
            RCLCPP_INFO(this->get_logger(), "Ooops, no INA219 detected ... Check your wiring or I2C ADDR!");

            rclcpp::Time time_end = this->now();
            rclcpp::Duration duration = time_end - time_begin;
            if(duration.seconds() >= 30.0)
            {
              RCLCPP_ERROR(this->get_logger(), "Error when initializing the IN219 sensor.");
              exit(2);
            }

            usleep(2000000);
        }

        ina219->setShuntSizeInOhms(shunt_resistor_value);
        ina219->setShuntVoltOffset_mV(shunt_volt_offset);
        ina219->setCalibration_32V_2A();

        battery_pub_ = this->create_publisher<uuv_msgs::msg::Battery>("main/enclosure/battery/data", qos);
    }

    //ROS2 timers
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&I2C_SENSORS::timerCallback, this)); //2hz

    last_time_ina219 = last_time = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "I2C_SENSORS - Success!");
}

I2C_SENSORS::~I2C_SENSORS() {

    delete [] filename;
    delete [] buffer;
    delete [] default_calib;
}

void I2C_SENSORS::call_imu_calibration(const std_msgs::msg::Bool::SharedPtr status)
{
    if(useBNO055)
    {
        writeCalibration = status->data;
        if(writeCalibration && !calibrationFlag)
        {
            calibrationFlag = true;
            RCLCPP_INFO(this->get_logger(), "The calibration process has been selected!");
            imu_calib_status_msgs.data = true;
            imu_calib_pub_->publish(imu_calib_status_msgs);

        }
        else if(writeCalibration && calibrationFlag)
        {
            RCLCPP_INFO(this->get_logger(), "The calibration process has been cancelled!");
            calibrationFlag = false;
            imu_calib_status_msgs.data = false;
            imu_calib_pub_->publish(imu_calib_status_msgs);
        }
    }
}


void I2C_SENSORS::close_i2c()
{
   close(file);
}


void I2C_SENSORS::Int2bytes(int value, uint8_t *bytes)
{
    bytes[0] = (value >> 24) & 0xFF;
    bytes[1] = (value >> 16) & 0xFF;
    bytes[2] = (value >> 8) & 0xFF;
    bytes[3] = value & 0xFF;
}

void I2C_SENSORS::timerCallback()
{
    if(useMS5837 || useKellerLD)
        this->read_barometer();
    if(useBNO055)
    {
        this->read_BNO055();
        //Read sensor quality data or setup calibration
        if(calibrationFlag)
        {
            //int addr = 0x28; /* The I2C address */
            if (ioctl(file, I2C_SLAVE, BNO055_ADDRESS_A) < 0)
                RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to BNO055!");

            uint8_t system, gyro, accel, mag = 0;
            bno->getCalibration(&system, &gyro, &accel, &mag);
            //bno.getCalibationStatus(&system, &gyro, &accel, &mag);
            std::cout<< "CALIBRATION: Sys=" << (int)system << " Gyro=" << (int)gyro
                << " Accel=" << (int)accel << " Mag=" << (int)mag << std::endl;

            bno->getSensorOffsets(default_calib);


            if((int)mag == 3 && (int)gyro ==3 && (int)accel ==3){
                for(uint8_t i=0; i<NUM_BNO055_OFFSET_REGISTERS; i++)
                    printf("Hex: %d\n", default_calib[i]);
                bno->setSensorOffsets(default_calib);
                RCLCPP_INFO(this->get_logger(), "The BNO055 Sensor has been calibrated!");
                calibrationFlag = false;
                imu_calib_status_msgs.data = false;
                imu_calib_pub_->publish(imu_calib_status_msgs);
            }
        }

    }
    if(useINA219)
    {
        this->read_ina219();
    }
}



void I2C_SENSORS::read_barometer()
{
    if(!calibrationFlag)
    {
        std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
        if(useMS5837)
        {
            //int addr = 0x76; The I2C address
            if (ioctl(file, I2C_SLAVE, MS5837_ADDR) < 0) 
                RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to MS5837.");
            else
            {
                if(!ms5837WaitingForData)
                {
                    ms5837->read_sensor_part1();
                    ms5837WaitingForData = true;
                    last_time = current_time;
                }

                double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
                if(dt >= 20)
                {
                    loops++;
                    if(loops == 1)
                        ms5837->read_sensor_part2();  
                    else if(loops == 2){
                        ms5837->read_sensor_part3();
                        ms5837->pressure();

                        depth = (double)ms5837->depth();

                        if(!barometer_ready_)
                        {
                            if(depth < 0)
                                depth *= -1;
                            depth_adjustment_ = depth;
                            barometer_ready_ = true;
                            RCLCPP_INFO(this->get_logger(), "Barometer ready, depth adjustment: '%6.2F'", depth_adjustment_);
                        }

                        if(depth < 0)
                            current_depth = depth + depth_adjustment_;
                        else if(depth >= 0.0)
                            current_depth = depth - depth_adjustment_;

                        //RCLCPP_INFO(this->get_logger(), "Current depth: '%6.2F'", depth);

                        //Publish barometer data
                        uuv_msgs::msg::Barometer baro_msg;
                        baro_msg.header.stamp = Node::now();
                        baro_msg.header.frame_id = pressure_frame_id;
                        baro_msg.depth = current_depth;
                        baro_msg.temperature = (double)ms5837->temperature();
                        baro_msg.pressure = (double)ms5837->pressure();

                        barometer_pub_->publish(baro_msg);
                        ms5837WaitingForData = false;
                        loops = 0;
                    }
                    last_time = current_time;   
                }
            }
        }
        else if(useKellerLD)
        {
            if (ioctl(file, I2C_SLAVE, LD_ADDR) < 0) 
                RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to KellerLD.");
            else
            {
                if(!kellerWaitingForData)
                {
                    if(kellerLD->read_request() != 99)
                        kellerWaitingForData = true;
                    else
                        RCLCPP_ERROR(this->get_logger(), "KellerLD: Failed to write 1 byte to the i2c bus.");

                    last_time = current_time;
                }

                double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

                //The sensor needs at least 9ms to compute the internal variables
                //dt must be in that case greater or equal to 10ms
                if(dt >= 50) //We use a data rate of 20Hz
                {
                    uint8_t status = kellerLD->read_data();
                    if(status != 99)
                    {
                        //Publish barometer data
                        uuv_msgs::msg::Barometer baro_msg;
                        baro_msg.header.stamp = Node::now();
                        baro_msg.header.frame_id = pressure_frame_id;
                        baro_msg.depth = (double)kellerLD->depth();
                        baro_msg.temperature = (double)kellerLD->temperature();
                        baro_msg.pressure = (double)kellerLD->pressure(KellerLD::bar);

                        barometer_pub_->publish(baro_msg);
                    }
                    else RCLCPP_ERROR(this->get_logger(), "KellerLD: Failed to read 5 bytes from the i2c bus.");
                    kellerWaitingForData = false;
                    last_time = current_time;
                }
                
            }
        }
    }
    
}

void I2C_SENSORS::read_BNO055()
{
    if(!calibrationFlag)
    {
        if (ioctl(file, I2C_SLAVE, BNO055_ADDRESS_A) < 0)
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to BNO055!");
        else
        {
            // Possible vector values can be:
            // - VECTOR_ACCELEROMETER - m/s^2
            // - VECTOR_MAGNETOMETER  - uT
            // - VECTOR_GYROSCOPE     - rad/s
            // - VECTOR_EULER         - degrees
            // - VECTOR_LINEARACCEL   - m/s^2
            // - VECTOR_GRAVITY       - m/s^2
            //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 
            imu::Vector<3> gyro = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            imu::Vector<3> acc = bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            imu::Vector<3> mag = bno->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
            //ENU frame
            imu::Quaternion quat = bno->getQuat();

            //Change frame orientation from ENU to NED
            tf2::Quaternion q_rot, q_new;
            tf2::Quaternion q_orig(quat.x(), quat.y(), quat.z(), quat.w());
            // Rotate the previous sensor pose by 180° about X and 90° about Z
            q_rot.setRPY(M_PI, 0.0, M_PI/2);

            q_new = q_rot * q_orig;
            q_new.normalize();
            
            tf2::Matrix3x3 m(q_new);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);


            if(yaw < 0)
            yaw += 2.0*M_PI;
            
            //NED frame (x=north, y=west, z=down)
            imu_data.clear();
            imu_data.gx = (float)gyro.x();
            imu_data.gy = (float)gyro.y();
            imu_data.gz = (float)gyro.z();

            imu_data.ax = (float)acc.x();
            imu_data.ay = (float)acc.y();
            imu_data.az = (float)acc.z();
            
            euler_data.clear();
            euler_data.roll = roll*180.0/M_PI; 
            euler_data.pitch = pitch*180.0/M_PI; 
            euler_data.yaw = yaw*180.0/M_PI;

            //----- Publish IMU data ----------------
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = Node::now();
            imu_msg.header.frame_id = imu_frame_id;
            
            imu_msg.orientation.x = q_new[0];
            imu_msg.orientation.y = q_new[1];
            imu_msg.orientation.z = q_new[2];
            imu_msg.orientation.w = q_new[3];
            
            imu_msg.linear_acceleration.x = (double)imu_data.ax;
            imu_msg.linear_acceleration.y = (double)imu_data.ay;
            imu_msg.linear_acceleration.z = (double)imu_data.az;

            imu_msg.angular_velocity.x =  (double)imu_data.gx;
            imu_msg.angular_velocity.y =  (double)imu_data.gy;
            imu_msg.angular_velocity.z =  (double)imu_data.gz;
            
            //----- Publish Euler data  ----------------
            geometry_msgs::msg::Vector3Stamped euler_msg;
            euler_msg.header.stamp = Node::now();
            euler_msg.header.frame_id = euler_frame_id;
            euler_msg.vector.x = (double)euler_data.roll;
            euler_msg.vector.y = (double)euler_data.pitch;
            euler_msg.vector.z = (double)euler_data.yaw;

            //----- Publish Manetometer data  ----------------
            sensor_msgs::msg::MagneticField mag_msg;
            mag_msg.header.stamp = Node::now();
            mag_msg.header.frame_id = mag_frame_id;
            // Magnetic field in Tesla
            mag_msg.magnetic_field.x = (double)mag.x() * 1e-6;
            mag_msg.magnetic_field.y = (double)mag.y() * 1e-6;
            mag_msg.magnetic_field.z = (double)mag.z() * 1e-6;
            
            euler_pub_->publish(euler_msg);
            imu_pub_->publish(imu_msg);
            mag_pub_->publish(mag_msg);
        }
    }
}

void I2C_SENSORS::read_ina219()
{
    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time_ina219).count();

    if(dt >= 100) //10Hz
    {
        if(!ina219->scan())
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to INA219!");
        else
        {
            float shuntvoltage = ina219->getShuntVoltage_mV();
            float busvoltage = ina219->getBusVoltage_V();
            float loadvoltage = busvoltage + (shuntvoltage / 1000.0);
            float current = ina219->getCurrent_mA();
            //float power = ina219->getPower_mW();

            uuv_msgs::msg::Battery battery_msgs;
            battery_msgs.header.stamp = Node::now();
            battery_msgs.header.frame_id = wattmeter_frame_id;
            battery_msgs.voltage = (double)loadvoltage;
            battery_msgs.current = (double)(current / 1000.0);
            battery_msgs.capacity = battery_capacity;
            
            if((loadvoltage < LOW_BATTERY) && (loadvoltage > 10.0f))
                battery_msgs.low_battery = true;
            
            battery_pub_->publish(battery_msgs);
        }

        last_time_ina219 = current_time;
    }
}


} //namespace


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigation_module::I2C_SENSORS>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
