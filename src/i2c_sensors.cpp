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
    //ROS2 Parameters
    this->declare_parameter<std::string>("i2c_bus_address", "/dev/i2c-1");
    i2c_address = this->get_parameter("i2c_bus_address").as_string();
    RCLCPP_INFO(get_logger(), "I2C_BUS_ADDRESS: '%s'", i2c_address.c_str());

    this->declare_parameter<bool>("use_bno055", true);
    this->declare_parameter<bool>("use_ms5837", true);

    useBNO055 = this->get_parameter("use_bno055").as_bool();
    useMS5837 = this->get_parameter("use_ms5837").as_bool();

    this->declare_parameter<bool>("read_sensor_quality", false);
    this->declare_parameter<bool>("write_calibration", false);   

    
    calib_data = new uint8_t[22]; 
    filename = new char[15];
    
    sprintf(filename,(char*)i2c_address.c_str());
    file = open(filename, O_RDWR);
 
    bno = Adafruit_BNO055();
    ms5837 = MS5837();

    if(useMS5837)
    {
        RCLCPP_INFO(get_logger(), "The pressure sensor has been selected.");
        //int addr = 0x76; The I2C address
        if (ioctl(file, I2C_SLAVE, MS5837_ADDR) < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to MS5837.");
            //ERROR HANDLING; you can check errno to see what went wrong 
            exit(1);
        }

        ms5837.setPort(file);
        
        while (!ms5837.init()) {
            RCLCPP_INFO(this->get_logger(), "Are SDA/SCL connected correctly?");
            RCLCPP_INFO(this->get_logger(), "Blue Robotics Bar30: White=SDA, Green=SCL");
            usleep(2000000); // hold on
        }

        barometer_pub_ = this->create_publisher<uuv_msgs::msg::Barometer>("barometer/data", 1);
    }

    if(useBNO055)
    {
        RCLCPP_INFO(get_logger(), "The IMU sensor has been selected.");
        //int addr = 0x28; /* The I2C address */
        if (ioctl(file, I2C_SLAVE, BNO055_ADDRESS_A) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to BNO055!");
            exit(1);
        }
        
        bno.setI2CPort(file);

        while(!bno.begin(bno.OPERATION_MODE_NDOF)){
            RCLCPP_INFO(this->get_logger(), "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
            usleep(2000000); // hold on
        }
        
        bno.setExtCrystalUse(true);


        
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

        //--- Accelerometer Offset registers ---
        calib_data[0] = 11;  //226
        calib_data[1] = 0;   //255
        calib_data[2] = 51;   //222
        calib_data[3] = 0;    //255
        calib_data[4] = 244;  //255
        calib_data[5] = 255;
        calib_data[6] = 213;  //240
        //--- Magnetometer Offset registers --- 
        calib_data[7] = 253;  //255
        calib_data[8] = 11;   //93
        calib_data[9] = 255;
        calib_data[10] = 171;  //174
        calib_data[11] = 255;  //254
        calib_data[12] = 255;    //0
        //--- Gyroscope Offset registers ---
        calib_data[13] = 255;    //0
        calib_data[14] = 254;    //254
        calib_data[15] = 255;
        calib_data[16] = 1;    //255
        calib_data[17] = 0;
        //--- Radius registers --- 
        calib_data[18] = 232;
        calib_data[19] = 3;
        calib_data[20] = 26;   //186
        calib_data[21] = 4;     //2

        bno.setSensorOffsets(calib_data);
        usleep(2000000); // hold on

        euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("euler/data", 1);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1); 
    }

    //ROS2 timers
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&I2C_SENSORS::timerCallback, this)); //2hz
    timer_parameters_ = this->create_wall_timer(1000ms, std::bind(&I2C_SENSORS::get_parameters, this));

    last_time = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "Run!");
}

I2C_SENSORS::~I2C_SENSORS() {

    delete [] filename;
    delete [] buffer;
    delete [] calib_data;
    //delete [] vector_data;
}

void I2C_SENSORS::get_parameters()
{
    if(useBNO055)
    {
        readSensorQuality = this->get_parameter("read_sensor_quality").as_bool();
        writeCalibration = this->get_parameter("write_calibration").as_bool(); 

        //Read sensor quality data or setup calibration
        if(readSensorQuality || writeCalibration)
        {
            //int addr = 0x28; /* The I2C address */
            if (ioctl(file, I2C_SLAVE, BNO055_ADDRESS_A) < 0)
                RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to BNO055!");

            uint8_t system, gyro, accel, mag = 0;
            bno.getCalibration(&system, &gyro, &accel, &mag);
            //bno.getCalibationStatus(&system, &gyro, &accel, &mag);
            std::cout<< "CALIBRATION: Sys=" << (int)system << " Gyro=" << (int)gyro
                << " Accel=" << (int)accel << " Mag=" << (int)mag << std::endl;
        
            
            bno.getSensorOffsets(calib_data);
            for(uint8_t i=0; i<NUM_BNO055_OFFSET_REGISTERS; i++)
                printf("Hex: %d\n", calib_data[i]);
                
            
            if((int)mag == 3 && (int)gyro ==3 && (int)accel ==3 && writeCalibration){
                //this->setBNO055_Calib_Data(); 
                bno.setSensorOffsets(calib_data);
                RCLCPP_INFO(this->get_logger(), "The BNO055 Sensor has been calibrated!");

                this->set_parameters(
                {
                    rclcpp::Parameter("read_sensor_quality", false),
                    rclcpp::Parameter("write_calibration", false),
                });
            }
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
    if(useMS5837)
        this->read_barometer();
    if(useBNO055)
        this->read_BNO055();
}



void I2C_SENSORS::read_barometer()
{
    if((!readSensorQuality && !writeCalibration))
    {
        std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

        //int addr = 0x76; The I2C address
        if (ioctl(file, I2C_SLAVE, MS5837_ADDR) < 0) 
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to MS5837.");

        if(dt >= 20) //19 (22.10.21)
        {
            loops++;
            if(loops == 1)
                ms5837.read_sensor_part1(); 
            else if(loops == 2)
                ms5837.read_sensor_part2();  
            else if(loops == 3){
                ms5837.read_sensor_part3();
                ms5837.pressure();

                depth = (double)ms5837.depth();

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
                baro_msg.header.frame_id = "barometer_data_link";
                baro_msg.depth = current_depth;
                baro_msg.temperature = (double)ms5837.temperature();
                baro_msg.pressure = (double)ms5837.pressure();

                barometer_pub_->publish(baro_msg);
                loops = 0;
            }  
            last_time = current_time;   
        }


        //RCLCPP_INFO(this->get_logger(), "Depth: '%6.2F' Curr Depth: '%6.2F'", depth, current_depth);
        //RCLCPP_INFO(this->get_logger(), "Temperature: '%6.2F'", ms5837.temperature());
        //RCLCPP_INFO(this->get_logger(), "Pressure: '%6.2F'", ms5837.pressure());
    }
    
}

void I2C_SENSORS::read_BNO055()
{
    //int addr = 0x28; /* The I2C address */
    if (ioctl(file, I2C_SLAVE, BNO055_ADDRESS_A) < 0)
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to BNO055!");

    if((!readSensorQuality && !writeCalibration))
    {
        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);


        imu_data.clear();
        imu_data.gx = -(float)gyro.y();
        imu_data.gy = -(float)gyro.x();
        imu_data.gz = (float)gyro.z();

        imu_data.ax = (float)acc.x();
        imu_data.ay = (float)acc.y();
        imu_data.az = (float)acc.z();
        
        euler_data.clear();
        euler_data.roll = (float)euler.y();
        euler_data.pitch = -(float)euler.z();
        euler_data.yaw = (float)euler.x();

        //----- Publish IMU data ----------------
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = Node::now();
        imu_msg.header.frame_id = "imu_data_link";
        imu_msg.linear_acceleration.x = (double)imu_data.ax;
        imu_msg.linear_acceleration.y = (double)imu_data.ay;
        imu_msg.linear_acceleration.z = (double)imu_data.az;

        imu_msg.angular_velocity.x =  (double)imu_data.gx;
        imu_msg.angular_velocity.y =  (double)imu_data.gy;
        imu_msg.angular_velocity.z =  (double)imu_data.gz;
        
        //----- Publish Euler data  ----------------
        geometry_msgs::msg::Vector3Stamped euler_msg;
        euler_msg.header.stamp = Node::now();
        euler_msg.header.frame_id = "euler_data_link";
        euler_msg.vector.x = (double)euler_data.roll;
        euler_msg.vector.y = (double)euler_data.pitch;
        euler_msg.vector.z = (double)euler_data.yaw;  
        
        euler_pub_->publish(euler_msg);
        imu_pub_->publish(imu_msg);
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


