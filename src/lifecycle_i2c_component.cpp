#include "i2c_sensors/lifecycle_i2c_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;


namespace composition
{

int file;

LifecycleI2CSensors::LifecycleI2CSensors(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("i2c_sensors_node", options)
{
    //ROS2 Parameters
    this->declare_parameter<bool>("read_sensor_quality", false);
    this->declare_parameter<bool>("write_calibration",   false);
}

LifecycleI2CSensors::~LifecycleI2CSensors() {
    delete [] filename;
    delete [] buffer;
    delete [] calib_data;
    this->close_i2c();
}

/// Transition callback for state configuring
/// TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
/// TRANSITION_CALLBACK_FAILURE transitions to "inactive"
/// TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleI2CSensors::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    calib_data = new uint8_t[22]; 
    filename = new char[15];
    
    sprintf(filename,"/dev/i2c-1");
    file = open(filename, O_RDWR);
 
    bno = Adafruit_BNO055();
    ms5837 = MS5837();

    
    //int addr = 0x76; The I2C address
    if (ioctl(file, I2C_SLAVE, MS5837_ADDR) < 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to MS5837.");
    	//ERROR HANDLING; you can check errno to see what went wrong 
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    ms5837.setPort(file);
    
    while (!ms5837.init()) {
        RCLCPP_INFO(this->get_logger(), "Are SDA/SCL connected correctly?");
        RCLCPP_INFO(this->get_logger(), "Blue Robotics Bar30: White=SDA, Green=SCL");
        std::this_thread::sleep_for(2s);
    }
    
    //int addr = 0x28; /* The I2C address */
    if (ioctl(file, I2C_SLAVE, BNO055_ADDRESS_A) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to BNO055!");
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    
    bno.setI2CPort(file);

    while(!bno.begin(bno.OPERATION_MODE_NDOF)){
        RCLCPP_INFO(this->get_logger(), "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        std::this_thread::sleep_for(2s);
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
    calib_data[0] = 11;   
    calib_data[1] = 0;    
    calib_data[2] = 51;   
    calib_data[3] = 0;    
    calib_data[4] = 244;  
    calib_data[5] = 255;
    calib_data[6] = 213; 
    //--- Magnetometer Offset registers --- 
    calib_data[7] = 253;   
    calib_data[8] = 11;    
    calib_data[9] = 255;
    calib_data[10] = 171;  
    calib_data[11] = 255;  
    calib_data[12] = 255;  
    //--- Gyroscope Offset registers ---
    calib_data[13] = 255;  
    calib_data[14] = 254;  
    calib_data[15] = 255;
    calib_data[16] = 1;   
    calib_data[17] = 0;
    //--- Radius registers --- 
    calib_data[18] = 232;
    calib_data[19] = 3;
    calib_data[20] = 26;   
    calib_data[21] = 4;    

    bno.setSensorOffsets(calib_data);
    std::this_thread::sleep_for(2s);
    
    while(readSensorQuality || writeCalibration)
    {
        this->set_new_calibration_parameters();
        std::this_thread::sleep_for(0.5s);
    }

    //ROS2
    barometer_pub_ = this->create_publisher<uuv_msgs::msg::Barometer>("barometer/data", 1);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
    euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("euler/data", 1); 
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&LifecycleI2CSensors::on_timer, this)); //2hz

                       
    RCLCPP_INFO(this->get_logger(), "Run!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state activating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleI2CSensors::on_activate(const rclcpp_lifecycle::State &)
{
    barometer_pub_->on_activate();
    imu_pub_->on_activate();
    euler_pub_->on_activate();
    
    last_time = std::chrono::high_resolution_clock::now();
    last_time_parameters = std::chrono::steady_clock::now();
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state deactivating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleI2CSensors::on_deactivate(const rclcpp_lifecycle::State &)
{
   
    barometer_pub_->on_deactivate();
    imu_pub_->on_deactivate();
    euler_pub_->on_deactivate();
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state cleaningup

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleI2CSensors::on_cleanup(const rclcpp_lifecycle::State &)
{

    readSensorQuality = false;
    writeCalibration = false;
    barometer_ready_ = false;
    
    //Reset parameters
    this->set_parameters({
                           rclcpp::Parameter("read_sensor_quality", false),
                           rclcpp::Parameter("write_calibration",   false), });
    
    count_baro = 0;
    loops = 0;
    depth_adjustment_ = 0.0;
    current_depth = 0.0;
    depth = 0.0;
    
    this->close_i2c();
    timer_.reset();
    barometer_pub_.reset();
    imu_pub_.reset();
    euler_pub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;  
}

/// Transition callback for state shutting down
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleI2CSensors::on_shutdown(const rclcpp_lifecycle::State & state)
{
    timer_.reset();
    barometer_pub_.reset();
    imu_pub_.reset();
    euler_pub_.reset();

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());
      
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LifecycleI2CSensors::close_i2c()
{
   close(file);
}


void LifecycleI2CSensors::Int2bytes(int value, uint8_t *bytes)
{
    bytes[0] = (value >> 24) & 0xFF;
    bytes[1] = (value >> 16) & 0xFF;
    bytes[2] = (value >> 8) & 0xFF;
    bytes[3] = value & 0xFF;
}



bool LifecycleI2CSensors::read_barometer()
{

    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

    //int addr = 0x76; The I2C address
    if (ioctl(file, I2C_SLAVE, MS5837_ADDR) < 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to MS5837.");
        return false;
    }

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
    
    
    return true;
}

bool LifecycleI2CSensors::read_BNO055()
{
    //int addr = 0x28; /* The I2C address */
    if (ioctl(file, I2C_SLAVE, BNO055_ADDRESS_A) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to BNO055!");
        return false;
    }
 
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
    imu_msg.linear_acceleration.x = (double)imu_data.ax;
    imu_msg.linear_acceleration.y = (double)imu_data.ay;
    imu_msg.linear_acceleration.z = (double)imu_data.az;

    imu_msg.angular_velocity.x =  (double)imu_data.gx;
    imu_msg.angular_velocity.y =  (double)imu_data.gy;
    imu_msg.angular_velocity.z =  (double)imu_data.gz;
        
    //----- Publish Euler data  ----------------
    geometry_msgs::msg::Vector3Stamped euler_msg;
    euler_msg.vector.x = (double)euler_data.roll;
    euler_msg.vector.y = (double)euler_data.pitch;
    euler_msg.vector.z = (double)euler_data.yaw;  
        
    euler_pub_->publish(euler_msg);
    imu_pub_->publish(imu_msg);
    
    return true;
}

void LifecycleI2CSensors::set_new_calibration_parameters()
{
    //Read sensor quality data or setup calibration
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


void LifecycleI2CSensors::on_timer()
{

    if(!barometer_pub_->is_activated() || !imu_pub_->is_activated() || !euler_pub_->is_activated())
    {
        //RCLCPP_INFO(get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    }
    else
    { 
        try
        {
            this->read_barometer();
            this->read_BNO055();
        }
        catch(std::exception& e)
        {
            std::cout << "Exception:" << std::endl;
            std::cout << e.what() << std::endl;
        }  
    }
}


void LifecycleI2CSensors::set_parameter(bool &value, bool &out, 
rcl_interfaces::msg::SetParametersResult &result)
{
    result.successful = true;
    result.reason = "success";
    out = value;
}

rcl_interfaces::msg::SetParametersResult LifecycleI2CSensors::parametersCallback(
const std::vector<rclcpp::Parameter> &parameters)
{
    std::string reason;

    rcl_interfaces::msg::SetParametersResult result;
    for (const auto &parameter : parameters)
    {
        const std::string name = parameter.get_name();
        const rclcpp::ParameterType param_type = parameter.get_type();
        
        if(param_type == rclcpp::ParameterType::PARAMETER_BOOL)
        {
            bool value = parameter.as_bool();
            if(name == "read_sensor_quality")
                this->set_parameter(value, readSensorQuality, result);
            else if(name == "write_calibration")
                this->set_parameter(value, writeCalibration, result);
        }

    }
    return result;
}




}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::LifecycleI2CSensors)
