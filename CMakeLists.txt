cmake_minimum_required(VERSION 3.5)
project(i2c_sensors)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uuv_msgs REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)



include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR} 
)
ament_python_install_package(scripts)

file(GLOB SOURCE_FILES
 src/BNO055.cpp
 include/${PROJECT_NAME}/BNO055.h
 include/${PROJECT_NAME}/MS5837.hpp
)



add_library(i2c_sensors_component SHARED src/lifecycle_i2c_component.cpp ${SOURCE_FILES})
target_compile_definitions(i2c_sensors_component PRIVATE "COMPOSITION_BUILDING_DLL")
  
ament_target_dependencies(i2c_sensors_component
  "rclcpp_components"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "uuv_msgs"
  "sensor_msgs"
  "geometry_msgs")
  
rclcpp_components_register_nodes(i2c_sensors_component "composition::LifecycleI2CSensors")
set(node_plugins "${node_plugins}composition::LifecycleI2CSensors;$<TARGET_FILE:i2c_sensors_component>\n")

add_executable(${PROJECT_NAME}_node src/i2c_sensors.cpp ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME}_node 
  "rclcpp"
  "uuv_msgs" 
  "sensor_msgs" 
  "geometry_msgs")

#Install C++ targets
install(TARGETS
  i2c_sensors_component

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
  
#Install C++ targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

#Install Python scripts
#file(GLOB SCRIPT_FILES scripts/*)
install(PROGRAMS
  #${SCRIPT_FILES}
  #scripts/dvl-A50.py
  DESTINATION lib/${PROJECT_NAME}
)

#Install Launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
