/* Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
------------------------------------------------------------
 
Title: Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
Description: This library provides utilities to communicate with and to
read data from the Measurement Specialties MS5837-30BA pressure/temperature 
sensor.
Authors: Rustom Jehangir, Blue Robotics Inc.
         Adam imko, Blue Robotics Inc.
-------------------------------
The MIT License (MIT)
Copyright (c) 2015 Blue Robotics Inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/ 

#ifndef MS5837_H_BLUEROBOTICS
#define MS5837_H_BLUEROBOTICS


#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <stdlib.h>
#include <cstdint>

#include <iostream>
#include <time.h>
#include <stdint.h>
#include <string.h>

#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>

#define MS5837_ADDR               0x76  
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

int adapter_nr = 2; 
char filename[40];

class MS5837
{
public:
    static constexpr float Pa = 100.0f;
    static constexpr float bar = 0.001f;
    static constexpr float mbar = 1.0f;
    uint8_t ready = 0;

    MS5837()
    {	
        this->setFluidDensity(1029);
    }

    bool init() 
    {
        uint8_t buffer[2];
	int n_writ, n_read;
	memset(buffer,'\0',2);

	// Reset the MS5837, per datasheet
	buffer[0] = MS5837_RESET;
	n_writ = write(file_,buffer,1);
	if( n_writ < 1)
	{
	    /* ERROR HANDLING: i2c transaction failed */
	    return false;
	}

        //printf("Waiting for PRESSURE SENSOR to initialize...\n\n");
        //RCLCPP_INFO(this->get_logger(), "Waiting for PRESSURE SENSOR to initialize...");

        // Wait for reset to complete
        usleep(100000);
        //printf("PRESSURE SENSOR initialized!\n\nReading and storing calibration data...\n\n");
        //RCLCPP_INFO(this->get_logger(), "PRESSURE SENSOR initialized!");
        //RCLCPP_INFO(this->get_logger(), "Reading and storing calibration data...");


	memset(buffer,'\0',2);
	// Read calibration values and CRC
	for ( uint8_t i = 0 ; i < 7 ; i++ ) 
	{
		memset(buffer,'\0',2);
		buffer[0] = MS5837_PROM_READ+(i*2);
		n_writ = write(file_,buffer,1);
		n_read = read(file_,buffer,2);
            if (n_read != 2) 
	    {
                /* ERROR HANDLING: i2c transaction failed */
                C[i] = 0;
            }
	    else
	    {	
		C[i] = (((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
	    }
			
	}
			
        //RCLCPP_INFO(this->get_logger(), "Calibration data stored. Checking CRC...");
	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);

	//printf("CRC Recv'd:  %i, CRC Calc:  %i\n\n",crcRead,crcCalculated);

	if ( crcCalculated == crcRead ) 
	{
		// Success
		//printf("Calibration Values CRC Check Success!\n");
		return true;
	} 
	else 
	{
		// Failure - try again?
		//printf("Calibration Values CRC Check FAILURE!\n");
		return false;
                exit (EXIT_FAILURE);
	}

        //RCLCPP_INFO(this->get_logger(), "Setup complete!");
	//return ready;
    }
 
    void setPort(int &port)
    {
	file_ = port;
    }

    /** Provide the density of the working fluid in kg/m^3. Default is for 
     * seawater. Should be 997 for freshwater.
     */
    void setFluidDensity(float density) {
	fluidDensity = density;
    }

    /** The read from I2C takes up for 40 ms, so use sparingly is possible.
     */
    void read_sensor_part1()
    {
	uint8_t buffer[3];
	memset(buffer,'\0',3);
	int n_writ = 0;
	// Request D1 conversion
	buffer[0] = MS5837_CONVERT_D1_8192;
	n_writ = write(file_,buffer,1);
	if (n_writ != 1) 
	{
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to write 1 to the i2c bus.\n");
        }
    }
    
    void read_sensor_part2()
    {
	uint8_t buffer[3];
	memset(buffer,'\0',3);
	int n_writ = 0, n_read = 0;
	buffer[0] = MS5837_ADC_READ;
	n_writ = write(file_,buffer,1);
	if (n_writ != 1) 
	{
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to write 2 to the i2c bus.\n");
        }
        
	D1 = 0;
	n_read = read(file_,buffer,3);
	if (n_read != 3) 
	{
           /* ERROR HANDLING: i2c transaction failed */
           printf("Failed to read 1 from the i2c bus.\n");
        }
	else
	{
	    D1 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
	}
	
	// Request D2 conversion
	buffer[0] = MS5837_CONVERT_D2_8192;
	n_writ = write(file_,buffer,1);
	if (n_writ != 1) 
	{
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to write 3 to the i2c bus.\n");
        }
    }
    
    
    void read_sensor_part3()
    {
	uint8_t buffer[3];
	memset(buffer,'\0',3);
        int n_writ = 0, n_read = 0;
	buffer[0] = MS5837_ADC_READ;
	n_writ = write(file_,buffer,1);
	if (n_writ != 1) 
	{
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to write 4 to the i2c bus.\n");
        }
	D2 = 0;
	n_read = read(file_,buffer,3);
	if (n_read != 3) 
	{
           /* ERROR HANDLING: i2c transaction failed */
           printf("Failed to read 2 from the i2c bus.\n");
        }
	else
	{
	    D2 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
	}

	//printf("D1 = %i,D2 = %i\n\n",D1,D2);
	calculate();
    }

    /** This function loads the datasheet test case values to verify that
     *  calculations are working correctly. No example checksum is provided
     *  so the checksum test may fail.
     */
    void readTestCase() {
	C[0] = 0;
	C[1] = 34982;
	C[2] = 36352;
	C[3] = 20328;
	C[4] = 22354;
	C[5] = 26646;
	C[6] = 26146;
	C[7] = 0;

	D1 = 4958179;
	D2 = 6815414;

	calculate();
    }

    /** Pressure returned in mbar or mbar*conversion rate.
     */
    float pressure(float conversion = 1.0f) {
	return P/10.0f*conversion;
    }


    /** Temperature returned in deg C.
     */
    float temperature() {
	return TEMP/100.0f;
    }

    /** Depth returned in meters (valid for operation in incompressible
     *  liquids only. Uses density that is set for fresh or seawater.
     */
    float depth() {
	return (pressure(Pa)-101300)/(fluidDensity*9.80665);
    }

    /** Altitude returned in meters (valid for operation in air only).
     */
    float altitude() {
	return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
    }


private:
    uint16_t C[8];
    uint32_t D1, D2;
    int32_t TEMP;
    int32_t P;
    int file_;

    float fluidDensity;

    void calculate() {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation
	
        int32_t dT;
	int64_t SENS;
	int64_t OFF;
	int32_t SENSi; 
	int32_t OFFi;  
	int32_t Ti;    
	int64_t OFF2;
	int64_t SENS2;
	
	// Terms called
	dT = D2-uint32_t(C[5])*256l;
	SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
	OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
	
	
	//Temp and P conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
	P = (D1*SENS/(2097152l)-OFF)/(8192l);
	
	//Second order compensation
	if((TEMP/100)<20){         //Low temp
	    Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
	    OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
	    SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
	    if((TEMP/100)<-15){    //Very low temp
	        OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
		SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
	    }
	}
	else if((TEMP/100)>=20){    //High temp
	    Ti = 2*(dT*dT)/(137438953472LL);
	    OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
	    SENSi = 0;
	}
	
	OFF2 = OFF-OFFi;   //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;
	
	TEMP = (TEMP-Ti);
	P = (((D1*SENS2)/2097152l-OFF2)/8192l);
    }


    uint8_t crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
	    if ( i%2 == 1 ) {
                n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
	    } else {
	        n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
	    }
	    for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
	        if ( n_rem & 0x8000 ) {
		    n_rem = (n_rem << 1) ^ 0x3000;
		} else {
		    n_rem = (n_rem << 1);
		}
	    }
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);
	return (n_rem ^ 0x00);
    }

};

#endif
