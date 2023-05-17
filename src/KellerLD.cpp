#include "i2c_sensors/KellerLD.h"

KellerLD::KellerLD() {
    fluidDensity = 1029;
}

void KellerLD::setI2CPort(int port)
{
    this->file_ = port;
}

void KellerLD::init() {
    // Request memory map information
    cust_id0 = readMemoryMap(LD_CUST_ID0);
    cust_id1 = readMemoryMap(LD_CUST_ID1);

    code = (uint32_t(cust_id1) << 16) | cust_id0;
    equipment = cust_id0 >> 10;
    place = cust_id0 & 0b000000111111111;
    file = cust_id1;

    uint16_t scaling0;
    scaling0 = readMemoryMap(LD_SCALING0);

    mode = scaling0 & 0b00000011;
    year = scaling0 >> 11;
    month = (scaling0 & 0b0000011110000000) >> 7;
    day = (scaling0 & 0b0000000001111100) >> 2;
	
    // handle P-mode pressure offset (to vacuum pressure)

    if (mode == 0) { 
        // PA mode, Vented Gauge. Zero at atmospheric pressure
        P_mode = 1.01325;
    } else if (mode == 1) {
        // PR mode, Sealed Gauge. Zero at 1.0 bar
        P_mode = 1.0;
    } else {
        // PAA mode, Absolute. Zero at vacuum
        // (or undefined mode)
        P_mode = 0;
    }

    uint32_t scaling12 = (uint32_t(readMemoryMap(LD_SCALING1)) << 16) | readMemoryMap(LD_SCALING2);
    P_min = *reinterpret_cast<float*>(&scaling12);
    uint32_t scaling34 = (uint32_t(readMemoryMap(LD_SCALING3)) << 16) | readMemoryMap(LD_SCALING4);
    P_max = *reinterpret_cast<float*>(&scaling34);
}

void KellerLD::setFluidDensity(float density) {
    fluidDensity = density;
}


uint8_t KellerLD::read_request() {
    uint8_t buffer[3];
    memset(buffer,'\0',3);
    int n_writ = 0;

    buffer[0] = LD_REQUEST;
    n_writ = write(file_,buffer,1);
    if (n_writ != 1)
    {
        printf("Failed to write 1 byte to the i2c bus.\n");
        return 99;
    }
    return 0;
}

uint8_t KellerLD::read_data() {
    uint8_t status;
    uint8_t buffer[5];
    memset(buffer,'\0',5);
    int n_read = 0;
    n_read = read(file_,buffer,5);
    if (n_read != 5) 
    {
        printf("Failed to read 5 bytes from the i2c bus.\n");
        return 99;
    }
    else
    {
        status = buffer[0];
		P = ((uint16_t)buffer[1] << 8) | (uint16_t)buffer[2];
        uint16_t T = ((uint16_t)buffer[3] << 8) | (uint16_t)buffer[4];

        P_bar = (float(P)-16384)*(P_max-P_min)/32768 + P_min + P_mode;
        T_degc = ((T>>4)-24)*0.05-50;
        return status;
    }

}

uint16_t KellerLD::readMemoryMap(uint8_t mtp_address) {
    uint8_t status;
    uint8_t buffer[3];
    memset(buffer,'\0',3);
    int n_writ = 0;

    buffer[0] = mtp_address;
    n_writ = write(file_,buffer,1);
    if (n_writ != 1) 
    printf("Failed to write 1 to the i2c bus.\n");

    usleep(1000); //1ms

    memset(buffer,'\0',3);
    int n_read = 0;
    n_read = read(file_,buffer,3);

    if (n_read != 3)
    {
       status = 99;
       printf("Failed to read 3 bytes from the i2c bus.\n");
       return 0;
    }
    else
    {
      status = buffer[0];
      uint16_t data = ((uint16_t)buffer[1] << 8) | (uint16_t)buffer[2];
      return data;
    }

    (void)status;
}

bool KellerLD::status() {
    if (equipment <= 62 ) {
        return true;
    } else {
        return false;
    }
}

float KellerLD::range() {
    return P_max-P_min;
}

float KellerLD::pressure(float conversion) {
    return P_bar*1000.0f*conversion;
}

float KellerLD::temperature() {
    return T_degc;
}

float KellerLD::depth() {
    return (pressure(KellerLD::Pa)-101325)/(fluidDensity*9.80665);
}

float KellerLD::altitude() {
    return (1-std::pow((pressure()/1013.25),0.190284))*145366.45*.3048;
}

bool KellerLD::isInitialized() {
    return (cust_id0 >> 10) != 63; // If not connected, equipment code == 63
}