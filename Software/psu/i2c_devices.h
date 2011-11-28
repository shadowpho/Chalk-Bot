#ifndef I2C_DEVICES_H
#define I2C_DEVICES_H

#include "device.h"

#define CH1  0 
#define CH2  1

#define CHAN1_INA226 68
#define CHAN2_INA226 69

#define CHAN1_TMP100 73
#define CHAN2_TMP100 79

//0b01100000 write on reg1, config write, high accuracy. always on.
//0b01100000 = 0x60
#define TMP100_CNF  0x60
////Magical number, used by 0.00512 / ((max_current/2^15) * 40 milliohms to ohms), 16 bit watch out
#define INA226_SHUNT 2097
////Magical number, used by 56 mv / 2.5 microvolts = 22400, 16 bit watch out
#define INA226_LIMIT 22400

struct ina226_status_s{
	float shunt_voltage;
	float bus_voltage;
	float current;
	float power;
};
extern volatile float tmp100_status[2];
extern volatile struct ina226_status_s ina226_status[2];
	
int set_tmp100(int channel);
int set_ina226(int channel);

int read_tmp100(int channel);
int read_ina226(int channel);

#endif //I2C_DEVICES_H