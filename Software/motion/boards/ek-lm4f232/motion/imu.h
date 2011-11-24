// imu.h - Definition file for interface for Pololu MinIMU-9 via I2C.
// Alex Suchko for ChalkBot
// November 20,2011

// Definitions


// Function Prototypes

void imu_init(void);                  // setup pins, I2C interface for imu
void imu_poll_gyro(void);             // poll gyro

// EOF