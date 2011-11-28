// imu.h - Definition file for interface for Pololu MinIMU-9 via I2C.
// Alex Suchko for ChalkBot
// November 20,2011

// Definitions

// Typedef
typedef enum {
  GYRO_MACHINE_START_I2C = 0,                             // init I2C state
  GYRO_MACHINE_CONFIG_1,                                  // configuration 1 - init reg 1
  GYRO_MACHINE_CONFIG_2,                                  // configuration 2 - init reg 4
  GYRO_MACHINE_ZEROING,                                   // stationary condition averaging
  GYRO_MACHINE_RUNNING                                    // normal operation - integrating deltas
} gyro_machine_state_t;                                   // gyro state machine

// Function Prototypes

void imu_init(void);                  // setup pins, I2C interface for imu
void imu_SoftI2CCallback(void);       // callback function for soft I2C driver to give us an event
void imu_poll_gyro(void);             // poll gyro
signed long imu_get_heading(void);    // gets heading in millidegrees relative to where robot started
signed long imu_get_yaw_rate(void);   // gets heading yaw rate in millidps

// EOF