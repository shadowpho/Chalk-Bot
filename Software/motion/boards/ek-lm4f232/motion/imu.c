// imu.c - Interface for Pololu MinIMU-9 via I2C.
// Alex Suchko for ChalkBot
// November 21, 2011

// Includes

#include "inc/hw_types.h"     // types used throughout this file and it's libraries
#include "inc/hw_memmap.h"    // memory map, register base addresses
#include "driverlib/gpio.h"   // gpio abstraction library
#include "driverlib/i2c.h"    // I2C library
#include "imu.h"              // definition file for this file
#include "driverlib/rom.h"    // definitions for rom functions
#include "driverlib/sysctl.h" // definitions for system control
#include "driverlib/interrupt.h"
                              // NVIC API
#include "inc/hw_ints.h"      // NVIC hardware interrupt vector enumerations/aliases

// Definitions

#define IMU_PORT        (SYSCTL_PERIPH_GPIOA)             // pin configuration
#define IMU_PORT_BASE   (GPIO_PORTA_BASE)
#define IMU_PINS        (GPIO_PIN_6 | GPIO_PIN_7)
#define IMU_SCL         (GPIO_PIN_6)
#define IMU_SDA         (GPIO_PIN_7)
#define IMU_SCL_MUX     (GPIO_PA6_I2C1SCL)
#define IMU_SDA_MUX     (GPIO_PA7_I2C1SDA)

#define IMU_I2C_BASE    (I2C1_MASTER_BASE)                // I2C configuration



// Variables

// Functions

// * imu_init *****************************************************************
// * Setup pins and I2C interface to communicate with IMU (Pololu MinIMU-9).  *
// ****************************************************************************
void imu_init(void)
{
  unsigned long config_temp;                              // used when assembling configuration values
  
                                                          // setup pins for I2C--------------------
  ROM_SysCtlPeripheralEnable(IMU_PORT);                   // enable clock to GPIO port
                                                          // configure I/O pads 
  ROM_GPIOPinConfigure(IMU_SCL_MUX);                      // select mux for I2C input
  ROM_GPIOPinConfigure(IMU_SDA_MUX);                      // select mux for I2C input
                                                          // Modified to have no pull up/down resistors,
                                                          // these are on MinIMU-9 board.
  ROM_GPIODirModeSet(IMU_PORT_BASE, IMU_PINS, GPIO_DIR_MODE_HW); 
                                                          // I2C pin tristates under HW control.
  ROM_GPIOPadConfigSet(IMU_PORT_BASE, IMU_PINS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
                                                          // Note, output drive config only configs
                                                          // pull up/down resistors when pin is input.
  
                                                          // setup I2C for master mode-------------
  ROM_I2CMasterInitExpClk(IMU_I2C_BASE, ROM_SysCtlClockGet(), false);
                                                          // setup I2C for master mode 100Kbps
}

// EOF