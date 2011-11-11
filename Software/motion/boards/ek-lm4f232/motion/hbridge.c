// hbridge.c - Interfaces with the ChalkBot hbridge modules.
// Alex Suchko for ChalkBot
// November 10,2011

// Includes

#include "inc/hw_types.h"     // types used throughout this file and it's libraries
#include "inc/hw_memmap.h"    // memory map, register base addresses
#include "driverlib/gpio.h"   // gpio abstraction library
#include "hbridge.h"          // definition file for this file
#include "driverlib/rom.h"    // definitions for rom functions
#include "driverlib/sysctl.h" // definitions for system control

// Definitions

#define HBR_INPUT_PORT        (SYSCTL_PERIPH_GPION)
#define HBR_INPUT_PORT_BASE   (GPIO_PORTN_BASE)
#define HBR_INPUT_PINS        (GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5)
#define HBR_1_FF2             (GPIO_PIN_4)
#define HBR_1_FF1             (GPIO_PIN_3)
#define HBR_1_ALERT           (GPIO_PIN_2)
#define HBR_2_FF2             (GPIO_PIN_7)
#define HBR_2_FF1             (GPIO_PIN_6)
#define HBR_2_ALERT           (GPIO_PIN_5)
                              // pins for reading from input port on hbridge

#define HBR_OUTPUT_PORT       (SYSCTL_PERIPH_GPIOD)
#define HBR_OUTPUT_PORT_BASE  (GPIO_PORTD_BASE)
#define HBR_OUTPUT_PINS       (GPIO_PIN_4 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_7 | GPIO_PIN_0 | GPIO_PIN_6 | GPIO_PIN_5)
#define HBR_1_RESET           (GPIO_PIN_4)
#define HBR_1_PWMH            (GPIO_PIN_1)    // this pin will be operated by PWM hardware
#define HBR_1_PWML            (GPIO_PIN_3)
#define HBR_1_PHASE           (GPIO_PIN_2)
#define HBR_2_RESET           (GPIO_PIN_7)
#define HBR_2_PWMH            (GPIO_PIN_0)    // this pin will be operated by PWM hardware
#define HBR_2_PWML            (GPIO_PIN_6)
#define HBR_2_PHASE           (GPIO_PIN_5)
                              // pins for outputting to hbridge
#define HBR_1_PWM_MUX         (GPIO_PD1_M1PWM1)
                              // pin mux setting to access hardware as desired
#define HBR_2_PWM_MUX         (GPIO_PD0_M1PWM0)
                              // pin mux setting to access hardware as desired

// Variables

// Functions

// * hbr_init *****************************************************************
// * setup pins and PWM hardware to talk to hbridges                          *
// ****************************************************************************
void hbr_init(void)
{
  // Setup GPIO for HBRIDGE_LEFT AND HBRIDGE_RIGHT
                                                          // HBRIDGE_LEFT
  ROM_SysCtlPeripheralEnable(HBR_INPUT_PORT);             // enable clock to GPION
  ROM_GPIODirModeSet(HBR_INPUT_PORT_BASE, HBR_INPUT_PINS, 
                     GPIO_DIR_MODE_IN);                   // configure input GPIO
  ROM_GPIOPadConfigSet(HBR_INPUT_PORT_BASE, HBR_INPUT_PINS,
                       GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
                                                          // configure input pads 
                                                          // (note that type is input, drive spec meaningless)
  ROM_SysCtlPeripheralEnable(HBR_OUTPUT_PORT);            // enable clock to GPIOD
  ROM_GPIODirModeSet(HBR_OUTPUT_PORT_BASE, HBR_OUTPUT_PINS, 
                     GPIO_DIR_MODE_OUT);                  // configure output GPIO
  ROM_GPIOPadConfigSet(HBR_OUTPUT_PORT_BASE, HBR_OUTPUT_PINS,
                       GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
                                                          // configure output pads 
  
                                                          // configure PWM
  
  
  ROM_GPIOPinConfigure(HBR_1_PWM_MUX);                    // select mux for pwm output hbridge 1
  ROM_GPIOPinConfigure(HBR_2_PWM_MUX);                    // select mux for pwm output hbridge 2
}

// EOF