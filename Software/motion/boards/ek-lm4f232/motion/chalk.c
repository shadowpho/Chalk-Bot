// chalk.c - Chalk mechanism controller.
// Alex Suchko for ChalkBot
// November 27,2011

// Includes

#include "inc/hw_types.h"     // types used throughout this file and it's libraries
#include "inc/hw_memmap.h"    // memory map, register base addresses
#include "driverlib/gpio.h"   // gpio abstraction library
#include "inc/hw_gpio.h"      // lower level access to gpio registers
#include "driverlib/rom.h"    // definitions for rom functions
#include "driverlib/sysctl.h" // definitions for system control
#include "svm.h"              // servomotor interface driver
#include "chalk.h"            // definition file for this file

// Definitions

#define CHALK_SW_PORT       (SYSCTL_PERIPH_GPIOJ)
#define CHALK_SW_PORT_BASE  (GPIO_PORTJ_BASE)
#define CHALK_SW_PINS       (GPIO_PIN_0)
#define CHALK_SW_PIN        (GPIO_PIN_0)              // grounded when chalk sw depressed
#define CHALK_SW_STATUS()   (!(HWREG(CHALK_SW_PORT_BASE + (GPIO_O_DATA + (CHALK_SW_PINS << 2))) & CHALK_SW_PIN))
                                                      // returns true for chalk switch depressed
                                                      // code copied+modified from StellarisWare GPIO library
                                                      // see page 680 of LM4F232H5QD manual, regarding ADDR[9:2] masking

// Variables

// Functions

// * chalk_init ***************************************************************
// * Initializes underlying svm driver (see "svm.h") and chalk mechanism      *
// * state variables.                                                         *
// * Assumes system clock already configured                                  *
// ****************************************************************************
void chalk_init(void)
{
  ROM_SysCtlPeripheralEnable(CHALK_SW_PORT);              // enable clock
  ROM_GPIOPinTypeGPIOInput(CHALK_SW_PORT_BASE,CHALK_SW_PINS);
                                                          // configure input pad(s)
  
}

// * chalk_sw_status **********************************************************
// * Reads the chalk switch status, true for depressed.                       *
// ****************************************************************************
tBoolean chalk_sw_status(void)
{
  return(CHALK_SW_STATUS());                                // return the chalk switch status
}

// EOF