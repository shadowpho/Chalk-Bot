// svm.c - Interface for RC servo PWM, main update tick for system.
// Alex Suchko for ChalkBot
// November 10,2011

// Includes

#include "inc/hw_types.h"     // types used throughout this file and it's libraries
#include "inc/hw_memmap.h"    // memory map, register base addresses
#include "driverlib/gpio.h"   // gpio abstraction library
#include "driverlib/pwm.h"    // pwm abstraction library
#include "svm.h"              // definition file for this file
#include "driverlib/rom.h"    // definitions for rom functions
#include "driverlib/sysctl.h" // definitions for system control

// Definitions

#define SVM_OUTPUT_PORT       (SYSCTL_PERIPH_GPIOH)
#define SVM_OUTPUT_PORT_BASE  (GPIO_PORTH_BASE)
#define SVM_OUTPUT_PINS       (GPIO_PIN_2 | GPIO_PIN_3)
#define SVM_1_PWM_PIN         (GPIO_PIN_2)            // this pin will be operated by PWM hardware
#define SVM_2_PWM_PIN         (GPIO_PIN_3)            // this pin will be operated by PWM hardware

                                                      // pins for outputting to chalk servomotors
                                                      // NOTICE - PWM Module 0 also contains the
                                                      //          generator module attached to the
                                                      //          accessory drives.
#define SVM_1_PWM_MUX         (GPIO_PH2_M0PWM2)       // pin mux setting to access hardware as desired
#define SVM_2_PWM_MUX         (GPIO_PH3_M0PWM3)       // pin mux setting to access hardware as desired

#define SVM_PWM_MODULE        (SYSCTL_PERIPH2_PWM0)   // SVM (and accessory) control on PWM Module 0
#define SVM_PWM_BASE          (PWM0_BASE)             // pwm module base address
#define SVM_PWM_GEN           (PWM_GEN_1)             // pwm generator reference for SVM (accessory on PWM_GEN_0)
#define SVM_PWM_FPWM_DIV      (SYSCTL_PWMDIV_16)      // div16 to enable ~50Hz PWM output for SVM control
                                                      // when Fsys = 50MHz, yields Fpwm=3.125MHz
                                                      // this setup is also performed in hbridge.c\hbr_init(),
                                                      // but is repeated here to avoid confusion over ordering of
                                                      // init functions
                                                      // Resulting resolution: 3.125ticks/us
                                                      // Reference max range for RC servo control: 700us-2300us, 1500us center
#define SVM_PWM_PERIOD_TICKS  (52082)                 // (desired divide for Fsw on pwm ouptut) - 1, from Fpwm
                                                      // specifically, this specifies the pwm counter rollover
                                                      // 52082 yields Fsvm of approximatly 60Hz
#define SVM_1_PWM             (PWM_OUT_2)             // pwm channel for SVM_1
#define SVM_2_PWM             (PWM_OUT_3)             // pwm channel for SVM_2

#define SVM_1_PWM_BIT         (PWM_OUT_2_BIT)         // pwm bit for SVM_1
#define SVM_2_PWM_BIT         (PWM_OUT_3_BIT)         // pwm bit for SVM_2

                                                      // Conversion factor us->ticks, for fast binary fraction implementation
                                                      // Implemented conversion factor: x3.125(ticks/us)
#define SVM_USTOTICK_WHOLE    (3)                     // whole number part of multiplier
#define SVM_USTOTICK_SHIFT    (3)                     // shift amount to perform binary fraction part of multiplier
#define SVM_USTOTICK_FRACTION (1)                     // binary fraction part of multiplier [fraction*2^(SVM_USTOTICK_SHIFT)]

// Variables

// Functions

// * svm_init *****************************************************************
// * setup pins and PWM hardware to talk to servomotors                       *
// * Assumes system clock already configured                                  *
// ****************************************************************************
void svm_init(void)
{
  ROM_SysCtlPeripheralEnable(SVM_OUTPUT_PORT);            // enable clock to GPIO port
                                                          // configure output pads 
  GPIOPinTypeGPIOOutput(SVM_OUTPUT_PORT_BASE, SVM_OUTPUT_PINS);
  
                                                          // configure PWM
  ROM_SysCtlPeripheralEnable(SVM_PWM_MODULE);             // enable clock to pwm module
  ROM_SysCtlPWMClockSet(SVM_PWM_FPWM_DIV);                // configure clock divider to derive Fpwm from Fsys
                                                          // wrap 16b PWM counter at 1041 for 3kHz pwm output
  ROM_PWMDeadBandDisable(SVM_PWM_BASE, SVM_PWM_GEN);      // allow PWM0, PWM1 to behave independently
  ROM_PWMGenConfigure(SVM_PWM_BASE, SVM_PWM_GEN,          // configure pwm generator
                      PWM_GEN_MODE_DOWN |                 // up/down count for center timed PWM
                      PWM_GEN_MODE_NO_SYNC);              // outputs from generator behave independently
  ROM_PWMGenPeriodSet(SVM_PWM_BASE, SVM_PWM_GEN,          // sets period for generator to appropriate period
                      SVM_PWM_PERIOD_TICKS);
  ROM_PWMPulseWidthSet(SVM_PWM_BASE, SVM_1_PWM, 0);       // set initial pulse widths to 0 for safety
  ROM_PWMPulseWidthSet(SVM_PWM_BASE, SVM_2_PWM, 0);       // set initial pulse widths to 0 for safety
  
  ROM_GPIOPinConfigure(SVM_1_PWM_MUX);                    // select mux for pwm output hbridge 1
  ROM_GPIOPinConfigure(SVM_2_PWM_MUX);                    // select mux for pwm output hbridge 2
  ROM_GPIOPinTypePWM(SVM_OUTPUT_PORT_BASE, SVM_1_PWM_PIN);// do some other step configuring pwm...
  ROM_GPIOPinTypePWM(SVM_OUTPUT_PORT_BASE, SVM_2_PWM_PIN);// ...exact function is unknown
  
  ROM_PWMOutputState(SVM_PWM_BASE, SVM_1_PWM_BIT | SVM_2_PWM_BIT, true);
                                                          // enable outputs from pwm generator to pins
  ROM_PWMGenEnable(SVM_PWM_BASE, SVM_PWM_GEN);            // enable pwm output generator
}

// * svm_set_pulse ************************************************************
// * update state of svm line                                                 *
// * ucSvm should be one of: SVM_1, SVM_2                                     *
// * ulWidth is in units of ticks and should not be larger than               *
// *                          SVM_PWM_PERIOD_TICKS - 1                        *
// * Assumes pins already set up (svm_init()).                                *
// ****************************************************************************
void svm_set_pulse(unsigned char ucSvm, unsigned long ulWidth)
{
  if(ulWidth > (SVM_PWM_PERIOD_TICKS - 1))                  // ceiling ulWidth at max legal value
    ulWidth = (SVM_PWM_PERIOD_TICKS - 1);
  
  if(ucSvm == SVM_1)
      ROM_PWMPulseWidthSet(SVM_PWM_BASE, SVM_1_PWM, ulWidth);
                                                            // update ceilinged pulse width to appropriate svm
  else if(ucSvm == SVM_2)
      ROM_PWMPulseWidthSet(SVM_PWM_BASE, SVM_2_PWM, ulWidth);
                                                            // update ceilinged pulse width to appropriate svm    

}

// * svm_set_us ***************************************************************
// * update pulse width of svm line in microseconds                           *
// * ucSvm should be one of: SVM_1, SVM_2                                     *
// * ulMicroseconds is in units of us and for RC servo control should be      *
// *                              700us-2300us                                *
// * Assumes pins already set up (svm_init()).                                *
// ****************************************************************************
void svm_set_us(unsigned char ucSvm, unsigned long ulMicroseconds)
{
  unsigned long ulTicks;                                    // temporary variable for quickly performing *3.125ticks/us conversion
  ulTicks = ulMicroseconds * SVM_USTOTICK_WHOLE;            // whole part multiply
  ulTicks <<= SVM_USTOTICK_SHIFT;                           // shift up to perform fractional operation
  ulTicks += ulMicroseconds * SVM_USTOTICK_FRACTION;        // add on fractional part of product
  ulTicks >>= SVM_USTOTICK_SHIFT;                           // truncate fractonal part of result, done.
  svm_set_pulse(ucSvm, ulTicks);
}

// EOF