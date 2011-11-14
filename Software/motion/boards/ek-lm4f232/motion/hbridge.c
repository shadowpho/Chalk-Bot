// hbridge.c - Interfaces with the ChalkBot hbridge modules.
// Alex Suchko for ChalkBot
// November 10,2011

// Includes

#include "inc/hw_types.h"     // types used throughout this file and it's libraries
#include "inc/hw_memmap.h"    // memory map, register base addresses
#include "driverlib/gpio.h"   // gpio abstraction library
#include "driverlib/pwm.h"    // pwm abstraction library
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

#define HBR_PWM_MODULE        (SYSCTL_PERIPH_PWM1)    // traction motor control on module 1 (NOT 0, that is for SVM)
#define HBR_PWM_BASE          (PWM1_BASE)             // pwm module base address
#define HBR_PWM_GEN           (PWM_GEN_0)             // pwm generator reference
#define HBR_PWM_FPWM_DIV      (SYSCTL_PWMDIV_16)      // div16 to enable ~50Hz PWM output for SVM control
                                                      // when Fsys = 50MHz, yields Fpwm=3.125MHz
#define HBR_PWM_PERIOD_TICKS  (1024)                  // (desired divide for Fsw on pwm ouptut) - 1, from Fpwm
                                                      // specifically, this specifies the pwm counter rollover
                                                      // 1024 yields Fsw of approximatly 3.049kHz
#define HBR_1_PWM             (PWM_OUT_1)             // pwm channel for HBR_1
#define HBR_2_PWM             (PWM_OUT_0)             // pwm channel for HBR_2

#define HBR_1_PWM_BIT         (PWM_OUT_1_BIT)         // pwm bit for HBR_1
#define HBR_2_PWM_BIT         (PWM_OUT_0_BIT)         // pwm bit for HBR_2

// Variables

// Functions

// * hbr_init *****************************************************************
// * setup pins and PWM hardware to talk to hbridges                          *
// * Assumes system clock already configured                                  *
// ****************************************************************************
void hbr_init(void)
{
  // Setup GPIO for HBRIDGE_LEFT AND HBRIDGE_RIGHT

  ROM_SysCtlPeripheralEnable(HBR_INPUT_PORT);             // enable clock to GPION
  ROM_GPIOPinTypeGPIOInput(HBR_INPUT_PORT_BASE,HBR_INPUT_PINS);
                                                          // configure input pads 
                                                          // (note that type is input, drive spec meaningless)
  ROM_SysCtlPeripheralEnable(HBR_OUTPUT_PORT);            // enable clock to GPIOD
                                                          // configure output pads 
  GPIOPinTypeGPIOOutput(HBR_OUTPUT_PORT_BASE, HBR_OUTPUT_PINS);
  
  
                                                          // configure PWM
  ROM_SysCtlPeripheralEnable(HBR_PWM_MODULE);             // enable clock to pwm module 0
  ROM_SysCtlPWMClockSet(HBR_PWM_FPWM_DIV);                // configure clock divider to derive Fpwm from Fsys
                                                          // wrap 16b PWM counter at 1041 for 3kHz pwm output
  ROM_PWMDeadBandDisable(HBR_PWM_BASE, HBR_PWM_GEN);      // allow PWM0, PWM1 to behave independently
  ROM_PWMGenConfigure(HBR_PWM_BASE, HBR_PWM_GEN,          // configure pwm generator
                      PWM_GEN_MODE_UP_DOWN |
                      PWM_GEN_MODE_SYNC |
                      PWM_GEN_MODE_DBG_STOP |
                      PWM_GEN_MODE_GEN_SYNC_LOCAL |
                      PWM_GEN_MODE_DB_NO_SYNC |
                      PWM_GEN_MODE_FAULT_UNLATCHED |
                      PWM_GEN_MODE_FAULT_NO_MINPER);
  ROM_PWMGenPeriodSet(HBR_PWM_BASE, HBR_PWM_GEN,          // sets period for generator to appropriate period
                      HBR_PWM_PERIOD_TICKS);
  ROM_PWMPulseWidthSet(HBR_PWM_BASE, HBR_1_PWM, 0);       // set initial pulse widths to 0 for safety
  ROM_PWMPulseWidthSet(HBR_PWM_BASE, HBR_2_PWM, 0);       // set initial pulse widths to 0 for safety
  
  ROM_GPIOPinConfigure(HBR_1_PWM_MUX);                    // select mux for pwm output hbridge 1
  ROM_GPIOPinConfigure(HBR_2_PWM_MUX);                    // select mux for pwm output hbridge 2
  ROM_GPIOPinTypePWM(HBR_OUTPUT_PORT_BASE, HBR_1_PWMH);   // do some other step configuring pwm...
  ROM_GPIOPinTypePWM(HBR_OUTPUT_PORT_BASE, HBR_2_PWMH);   // ...exact function is unknown
  
  ROM_PWMOutputState(HBR_PWM_BASE, HBR_1_PWM_BIT | HBR_2_PWM_BIT, true);
                                                          // enable outputs from pwm generator to pins
  ROM_PWMGenEnable(HBR_PWM_BASE, HBR_PWM_GEN);            // enable pwm output generator
  
  hbr_set_reset(HBR_LEFT, 0);
  hbr_set_reset(HBR_RIGHT, 0);
  hbr_set_pwml(HBR_LEFT, 1);
  hbr_set_pwml(HBR_RIGHT, 1);
  hbr_set_phase(HBR_LEFT, 0);
  hbr_set_phase(HBR_RIGHT, 0);
}

// * hbr_set_reset ************************************************************
// * update state of hbridge reset line                                       *
// * ucHbr should be one of: HBR_LEFT, HBR_RIGHT                              *
// * Assumes pins already set up (hbr_init()).                                *
// ****************************************************************************
void hbr_set_reset(unsigned char ucHbr, unsigned char new_state)
{
  unsigned char valtemp;                                    // holds value temporary
  valtemp = (new_state != 0) ? 0xFF : 0x00;                 // all 1s if true, 0s else
  if(ucHbr == 1)
    ROM_GPIOPinWrite(HBR_OUTPUT_PORT_BASE,HBR_1_RESET,valtemp & HBR_1_RESET);
  else if(ucHbr == 2)
    ROM_GPIOPinWrite(HBR_OUTPUT_PORT_BASE,HBR_2_RESET,valtemp & HBR_2_RESET);
}

// * hbr_set_pwml *************************************************************
// * update state of hbridge pwml line                                        *
// * ucHbr should be one of: HBR_LEFT, HBR_RIGHT                              *
// * Assumes pins already set up (hbr_init()).                                *
// ****************************************************************************
void hbr_set_pwml(unsigned char ucHbr, unsigned char new_state)
{
  unsigned char valtemp;                                    // holds value temporary
  valtemp = (new_state != 0) ? 0xFF : 0x00;                 // all 1s if true, 0s else
  if(ucHbr == 1)
    ROM_GPIOPinWrite(HBR_OUTPUT_PORT_BASE,HBR_1_PWML,valtemp & HBR_1_PWML);
  else if(ucHbr == 2)
    ROM_GPIOPinWrite(HBR_OUTPUT_PORT_BASE,HBR_2_PWML,valtemp & HBR_2_PWML);
}

// * hbr_set_phase ************************************************************
// * update state of hbridge phase line                                       *
// * ucHbr should be one of: HBR_LEFT, HBR_RIGHT                              *
// * Assumes pins already set up (hbr_init()).                                *
// ****************************************************************************
void hbr_set_phase(unsigned char ucHbr, unsigned char new_state)
{
  unsigned char valtemp;                                    // holds value temporary
  valtemp = (new_state != 0) ? 0xFF : 0x00;                 // all 1s if true, 0s else
  if(ucHbr == 1)
    ROM_GPIOPinWrite(HBR_OUTPUT_PORT_BASE,HBR_1_PHASE,valtemp & HBR_1_PHASE);
  else if(ucHbr == 2)
    ROM_GPIOPinWrite(HBR_OUTPUT_PORT_BASE,HBR_2_PHASE,valtemp & HBR_2_PHASE);
}

// * hbr_set_pulse ************************************************************
// * update state of hbridge phase line                                       *
// * ucHbr should be one of: HBR_LEFT, HBR_RIGHT                              *
// * ulWidth is in units of ticks and should not be larger than               *
// *                          HBR_PWM_PERIOD_TICKS - 1                        *
// * Assumes pins already set up (hbr_init()).                                *
// ****************************************************************************
void hbr_set_pulse(unsigned char ucHbr, unsigned long ulWidth)
{
  if(ulWidth > (HBR_PWM_PERIOD_TICKS - 1))                  // ceiling ulWidth at max legal value
    ulWidth = (HBR_PWM_PERIOD_TICKS - 1);
  
  if(ucHbr == 1)
      ROM_PWMPulseWidthSet(HBR_PWM_BASE, HBR_1_PWM, ulWidth);
                                                            // update ceilinged pulse width to appropriate hbridge
  else if(ucHbr == 2)
      ROM_PWMPulseWidthSet(HBR_PWM_BASE, HBR_2_PWM, ulWidth);
                                                            // update ceilinged pulse width to appropriate hbridge    

}

// * hbr_set_effort ***********************************************************
// * higher level control of hbridge, convenient for control applications     *
// * combines update of pulse width with direction of motor based on sign of  *
// * effort                                                                   *
// * ucHbr should be one of: HBR_LEFT, HBR_RIGHT                              *
// * ssEffort should inclusively lie within the range                         *
// *                      +/-(HBR_PWM_PERIOD_TICKS - 1)                       *
// * Assumes pins already set up (hbr_init()).                                *
// ****************************************************************************
void hbr_set_effort(unsigned char ucHbr, signed short ssEffort)
{
  if(ssEffort > (signed short)(HBR_PWM_PERIOD_TICKS - 1))           // bound ssEffort
    ssEffort = (signed short)(HBR_PWM_PERIOD_TICKS - 1);
  else if(ssEffort < -((signed short)(HBR_PWM_PERIOD_TICKS - 1)))
    ssEffort = -((signed short)(HBR_PWM_PERIOD_TICKS - 1));
  
  if(ssEffort < 0)                                                  // negative
  {
    hbr_set_phase(ucHbr, 0);                                        // set phase reverse
    ssEffort = -ssEffort;                                           // make positive
  }
  else                                                              // positive
  {
    hbr_set_phase(ucHbr, 1);                                        // set phase forward
                                                                    // ssEffort already kknown positive
  }
  hbr_set_pulse(ucHbr, (unsigned long)ssEffort);                    // update pulse width
}

// EOF