// encoder.c - Interface for 2ch of Quadrature Encoder Interface (QEI) to the encoders.
// Alex Suchko for ChalkBot
// November 20,2011

// Includes

#include "inc/hw_types.h"     // types used throughout this file and it's libraries
#include "inc/hw_memmap.h"    // memory map, register base addresses
#include "driverlib/gpio.h"   // gpio abstraction library
#include "driverlib/qei.h"    // qei abstraction library
#include "encoder.h"              // definition file for this file
#include "inc/hw_qei.h"       // hardware register definitions, used to perform addiitonal QEI setup
#include "driverlib/rom.h"    // definitions for rom functions
#include "driverlib/sysctl.h" // definitions for system control
#include "driverlib/interrupt.h"
                              // NVIC API
#include "inc/hw_ints.h"      // NVIC hardware interrupt vector enumerations/aliases
#include "hbridge.h"          // for debug I/O

// Definitions

                                                            // ENC_1-------------------------------
#define ENC_1_PORT        (SYSCTL_PERIPH_GPIOC)             // pin configuration
#define ENC_1_PORT_BASE   (GPIO_PORTC_BASE)
#define ENC_1_PINS        (GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4)
#define ENC_1_B           (GPIO_PIN_6)
#define ENC_1_A           (GPIO_PIN_5)
#define ENC_1_INDEX       (GPIO_PIN_4)
#define ENC_1_B_MUX       (GPIO_PC6_PHB1)
#define ENC_1_A_MUX       (GPIO_PC5_PHA1)
#define ENC_1_INDEX_MUX   (GPIO_PC4_IDX1)
#define ENC_1_MODULE      (SYSCTL_PERIPH2_QEI1)             // qei module configuration
#define ENC_1_BASE        (QEI1_BASE)

                                                            // ENC_2-------------------------------
#define ENC_2_PORT        (SYSCTL_PERIPH_GPIOF)
#define ENC_2_PORT_BASE   (GPIO_PORTF_BASE)
#define ENC_2_PINS        (GPIO_PIN_1 | GPIO_PIN_4)// used to include GPIO_PIN_0, dead pin.
#define ENC_2_B           (GPIO_PIN_1)
//#define ENC_2_A           (GPIO_PIN_0)// had dead pin PF0. Now sensed by PD6
#define ENC_2_INDEX       (GPIO_PIN_4)
#define ENC_2_B_MUX       (GPIO_PF1_PHB0)
//#define ENC_2_A_MUX       (GPIO_PF0_PHA0)// had dead pin PF0. Now sensed by PD6
#define ENC_2_INDEX_MUX   (GPIO_PF4_IDX0)

#define ENC_2_PORT_2      (SYSCTL_PERIPH_GPIOD)             // workaround for dead pin PF0
#define ENC_2_PORT_BASE_2 (GPIO_PORTD_BASE)
#define ENC_2_PINS_2      (GPIO_PIN_6)
#define ENC_2_A           (GPIO_PIN_6)                      // was PF0
#define ENC_2_A_MUX       (GPIO_PD6_PHA0)                   // was PF0

#define ENC_2_MODULE      (SYSCTL_PERIPH2_QEI0)             // qei module configuration
#define ENC_2_BASE        (QEI0_BASE)
#define ENC_2_INT_FLAGS   (QEI_INTTIMER)                    // qei interrupt configuration
#define ENC_2_INTVECT     (INT_QEI0)

                                                            // Shared Config-----------------------
#define ENC_MAX_POSITION  (0xFFFFFFFF)                      // Rollover value, using this value makes
                                                            // encoder position behave like 2's complement
                                                            // (0 - 1 => 0xFFFFFFFF (i.e. -1)).
#define ENC_VEL_PERIOD    (833333)                          // (desired_period_in_sys_clock_cycles - 1)
                                                            // This period determines the gating for the
                                                            // hardware velocity counter, note value reflects
                                                            // magnitude only, read direction bit as well.
                                                            // Note, hardware seems to be doing a -1 on load? +1 here.
#define ENC_FILT_CYC      (0xF)                             // 4b field (unsigned and no greater than 0xF)
                                                            // (Specify number of system clock cycles for
                                                            // edge to persist before edge detecting) - 1
                                                            // (i.e. 0xF yields 17 clock cycles).
#define ENC_FILT_ENA_BITS (((ENC_FILT_CYC << 16) & QEI_CTL_FILTCNT_M) | QEI_CTL_FILTEN)

// Variables

// Functions

// * enc_init *****************************************************************
// * setup pins and PWM hardware to talk to servomotors                       *
// * Assumes system clock already configured                                  *
// ****************************************************************************
void enc_init(void)
{
  unsigned long config_temp;                              // used when assembling configuration values
                                                          // setup pins for QEI
                                                          // ENC_1-------------------------------
  ROM_SysCtlPeripheralEnable(ENC_1_PORT);                 // enable clock to GPIO port
                                                          // configure input pads 
  ROM_GPIOPinConfigure(ENC_1_B_MUX);                      // select mux for qei input
  ROM_GPIOPinConfigure(ENC_1_A_MUX);                      // select mux for qei input
  ROM_GPIOPinConfigure(ENC_1_INDEX_MUX);                  // select mux for qei input
                                                          // copy and pasted from GPIOPinTypeQEI,
                                                          // modified to have no pull up/down resistors.
  GPIODirModeSet(ENC_1_PORT_BASE, ENC_1_PINS, GPIO_DIR_MODE_HW); 
                                                          // QEI pin tristates under HW control.
  GPIOPadConfigSet(ENC_1_PORT_BASE, ENC_1_PINS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
                                                          // Note, output drive config only configs
                                                          // pull up/down resistors when pin is input.
  
                                                          // ENC_2-------------------------------
  ROM_SysCtlPeripheralEnable(ENC_2_PORT);                 // enable clock to GPIO port
                                                          // configure input pads 
  ROM_GPIOPinConfigure(ENC_2_B_MUX);                      // select mux for qei input
  //ROM_GPIOPinConfigure(ENC_2_A_MUX);                      // select mux for qei input WORKAROUND FOR DEAD PIN
  ROM_GPIOPinConfigure(ENC_2_INDEX_MUX);                  // select mux for qei input
                                                          // copy and pasted from GPIOPinTypeQEI,
                                                          // modified to have no pull up/down resistors.
  GPIODirModeSet(ENC_2_PORT_BASE, ENC_2_PINS, GPIO_DIR_MODE_HW); 
                                                          // QEI pin tristates under HW control.
  GPIOPadConfigSet(ENC_2_PORT_BASE, ENC_2_PINS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
  //GPIODirModeSet(ENC_2_PORT_BASE, ENC_2_PINS, GPIO_DIR_MODE_IN);//DEBUG
  //DEBUG
                                                          // Note, output drive config only configs
                                                          // pull up/down resistors when pin is input.
  
                                                          // ENC_2 DEAD PIN WORKAROUND SETUP-----
  ROM_SysCtlPeripheralEnable(ENC_2_PORT_2);               // enable clock to GPIO port
                                                          // configure input pads 
  ROM_GPIOPinConfigure(ENC_2_A_MUX);                      // select mux for qei input WORKAROUND FOR DEAD PIN
                                                          // copy and pasted from GPIOPinTypeQEI,
                                                          // modified to have no pull up/down resistors.
  GPIODirModeSet(ENC_2_PORT_BASE_2, ENC_2_PINS_2, GPIO_DIR_MODE_HW); 
                                                          // QEI pin tristates under HW control.
  GPIOPadConfigSet(ENC_2_PORT_BASE_2, ENC_2_PINS_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
  
                                                          // setup QEI modules
                                                          // ENC_1-------------------------------
  ROM_SysCtlPeripheralEnable(ENC_1_MODULE);               // enable clock to QEI
  ROM_QEIConfigure(ENC_1_BASE, (QEI_CONFIG_CAPTURE_A_B |  // configure QEI
                                QEI_CONFIG_NO_RESET    |
                                QEI_CONFIG_QUADRATURE  |
                                QEI_CONFIG_NO_SWAP),
                    ENC_MAX_POSITION);
  ROM_QEIVelocityConfigure(ENC_1_BASE,QEI_VELDIV_1,       // configure hardware velocity capture
                           ENC_VEL_PERIOD);
  ROM_QEIVelocityEnable(ENC_1_BASE);                      // enable velocity capture
                                                          // enable digital filter on input
  config_temp = HWREG(ENC_1_BASE + QEI_O_CTL);            // grab copy of the QEI's control register
  config_temp |= ENC_FILT_ENA_BITS;                       // configure and enable digital persistence filter
  HWREG(ENC_1_BASE + QEI_O_CTL) = config_temp;            // write modified value back to register
  
                                                          // ENC_2-------------------------------
  ROM_SysCtlPeripheralEnable(ENC_2_MODULE);               // enable clock to QEI
  ROM_QEIConfigure(ENC_2_BASE, (QEI_CONFIG_CAPTURE_A_B |  // configure QEI
                                QEI_CONFIG_NO_RESET    |
                                QEI_CONFIG_QUADRATURE  |
                                QEI_CONFIG_NO_SWAP),
                    ENC_MAX_POSITION);
  ROM_QEIVelocityConfigure(ENC_2_BASE,QEI_VELDIV_1,       // configure hardware velocity capture
                           ENC_VEL_PERIOD);
  ROM_QEIVelocityEnable(ENC_2_BASE);                      // enable velocity capture
                                                          // enable digital filter on input
  config_temp = HWREG(ENC_2_BASE + QEI_O_CTL);            // grab copy of the QEI's control register
  config_temp |= ENC_FILT_ENA_BITS;                       // configure and enable digital persistence filter
  HWREG(ENC_2_BASE + QEI_O_CTL) = config_temp;            // write modified value back to register


  ROM_QEIEnable(ENC_1_BASE);                              // Enable one QEI after the other.
  ROM_QEIEnable(ENC_2_BASE);                              // Try to keep sync, later we will assume ENC_2
                                                          // velocity capture finishes just after ENC_1,
                                                          // so we will interrupt off ENC_2's velocity timer.
}

// * enc_2_int_init ***********************************************************
// * setup interrupt from qei                                                 *
// * Assumes system clock already configured                                  *
// ****************************************************************************
void enc_2_int_init(void)
{
  ROM_IntMasterEnable();                                  // enable NVIC interrupts
  ROM_QEIIntEnable(ENC_2_BASE, ENC_2_INT_FLAGS);          // enable interrupt signal(s)
  ROM_IntEnable(ENC_2_INTVECT);                           // enable NVIC vector for our module
                                                          // interrupt now active, remember to clear on interrupt
}

// * enc_2_ISR ****************************************************************
// * ISR for qei, registered to qei associated with ENC_2                     *
// ****************************************************************************
void enc_2_ISR(void)
{
  static unsigned char toggle = 0;    // test code
                                      // for now the only thing that can get us here
                                      // is the timer rollover interrupt
  ROM_QEIIntClear(ENC_2_BASE, ENC_2_INT_FLAGS);
  toggle ^= 0xFF;                     // test code
  if(toggle)
  {
    hbr_set_reset(HBR_LEFT, 1);
  }
  else
  {
    hbr_set_reset(HBR_LEFT, 0);
  }
}
  

// EOF