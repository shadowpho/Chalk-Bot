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

// Local Variables

tBoolean enc_period_expired = false;                        // used so low-accuracy timing routines can poll for end of velocity timing
                                                            // set true on interrupt

// Functions

// * enc_init *****************************************************************
// * Setup pins and hardware to read quadrture encoders. Setup causes each    *
// * encoder line to result in 4 pulses, i.e. a 100line encoder would yield   *
// * 400 pulses/revolution.                                                   *
// * Assumes system clock already configured                                  *
// ****************************************************************************
void enc_init(void)
{
  unsigned long config_temp;                              // used when assembling configuration values
                                                          // setup pins for QEI
                                                          // ENC_1-------------------------------
  ROM_SysCtlPeripheralEnable(ENC_1_PORT);                 // enable clock to GPIO port
  //ROM_SysCtlPeripheralReset(ENC_1_PORT);                  // reset to clear any previous configuration
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
  //ROM_SysCtlPeripheralReset(ENC_2_PORT);                  // reset to clear any previous configuration
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
//  static unsigned char toggle = 0;    // test code
//                                      // for now the only thing that can get us here
//                                      // is the timer rollover interrupt
  ROM_QEIIntClear(ENC_2_BASE, ENC_2_INT_FLAGS);
  enc_period_expired = true;            // flag that the velocity period has expired
//  toggle ^= 0xFF;                     // test code
//  if(toggle)
//  {
//    hbr_set_reset(HBR_LEFT, 1);
//  }
//  else
//  {
//    hbr_set_reset(HBR_LEFT, 0);
//  }
}

// * enc_dir_reverse **********************************************************
// * Enables/disables direction reversing for specified encoder.              *
// * NOTE: Changing this setting affects subsequent encoder motion only, it   *
// *       does not invert the sign on any pre-existing velocity or position. *
// * encoder should be ENC_1 or ENC_2.                                        *
// * reverse is considered true if !=0, false if ==0.                         *
// ****************************************************************************
void enc_dir_reverse(unsigned char encoder, unsigned char reverse)
{
  unsigned long configtemp;                                 // used while modifying configuration
  unsigned long* base_addr;                                 // pointer to base of appropriate control register
  
  if     (encoder == ENC_1) base_addr = (unsigned long*)ENC_1_BASE;         
                                                            // select encoder 1
  else if(encoder == ENC_2) base_addr = (unsigned long*)ENC_2_BASE;         
                                                            // select encoder 2
  else                      return;                         // encoder ID was not valid selection
  
  configtemp = HWREG(base_addr + QEI_O_CTL);                // grab value of appropriate config register
  if(reverse != 0)                                          // enable reversing
  {
    configtemp |= QEI_CTL_SWAP;                             // enable swap
  }
  else                                                      // disable reversing
  {
    configtemp &= ~QEI_CTL_SWAP;                            // disable swap
  }
  HWREG(base_addr + QEI_O_CTL) = configtemp;                // write modified value back to control reg
}

// * enc_pos_get **************************************************************
// * Returns the signed position of the encoder, in units of encoder pulses.  *
// * encoder should be ENC_1 or ENC_2.                                        *
// ****************************************************************************
signed long enc_pos_get(unsigned char encoder)
{
  signed long retval;                                       // holds value to be returned
  if(encoder == ENC_1)
  {
    retval = (signed long)QEIPositionGet(ENC_1_BASE);       // return encoder 1 reading
  }
  else if(encoder == ENC_2)
  {
    retval = (signed long)QEIPositionGet(ENC_2_BASE);       // return encoder 2 reading
  }
  else
  {
    retval = 0;                                             // return for invalid encoder ID
  }
  return retval;                                            // return the encoder position
}

// * enc_pos_set **************************************************************
// * Sets the signed position of the encoder, in units of encoder pulses.     *
// * encoder should be ENC_1 or ENC_2.                                        *
// ****************************************************************************
void enc_pos_set(unsigned char encoder, signed long position)
{
  if(encoder == ENC_1)
  {
    QEIPositionSet(ENC_1_BASE, (unsigned long)position);    // return encoder 1 reading
  }
  else if(encoder == ENC_2)
  {
    QEIPositionSet(ENC_2_BASE, (unsigned long)position);    // return encoder 2 reading
  }
                                                            // do nothing for invalid encoder ID
}

// * enc_vel_get **************************************************************
// * Returns the signed velocity of the encoder, in units of encoder pulses.  *
// * NOTE: If a change in the direction of rotation occured during the last   *
// *       velocity sampling period, this value will reflect the magnitude    *
// *       of displacement and it's sign will refer to the direction of       *
// *       encoder rotation at the end of the sampling period.                *
// *                                                                          *
// *       This yields undesirable results during direction changes, which    *
// *       may be unacceptable in your application. If this is the case,      *
// *       you should do your own first derivative calculation by sampling    *
// *       encoder position on a known timebase, and finding the delta from   *
// *       the previous sample.                                               *
// * encoder should be ENC_1 or ENC_2.                                        *
// ****************************************************************************
signed long enc_vel_get(unsigned char encoder)
{
  signed long retval;                                       // holds value to be returned
  if(encoder == ENC_1)
  {
    retval = (signed long)QEIVelocityGet(ENC_1_BASE);       // return encoder 1 reading
    retval *= QEIDirectionGet(ENC_1_BASE);                  // returns direction as -1 or +1
  }
  else if(encoder == ENC_2)
  {
    retval = (signed long)QEIVelocityGet(ENC_2_BASE);       // return encoder 2 reading
    retval *= QEIDirectionGet(ENC_2_BASE);                  // returns direction as -1 or +1    
 }
  else
  {
    retval = 0;                                             // return for invalid encoder ID
  }
  return retval;                                            // return the encoder position
}

// * enc_poll_period_expire ***************************************************
// * Polls for if the timer period has expired. Returns 1 once after each     *
// * velocity timer expiration (16.67ms 60.00Hz), else 0.                     *
// ****************************************************************************
unsigned char enc_poll_period_expire(void)
{
  if(enc_period_expired)                                    // if timer period has expired since previous sample
  {
    enc_period_expired = false;                             // clear flag, this is one-shot per period expiration
    return true;                                            // return true, the period has newly expired
  }
  else                                                      // timer has not expired since previous sample
  {
    return false;                                           // return false, period has not newly expired
  }
}

// EOF