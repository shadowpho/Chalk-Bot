// imu.c - Interface for Pololu MinIMU-9 via I2C.
// Alex Suchko for ChalkBot
// November 21, 2011

// Includes

#include <string.h>           // access to memset
#include "inc/hw_types.h"     // types used throughout this file and it's libraries
#include "inc/hw_memmap.h"    // memory map, register base addresses
#include "driverlib/gpio.h"   // gpio abstraction library
#include "imu.h"              // definition file for this file
#include "driverlib/rom.h"    // definitions for rom functions
#include "driverlib/sysctl.h" // definitions for system control
#include "driverlib/interrupt.h"
                              // NVIC API
#include "inc/hw_ints.h"      // NVIC hardware interrupt vector enumerations/aliases
#include "driverlib/timer.h"  // timer interface driver
#include "utils/softi2c.h"    // software i2c emulation

// Type definitions
typedef void(*imu_i2c_xfer_done_t)(unsigned long);        // transfer complete callback type
                                                          // argument is number of bytes remaining, if nonzero, error occured

                                                          // Note, initialization adapted from "MinIMU-9-Arduino-AHRS" project 


// Local prototypes
void imu_catch_gyro(unsigned long bytes_remain);          // catch returned data from gyro request, follows imu_i2c_xfer_done_t
void imu_i2c_int_en(void);                                // enable i2c master interrupt
void imu_i2c_int_dis(void);                               // disable i2c master interrupt
unsigned char imu_i2c_start_transaction(unsigned char dev_address, unsigned char reg_address, unsigned char* data, unsigned int data_byte_count, tBoolean dir_is_receive, imu_i2c_xfer_done_t done_callback);
                                                          // start a send from LM4F to I2C slave device
void imu_i2c_abort_transaction(void);                     // abort transaction asap (i.e. on error)
void imu_i2c_complete_transaction(void);                  // normal completion of transaction

// Definitions

#define IMU_PORT        (SYSCTL_PERIPH_GPIOA)             // pin configuration
#define IMU_PORT_BASE   (GPIO_PORTA_BASE)
#define IMU_PINS        (GPIO_PIN_6 | GPIO_PIN_7)
#define IMU_SCL         (GPIO_PIN_6)
#define IMU_SDA         (GPIO_PIN_7)
#define IMU_TIM         (SYSCTL_PERIPH_TIMER2)
#define IMU_TIM_BASE    (TIMER2_BASE)                     // Note, timer A is used.

#define IMU_TIM_SCLRATE (10000)                           // desired scl frequency in Hz
#define IMU_TIM_INTRATE (IMU_TIM_SCLRATE * 4)             // for softi2c driver, (interrupt rate)/4 = SCL rate
#define IMU_TIM_INT_VECT (INT_TIMER2A)                    // interrupt vector

#define IMU_I2C_SYSTEM_FREQUENCY  (50000000UL)            // system clock frequency in Hz

                                                          // function return codes
#define IMU_RET_SUCCESS       (0)                         // success
#define IMU_RET_BUSY          (1)                         // bus is busy (one or both of SDA, SCL not high)
#define IMU_RET_INVALID_ARGS  (2)                         // invlaid arguments passed into function

#define IMU_ADDR_ACCEL        (0x18)                      // accelerometer address (LSM303DLM)
#define IMU_ADDR_MAG          (0x1E)                      // magnetometer address  (LSM303DLM)
#define IMU_ADDR_GYRO         (0x69)                      // gyroscope address     (L3G4200D )

#define IMU_GYRO_CTRL_REG1    (0x20)                      // gyro control reg 1
#define IMU_GYRO_CTRL_REG4    (0x23)                      // gyro control reg 4
#define IMU_GYRO_OUT_Z        (0x2C)                      // gyro z value register, TWO BYTES this points a low byte (little endian)
#define IMU_GYRO_AVERAGES     (10)                        // number of averages to take when finding zero point on gyro

                                                          // conversion factor assuming 60Hz integration rate
                                                          // full scale 2000dps (16bit 2's complement)
                                                          // sample period (1/60)s
                                                          // desired output units milli degrees (x1000)
                                                          // resulting conversion factor, 1.0172526042
#define IMU_GYRO_CONVERT_FACTOR (1.0172526042)
#define IMU_GYRO_YAW_RATE_CONVERT_FACTOR (61.03515625)    // calibrates to millidps


// Local Variables
static tSoftI2C g_sI2C;                                   // soft I2C state structure
tBoolean       imu_i2c_in_progress = false;               // I2C transaction in progress
tBoolean       imu_i2c_dir_is_receive;                    // true for receive false for transmit
tBoolean       imu_i2c_is_multibyte = true;               // true for multibyte transaction, false for single byte
unsigned char* imu_i2c_data_buff;                         // pointer to next byte in currently filling(Rx)/emptying(Tx) data buffer
unsigned long  imu_i2c_data_byte_count;                   // buffer byte count
unsigned char  imu_i2c_reg_address;                       // selected register address to operate on (7b)
tBoolean       imu_i2c_is_addressing;                     // phase of transmission is addressing registers in chip
unsigned char  imu_i2c_dev_address;                       // holds device address
imu_i2c_xfer_done_t xfer_done_cb;                         // holds transfer done callback function pointer

gyro_machine_state_t gyro_machine_state = GYRO_MACHINE_START_I2C;                  
                                                          // gyro machine state - handles gyro startup and operation - poll on fixed timebase
tBoolean txn_done;                                        // transaction done flag for gyro machine
unsigned long avg_count;                                  // counts number of averages to take when finding zero
signed long zero_point;                                   // zero point found from averaging at rest
signed long yaw_rate_millidps;                            // latest heading yaw rate in millidps
signed long heading_millidegrees;                         // absolute heading in signed millidegrees (0.001degree)

#pragma alignment=4
unsigned char  imu_buff[256];                             // buffer space for communicating with imu
#pragma alignment

// Functions

// * imu_init *****************************************************************
// * Setup pins and I2C interface to communicate with IMU (Pololu MinIMU-9).  *
// *                                                                          *
// * NOTE: Also called internally by imu_i2c_abort_transaction during reset   *
// *       to recover from NAK/error.                                         *
// *                                                                          *
// * Portions for initialization of softi2c library copied/modified from      *
// * "soft_i2c_atmel.c" example code.                                         *
// ****************************************************************************
void imu_init(void)
{
                                                          // setup pins for I2C--------------------
    SysCtlPeripheralEnable(IMU_PORT);                     // enable GPIO port for IMU
    SysCtlPeripheralEnable(IMU_TIM);                      // enable timer to use for I2C bit timing
    SysCtlPeripheralReset(IMU_TIM);                       // reset timer to clear any previous configuration
    GPIOPinTypeI2C(IMU_PORT_BASE, IMU_PINS);              // configure pins for I2C
    memset(&g_sI2C, 0, sizeof(g_sI2C));                   // clear record
    SoftI2CCallbackSet(&g_sI2C, imu_SoftI2CCallback);     // set callback function
    SoftI2CSCLGPIOSet(&g_sI2C, IMU_PORT_BASE, IMU_SCL);   // set SCL pin
    SoftI2CSDAGPIOSet(&g_sI2C, IMU_PORT_BASE, IMU_SDA);   // set SDA pin
    SoftI2CInit(&g_sI2C);                                 // initialize software I2C driver
    SoftI2CIntEnable(&g_sI2C);                            // enable callback from softi2c
    
                                                          // configure timer interrupt
                                                          // Note, (interrupt rate)/4 = SCL frequency
    TimerConfigure(IMU_TIM_BASE, TIMER_CFG_32_BIT_PER);   // configure timer for 32b operation
    TimerLoadSet(IMU_TIM_BASE, TIMER_A, SysCtlClockGet() / IMU_TIM_INTRATE);
                                                          // configure divider to yield desired interrupt rate
    TimerIntEnable(IMU_TIM_BASE, TIMER_TIMA_TIMEOUT);     // enable timer wrap interrupt
    TimerEnable(IMU_TIM_BASE, TIMER_A);                   // enable timer to start counting
    IntEnable(IMU_TIM_INT_VECT);                          // enable interrupt vector in NVIC
  
}

// * imu_SoftI2CCallback ******************************************************
// * Callback function for soft I2C driver to give us an event.               *
// *                                                                          *
// * Portions for initialization of softi2c library copied/modified from      *
// * "soft_i2c_atmel.c" example code.                                         *
// ****************************************************************************
void imu_SoftI2CCallback(void)
{   
  unsigned long errcode;                                  // holds error code
  
  SoftI2CIntClear(&g_sI2C);                               // clear softi2c callback

  if(!imu_i2c_in_progress)                                // we shouldn't be here
  {
    imu_i2c_int_dis();                                    // try to make sure we don't spuriously get back here again
    return;
  }
  errcode = SoftI2CErr(&g_sI2C);                          // grab error code from softi2c
  if(errcode != SOFTI2C_ERR_NONE)                         // if error
  {
                                                          // could be address nak or data nak
    imu_i2c_abort_transaction();                          // abort current transaction
  }
  else if(imu_i2c_is_addressing)                          // first time here, addressing phase done, now ready for data phase
  {
    imu_i2c_is_addressing = false;                        // handled. clear flag.
    if(imu_i2c_dir_is_receive)                            // data is to be received, need to turn direction around then start data phase
    {
      SoftI2CSlaveAddrSet(&g_sI2C, imu_i2c_dev_address, imu_i2c_dir_is_receive);
                                                          // update R/S'
      if(imu_i2c_is_multibyte == false)                   // single byte
        SoftI2CControl(&g_sI2C, SOFTI2C_CMD_SINGLE_RECEIVE);
                                                          // receive a single byte (START, TRANSMIT, STOP)
      else                                                // byte stream (>1 bytes)
      {
        SoftI2CControl(&g_sI2C, SOFTI2C_CMD_BURST_RECEIVE_START);
                                                          // start a multi byte receive cycle (START, TRANSMIT...)
                                                          // (receives first byte as well)
      }
    }
    else                                                  // data is to be sent, already in correct direction, can continue with data phase
    {
      SoftI2CDataPut(&g_sI2C, *imu_i2c_data_buff); 
                                                          // load first data byte
      if(imu_i2c_is_multibyte == false)                   // single byte
        SoftI2CControl(&g_sI2C, SOFTI2C_CMD_BURST_SEND_FINISH);
                                                          // single byte send cycle ((already started) TRANSMIT, STOP)
      else                                                // byte stream (>1 bytes)
      {
        SoftI2CControl(&g_sI2C, SOFTI2C_CMD_BURST_SEND_CONT);
                                                          // continue a multi byte send cycle ((already started) TRANSMIT...)
                                                          // (sends first byte as well)
      }
    }
  }
  else                                                    // no error, transaction completed successfully, continuing with data after addressing
  {
    if(imu_i2c_dir_is_receive)                            // received a byte
    {
      *imu_i2c_data_buff = (unsigned char)SoftI2CDataGet(&g_sI2C);
                                                          // grab byte from data register
      imu_i2c_data_buff++;                                // prepare for next byte
      imu_i2c_data_byte_count--;                          // another byte received
      if(imu_i2c_is_multibyte)                            // multibyte transaction
      {
        if(imu_i2c_data_byte_count == 1)                  // 1 Byte remaining
        {
          SoftI2CControl(&g_sI2C, SOFTI2C_CMD_BURST_RECEIVE_FINISH);
                                                          // receive one more byte
        }
        else if(imu_i2c_data_byte_count == 0)             // transfer complete
        {
          imu_i2c_complete_transaction();                 // complete current transaction
        }
        else                                              // more bytes to go
        {
          SoftI2CControl(&g_sI2C, SOFTI2C_CMD_BURST_RECEIVE_CONT);
                                                          // receive next byte
        }
      }
      else                                                // single byte transaction
      {
        imu_i2c_complete_transaction();                   // complete current transaction
      }
    }
    else                                                  // sent a byte
    {
      imu_i2c_data_buff++;                                // prepare to send next byte
      imu_i2c_data_byte_count--;                          // another byte sent
      if(imu_i2c_is_multibyte)                            // multibyte transaction
      {
        if(imu_i2c_data_byte_count > 0)                   // more byte(s) to go
        {
          SoftI2CDataPut(&g_sI2C, *imu_i2c_data_buff);
                                                          // load next data byte to register
          if(imu_i2c_data_byte_count == 1)                // one more byte to go
          {
            SoftI2CControl(&g_sI2C, SOFTI2C_CMD_BURST_SEND_FINISH);
                                                          // send last byte and stop
          }
          else                                            // more than one byte remaining
          {
            SoftI2CControl(&g_sI2C, SOFTI2C_CMD_BURST_SEND_CONT);
                                                          // send next byte and remain in send state
          }
        }
        else                                              // transfer complete
        {
          imu_i2c_complete_transaction();                 // complete current transaction
        }
      }
      else                                                // single byte transaction
      {
        imu_i2c_complete_transaction();                   // complete current transaction
      }
    }
  }

}

// * imu_poll_gyro ************************************************************
// * poll gyro machine                                                        *
// ****************************************************************************
void imu_poll_gyro(void)
{
  signed long calctemp;                                     // calculation temporary variable
  float floattemp;                                          // floating point temporary variable
  //retval = imu_i2c_start_transaction(IMU_ADDR_GYRO, 0x0F, imu_buff, 1, true, imu_catch_gyro);
  switch(gyro_machine_state)
  {
    case GYRO_MACHINE_START_I2C:                            // init I2C state
      {
        imu_init();                                         // initialize gyro interface

                                                            // start transactions to be caught by next state
        txn_done = false;                                   // transaction not done by default
        imu_buff[0] = 0x0F;                                 // load configuration
        imu_i2c_start_transaction(IMU_ADDR_GYRO, IMU_GYRO_CTRL_REG1, imu_buff, 1, false, imu_catch_gyro);
                                                            // start sending first configuration
        gyro_machine_state = GYRO_MACHINE_CONFIG_1;         // go to first configuration state        
      }
      break;
    case GYRO_MACHINE_CONFIG_1:                             // configuration 1 - init reg 1
      {
        //imu_i2c_start_transaction(unsigned char dev_address, unsigned char reg_address, unsigned char* data, unsigned int data_byte_count, tBoolean dir_is_receive, imu_i2c_xfer_done_t done_callback)
        if(txn_done)                                        // previous config done, next config
        {
          txn_done = false;                                   // transaction not done by default
          imu_buff[0] = 0x20;                                 // load configuration (FULL SCALE 2000degrees/second)
          imu_i2c_start_transaction(IMU_ADDR_GYRO, IMU_GYRO_CTRL_REG4, imu_buff, 1, false, imu_catch_gyro);
                                                              // start sending first configuration
          gyro_machine_state = GYRO_MACHINE_CONFIG_2;         // go to 2nd configuration state         
        }
      }
      break;
    case GYRO_MACHINE_CONFIG_2:                             // configuration 2 - init reg 4
      {
        if(txn_done)                                        // previous config done, next ask for first sample from gyro to start averaging at rest, find zero point
        {
          txn_done = false;                                   // transaction not done by default
          imu_i2c_start_transaction(IMU_ADDR_GYRO, IMU_GYRO_OUT_Z, imu_buff, 2, true, imu_catch_gyro);
                                                              // start asking for first read
          avg_count = IMU_GYRO_AVERAGES;                      // load down-counter with how many averages to take
          zero_point = 0;                                     // clear any previous zero point
          gyro_machine_state = GYRO_MACHINE_ZEROING;          // go to zeroing state
        }
      }
      break;
    case GYRO_MACHINE_ZEROING:                              // stationary condition averaging
      {
        if(txn_done)                                        // previous config done, next ask for first sample from gyro to start averaging at rest, find zero point
        {
          txn_done = false;                                   // transaction not done by default
          zero_point += ((signed long)(*(signed short*)imu_buff));
                                                              // accumulate previous rest value
          imu_i2c_start_transaction(IMU_ADDR_GYRO, IMU_GYRO_OUT_Z, imu_buff, 2, true, imu_catch_gyro);
                                                              // ask for next rest value
          avg_count--;                                        // another sample read
          if(avg_count == 0)
          {
            zero_point /= ((signed long)IMU_GYRO_AVERAGES);   // finish average calculation, now have zero point at rest, you can now move the gyro
            imu_i2c_start_transaction(IMU_ADDR_GYRO, IMU_GYRO_OUT_Z, imu_buff, 2, true, imu_catch_gyro);
                                                              // ask for next (possibly dynamic) value to begin integrating
            heading_millidegrees = 0;                         // initialize heading to zero
            gyro_machine_state = GYRO_MACHINE_RUNNING;        // go to running (integrating) state now that we have the zero point
          }
        }
      }
      break;
    case GYRO_MACHINE_RUNNING:                              // normal operation - integrating deltas
      {
        if(txn_done)                                        // previous config done, next ask for first sample from gyro to start averaging at rest, find zero point
        {
          txn_done = false;                                   // transaction not done by default
          calctemp = ((signed long)(*(signed short*)imu_buff));
                                                              // grab 2B 2's complement sample
          imu_i2c_start_transaction(IMU_ADDR_GYRO, IMU_GYRO_OUT_Z, imu_buff, 2, true, imu_catch_gyro);
                                                              // ask for next rest value
          floattemp = (float)(calctemp-zero_point);           // would be better to leave this value raw then convert later to avoid accumulating error but this is faster to implement for now, note subtract out DC offset
          yaw_rate_millidps = (signed long)(floattemp * ((float)IMU_GYRO_YAW_RATE_CONVERT_FACTOR));
                                                              // update yaw rate
          floattemp *= (float)IMU_GYRO_CONVERT_FACTOR;        // multiply delta by conversion factor
          heading_millidegrees += (signed long)floattemp;     // accumulate/integrate sample
          
        }
      }
      break;
    default:
      gyro_machine_state = GYRO_MACHINE_START_I2C;          // get machine back on track, should never be here
      break;
  }
}

// * imu_get_heading **********************************************************
// * gets heading in millidegrees relative to where robot started             *
// ****************************************************************************
signed long imu_get_heading(void)
{
  return(heading_millidegrees);
}

// * imu_get_yaw_rate *********************************************************
// * gets yaw rate in millidps                                                *
// ****************************************************************************
signed long imu_get_yaw_rate(void)
{
  return(yaw_rate_millidps);
}

// * imu_catch_gyro ***********************************************************
// * catch returned data from gyro request, follows imu_i2c_xfer_done_t       *
// ****************************************************************************
void imu_catch_gyro(unsigned long bytes_remain)
{
  //unsigned long retval;
  //retval = bytes_remain;
  txn_done = true;                                          // flag that transaction has completed
}

// * imu_tim_ISR **************************************************************
// * ISR for soft i2c tim interrupt.                                          *
// *                                                                          *
// * Portions for initialization of softi2c library copied/modified from      *
// * "soft_i2c_atmel.c" example code.                                         *
// ****************************************************************************
void imu_tim_ISR(void)
{
  TimerIntClear(IMU_TIM_BASE, TIMER_TIMA_TIMEOUT);          // clear interrupt
  
  SoftI2CTimerTick(&g_sI2C);                                // perform softi2c tick update
}

// * imu_i2c_int_en ***********************************************************
// * Enables interrupt for I2C master, assumes NVIC global enable set         *
// * prior to call of this function.                                          *                                
// ****************************************************************************
void imu_i2c_int_en(void)
{
  SoftI2CIntEnable(&g_sI2C);                                // enable softi2c callback
}

// * imu_i2c_int_dis **********************************************************
// * Disable interrupt for I2C master.                                        *
// ****************************************************************************
void imu_i2c_int_dis(void)
{
  SoftI2CIntDisable(&g_sI2C);                               // disable softi2c callback
}

// * imu_i2c_start_transaction ************************************************
// * Start a receive/send' from LM4F to I2C slave device.                     *
// * dev_address    should be a 7b right justified number                     *
// * done_callback  Will be called when txn done, argument of # bytes remain, *
// *                if # bytes remain nonzero, an error occured.              *
// * Returns 0 for success, 1 for bus busy (send could not be started now).   *
// *                                                                          *
// * Portions for initialization of softi2c library copied/modified from      *
// * "soft_i2c_atmel.c" example code.                                         *
// ****************************************************************************
unsigned char imu_i2c_start_transaction(unsigned char dev_address, unsigned char reg_address, unsigned char* data, unsigned int data_byte_count, tBoolean dir_is_receive, imu_i2c_xfer_done_t done_callback)
{
  if(data_byte_count == 0 || data == 0 || done_callback == 0)
  {                                                       // no data to send or no place to take it from
    return IMU_RET_INVALID_ARGS;                          // invalid argument(s)
  }
  else if(SoftI2CBusy(&g_sI2C))                           // if bus busy
  {
    return IMU_RET_BUSY;                                  // return bus busy
  }
  xfer_done_cb = done_callback;                           // store callback function pointer
  imu_i2c_dev_address = dev_address;                      // store copy of device address for use in interrupt handler
  imu_i2c_reg_address = reg_address;                      // store register address
  SoftI2CSlaveAddrSet(&g_sI2C, dev_address, false);  
                                                          // set the slave address, always transmit first to get register address transmitted
  imu_i2c_dir_is_receive = dir_is_receive;                // make copy of current direction
  imu_i2c_data_buff = data;                               // initialize buffer pointer
  imu_i2c_data_byte_count = data_byte_count;              // initialize data byte count - will downcount during send
  imu_i2c_int_en();                                       // enable 'interrupts' (callbacks) for I2C
  imu_i2c_is_addressing = true;                           // first interrupt will be to finish register addressing stage
  imu_i2c_in_progress = true;                             // transaction is currently taking place
  imu_i2c_is_multibyte = false;                           // default to single byte
  if(data_byte_count > 1)
  {
    imu_i2c_reg_address |= 0x80;                          // turn msb on to indicate sequential read to chip
    imu_i2c_is_multibyte = true;                          // multi byte txn
  }
                                                          // start txn with address and send register address
  SoftI2CDataPut(&g_sI2C, imu_i2c_reg_address);           // load register address as data
  SoftI2CControl(&g_sI2C, SOFTI2C_CMD_BURST_SEND_START);  // instruct softi2c to generate start, send dev address, and 'data' (register address in part)
  
  return IMU_RET_SUCCESS;                                 // successfully started transfer of register address!
                                                          // now we will wait for interrupt to see what happened
}

// * imu_i2c_abort_transaction ************************************************
// * Aborts transaction in progress, used to quit xaction on error.           *
// ****************************************************************************
void imu_i2c_abort_transaction(void)
{
  imu_i2c_int_dis();                                      // disable interrupts
  imu_init();                                             // re-init i2c
  imu_i2c_in_progress = false;                            // transaction aborted
  xfer_done_cb(imu_i2c_data_byte_count);                  // call user back with number of bytes remaining, txn done
}

// * imu_i2c_complete_transaction *********************************************
// * Normal completion of a transaction.                                      *
// ****************************************************************************
void imu_i2c_complete_transaction(void)
{
  imu_i2c_int_dis();                                    // disable interrupts
  imu_i2c_in_progress = false;                          // transaction completed
  xfer_done_cb(imu_i2c_data_byte_count);                // call user back with number of bytes remaining, txn done
                                                        // should be zero in this case
}

// Note, I2C in master mode will generate an interrupt on: Send complete, Receive complete, Aborted due to error
// No DMA channel for I2c, write new byte each interrupt (at 100Kbps streaming, interrupt rate ~11kHz (9b/B)).

// EOF