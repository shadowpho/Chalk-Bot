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

// Type definitions
typedef void(*imu_i2c_xfer_done_t)(unsigned long);        // transfer complete callback type
                                                          // argument is number of bytes remaining, if nonzero, error occured

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
#define IMU_SCL_MUX     (GPIO_PA6_I2C1SCL)
#define IMU_SDA_MUX     (GPIO_PA7_I2C1SDA)

#define IMU_I2C_BASE    (I2C1_MASTER_BASE)                // I2C configuration
#define IMU_I2C_INTVECT (INT_I2C1)
#define IMU_I2C_TIMEOUT (0x7D)                            // timeout reload value for clock low timeout
                                                          // value given in StellarisWare manual
                                                          // note this sets upper 8b of 12b counter on SCL clock
                                                          // so it counts at the SCL rate (for low speed, 100kHz)
                                                          // example if 20ms desired, 20ms * 100KHz = 2000 = 0x7D0
                                                          //                          truncate bottom 4b -> 0x7D

                                                          // function return codes
#define IMU_RET_SUCCESS       (0)                         // success
#define IMU_RET_BUSY          (1)                         // bus is busy (one or both of SDA, SCL not high)
#define IMU_RET_INVALID_ARGS  (2)                         // invlaid arguments passed into function

#define IMU_ADDR_ACCEL        (0x18)                      // accelerometer address (LSM303DLM)
#define IMU_ADDR_MAG          (0x1E)                      // magnetometer address  (LSM303DLM)
#define IMU_ADDR_GYRO         (0x69)                      // gyroscope address     (L3G4200D )


// Local Variables
tBoolean       imu_i2c_in_progress = false;               // I2C transaction in progress
tBoolean       imu_i2c_dir_is_receive;                    // true for receive false for transmit
tBoolean       imu_i2c_is_multibyte = true;               // true for multibyte transaction, false for single byte
unsigned char* imu_i2c_data_buff;                         // pointer to next byte in currently filling(Rx)/emptying(Tx) data buffer
unsigned long  imu_i2c_data_byte_count;                   // buffer byte count
unsigned char  imu_i2c_reg_address;                       // selected register address to operate on (7b)
tBoolean       imu_i2c_is_addressing;                     // phase of transmission is addressing registers in chip
unsigned char  imu_i2c_dev_address;                       // holds device address
imu_i2c_xfer_done_t xfer_done_cb;                         // holds transfer done callback function pointer

unsigned char  imu_buff[256];                             // buffer space for communicating with imu

// Functions

// * imu_init *****************************************************************
// * Setup pins and I2C interface to communicate with IMU (Pololu MinIMU-9).  *
// ****************************************************************************
void imu_init(void)
{
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
  ROM_I2CMasterTimeoutSet(IMU_I2C_BASE, IMU_I2C_TIMEOUT); // load the reset value for the clock low timeout
                                                          // enable I2C master interrupt-----------
  ROM_IntMasterEnable();                                  // global enable NVIC interrupts
                                                          // I2C interrupts will be enabled on demand
}

// * imu_poll_gyro ************************************************************
// * poll gyro                                                                *
// ****************************************************************************
void imu_poll_gyro(void)
{
  unsigned char retval;                                   // will hold return value from start function call
  retval = imu_i2c_start_transaction(IMU_ADDR_GYRO, 0x0F, imu_buff, 1, true, imu_catch_gyro);
}

// * imu_catch_gyro ***********************************************************
// * catch returned data from gyro request, follows imu_i2c_xfer_done_t       *
// ****************************************************************************
void imu_catch_gyro(unsigned long bytes_remain)
{
  unsigned long retval;
  retval = bytes_remain;
  
}

// * imu_i2c_ISR **************************************************************
// * ISR for I2C master mode interrupt.                                       *
// ****************************************************************************
void imu_i2c_ISR(void)
{
  unsigned long intstat;                                    // holds interrupt status flags
  unsigned long errcode;                                    // holds error code
  intstat = ROM_I2CMasterIntStatusEx(IMU_I2C_BASE, true);   // grab current masked interrupt status
  ROM_I2CMasterIntClearEx(IMU_I2C_BASE, I2C_MASTER_INT_TIMEOUT | I2C_MASTER_INT_DATA);
                                                            // clear interrupts
  if(!imu_i2c_in_progress)                                  // we shouldn't be here
  {
    imu_i2c_int_dis();                                      // try to make sure we don't spuriously get back here again
    return;
  }
  if(intstat & I2C_MASTER_INT_DATA)                         // interrupt due to transaction complete with or without error
  {
    errcode = ROM_I2CMasterErr(IMU_I2C_BASE);               // grab error code from I2C unit
    if(errcode != I2C_MASTER_ERR_NONE)                      // if error
    {
                                                            // could be address nak, data nak, lost arbitration (multi-master I2C networks only)
      imu_i2c_abort_transaction();                          // abort current transaction
    }
    else if(imu_i2c_is_addressing)                          // first time here, addressing phase done, now ready for data phase
    {
      imu_i2c_is_addressing = false;                        // handled. clear flag.
      if(imu_i2c_dir_is_receive)                            // data is to be received, need to turn direction around then start data phase
      {
        ROM_I2CMasterSlaveAddrSet(IMU_I2C_BASE, imu_i2c_dev_address, imu_i2c_dir_is_receive);
                                                            // update R/S'
        if(imu_i2c_is_multibyte == false)                   // single byte
          ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
                                                            // receive a single byte (START, TRANSMIT, STOP)
        else                                                // byte stream (>1 bytes)
        {
          ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
                                                            // start a multi byte receive cycle (START, TRANSMIT...)
                                                            // (receives first byte as well)
        }
      }
      else                                                  // data is to be sent, already in correct direction, can continue with data phase
      {
        ROM_I2CMasterDataPut(IMU_I2C_BASE, *imu_i2c_data_buff); 
                                                            // load first data byte
        if(imu_i2c_is_multibyte == false)                   // single byte
          ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
                                                            // single byte send cycle ((already started) TRANSMIT, STOP)
        else                                                // byte stream (>1 bytes)
        {
          ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
                                                            // continue a multi byte send cycle ((already started) TRANSMIT...)
                                                            // (sends first byte as well)
        }
      }
    }
    else                                                    // no error, transaction completed successfully
    {
      if(imu_i2c_dir_is_receive)                            // received a byte
      {
        *imu_i2c_data_buff = (unsigned char)ROM_I2CMasterDataGet(IMU_I2C_BASE);
                                                            // grab byte from data register
        imu_i2c_data_buff++;                                // prepare for next byte
        imu_i2c_data_byte_count--;                          // another byte received
        if(imu_i2c_is_multibyte)                            // multibyte transaction
        {
          if(imu_i2c_data_byte_count == 1)                  // 1 Byte remaining
          {
            ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                                                            // receive one more byte
          }
          else if(imu_i2c_data_byte_count == 0)             // transfer complete
          {
            imu_i2c_complete_transaction();                 // complete current transaction
          }
          else                                              // more bytes to go
          {
            ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
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
            ROM_I2CMasterDataPut(IMU_I2C_BASE, *imu_i2c_data_buff);
                                                            // load next data byte to register
            if(imu_i2c_data_byte_count == 1)                // one more byte to go
            {
              ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
                                                            // send last byte and stop
            }
            else                                            // more than one byte remaining
            {
              ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
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
  else if(intstat & I2C_MASTER_INT_TIMEOUT)                 // interrupt due to timeout condition
  {
    imu_i2c_abort_transaction();                            // abort current transaction
  }
}

// * imu_i2c_int_en ***********************************************************
// * Enables interrupt for I2C master, assumes NVIC global enable set         *
// * prior to call of this function.                                          *                                
// ****************************************************************************
void imu_i2c_int_en(void)
{
   ROM_I2CMasterIntEnableEx(IMU_I2C_BASE, I2C_MASTER_INT_TIMEOUT | I2C_MASTER_INT_DATA);          
                                                            // enable timeout and data interrupt signals from I2C
   ROM_IntEnable(IMU_I2C_INTVECT);                          // enable NVIC vector for our module
                                                            // interrupt now active, remember to clear on interrupt
}

// * imu_i2c_int_dis **********************************************************
// * Disable interrupt for I2C master.                                        *
// ****************************************************************************
void imu_i2c_int_dis(void)
{
   ROM_IntDisable(IMU_I2C_INTVECT);                         // enable NVIC vector for our module
   ROM_I2CMasterIntDisableEx(IMU_I2C_BASE, I2C_MASTER_INT_TIMEOUT | I2C_MASTER_INT_DATA);          
                                                            // enable timeout and data interrupt signals from I2C
                                                            // interrupt now inactive
}

// * imu_i2c_start_transaction ************************************************
// * Start a receive/send' from LM4F to I2C slave device.                     *
// * dev_address    should be a 7b right justified number                     *
// * done_callback  Will be called when txn done, argument of # bytes remain, *
// *                if # bytes remain nonzero, an error occured.              *
// * Returns 0 for success, 1 for bus busy (send could not be started now).   *
// ****************************************************************************
unsigned char imu_i2c_start_transaction(unsigned char dev_address, unsigned char reg_address, unsigned char* data, unsigned int data_byte_count, tBoolean dir_is_receive, imu_i2c_xfer_done_t done_callback)
{
  if(data_byte_count == 0 || data == 0 || done_callback == 0)
  {                                                       // no data to send or no place to take it from
    return IMU_RET_INVALID_ARGS;                          // invalid argument(s)
  }
  else if(ROM_I2CMasterBusBusy(IMU_I2C_BASE))             // if bus busy
  {
    return IMU_RET_BUSY;                                  // return bus busy
  }
  xfer_done_cb = done_callback;                           // store callback function pointer
  imu_i2c_dev_address = dev_address;                      // store copy of device address for use in interrupt handler
  imu_i2c_reg_address = reg_address;                      // store register address
  ROM_I2CMasterSlaveAddrSet(IMU_I2C_BASE, dev_address, false);
                                                          // set the slave address, always transmit first to get register address transmitted
  imu_i2c_dir_is_receive = dir_is_receive;                // make copy of current direction
  imu_i2c_data_buff = data;                               // initialize buffer pointer
  imu_i2c_data_byte_count = data_byte_count;              // initialize data byte count - will downcount during send
  imu_i2c_int_en();                                       // enable interrupts for I2C
  imu_i2c_is_addressing = true;                           // first interrupt will be to finish register addressing stage
  imu_i2c_is_multibyte = false;                           // default to single byte
  if(data_byte_count > 1)
  {
    imu_i2c_reg_address |= 0x80;                          // turn msb on to indicate sequential read to chip
    imu_i2c_is_multibyte = true;                          // multi byte txn
  }
                                                          // start txn with address and send register address
  ROM_I2CMasterDataPut(IMU_I2C_BASE, imu_i2c_reg_address);// load register address as data
  ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
                                                          // instruct hardware to generate start, send dev address, and 'data' (register address in part)
  
  imu_i2c_in_progress = true;                             // transaction is currently taking place
  
  return IMU_RET_SUCCESS;                                 // successfully started transfer of register address!
                                                          // now we will wait for interrupt to see what happened
}

// * imu_i2c_abort_transaction ************************************************
// * Aborts transaction in progress, used to quit xaction on error.           *
// ****************************************************************************
void imu_i2c_abort_transaction(void)
{
  imu_i2c_int_dis();                                    // disable interrupts
  if(imu_i2c_is_multibyte)                              // if this is a multibyte transmission, we must first force a stop condition
  {
    ROM_I2CMasterControl(IMU_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
                                                        // above code signals stop for send or receive
  }
  imu_i2c_in_progress = false;                          // transaction aborted
  xfer_done_cb(imu_i2c_data_byte_count);                // call user back with number of bytes remaining, txn done
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