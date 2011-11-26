//*****************************************************************************
//
// usbserial.c - Data logger module to handle serial device functions.
//
// Copyright (c) 2011 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 8049 of the EK-LM4F232 Firmware Package.
//
//*****************************************************************************

#include <string.h>
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"
#include "motion.h"                 // prior to 11/10/2011, was qs-logger.h
#include "usbserial.h"

//*****************************************************************************
//
// This module manages the USB serial device function. It is used when the
// eval board is connected to a host PC as a serial device, and can transmit
// data log records to the host PC through a virtual serial port.
//
//*****************************************************************************

// Local definitions
#define CBUS_DATA_WORD_SIZE     (4)                   // number of bytes in a word, if you change this, check code and update word size (i.e. in holding register space definition
#define CBUS_BUFFER_SIZE        (64)                  // must be a multiple of 4, buffer size for ChalkBus communication in bytes
#define CBUS_MAX_REG_COUNT      (14)                  // in this one-buffer max transfer  implementation, this is the max# registers in a given transfer
#define CBUS_FN_CODE_READ       (0x03)                // cbus function code for multi reg read  (from Modicon Modbus RTU)
#define CBUS_FN_CODE_WRITE      (0x10)                // cbus function code for multi reg write (from Modicon Modbus RTU)
#define CBUS_FN_CODE_EXCEPTION  (0x80)                // THIS WILL BE OR-ED WITH OFFENDING FUNCTION CODE and returned as the exception function (i.e. 0x01 in error results in 0x81)
                                                      //  msb can only be set in an exception return function.
                                                      // exception codes
#define CBUS_EX_CODE_FUNCTION   (0x01)                // unknown function code
#define CBUS_EX_CODE_ADDRESS    (0x02)                // out of range address
#define CBUS_EX_CODE_NAK        (0x07)                // negative acknowledge (returned here for bad register count, or expected packet size and received total bytes mismatch)
                                                      // byte wise header sizes for function codes with optional data sections
#define CBUS_FN_HDR_SIZE_RD_WR  (8)                   // header size for function read or write

// Local type definitions
                                                      // see comment block for "USBChalkBusPoll" below for more info
#pragma pack(1)                                       // pack structures on 1B (so we can read buffers with these)
typedef struct {                                      // function read or write packet format (slave responds with same format, zero data)
  unsigned char function_code;                        // should be one of the defined function codes (CBUS_FN_CODE_READ or CBUS_FN_CODE_WRITE) above
  unsigned char reserved_byte_1;                      // padding byte to make alignment 4B for data
  unsigned short starting_reg;                        // index (address) of first register to perform function on
  unsigned short reg_count;                           // count of registers (indicies) to operate on
  unsigned short data_bytes;                          // count of bytes in the optional data stage (zero for no data stage)
  unsigned long data[];                               // head pointer to optional data stage (using [] to calculate address as offset from structure head 
                                                      // when dereferenced instead of reading data here and using that as address)
} cbus_function_read_write_t;

typedef struct {                                      // outgoing slave data response packet (sent with same function code, 0 data as ACK, or with data if data asked for)
  unsigned char function_code;                        // offending function code (CBUS_FN_CODE_*), OR-ed with 1 in the MSb
  unsigned char exception_code;                       // exception code per Modicon Modbus RTU standard, above defined (CBUS_EX_CODE_*)
  unsigned short reserved_short_1;                    // pad to 4B
} cbus_slave_exception_t;
#pragma pack()                                        // release packing on 1B
//*****************************************************************************
//
// Global variables
//
//*****************************************************************************
static volatile tBoolean g_bUSBDevConnected = false;
static volatile tBoolean g_bUSBRxAvailable = false;   // set when Rx available, cleared when read
unsigned long ChalkBusRegs[CBUS_REG_COUNT];           // holding registers accessible with ChalkBus

// Local variables
unsigned long ChalkBusBuff[CBUS_BUFFER_SIZE/4];       // make buffer for handling comms to/from host

// Local functions
void USBChalkBusSendException(unsigned char offending_function, unsigned char code);    
                                                      // sends an exception to the host, code should be one of CBUS_EX_CODE_*

//*****************************************************************************
//
// The line coding parameters for the virtual serial port.  Since there is
// no physical port this does not have any real effect, but we have a default
// set of values to report if asked, and will remember whatever the host
// configures.
//
//*****************************************************************************
static tLineCoding g_sLineCoding =
{
    115200, USB_CDC_STOP_BITS_1, USB_CDC_PARITY_NONE, 8
};

//*****************************************************************************
//
// Set the communication parameters for the virtual serial port.
//
//*****************************************************************************
static tBoolean
SetLineCoding(tLineCoding *psLineCoding)
{
    //
    // Copy whatever the host passes into our copy of line parameters.
    //
    memcpy(&g_sLineCoding, psLineCoding, sizeof(tLineCoding));

    //
    // ALways return success
    //
    return(true);
}

//*****************************************************************************
//
// Get the communication parameters in use on the UART.
//
//*****************************************************************************
static void
GetLineCoding(tLineCoding *psLineCoding)
{
    //
    // Copy whatever we have stored as the line parameter to the host.
    //
    memcpy(psLineCoding, &g_sLineCoding, sizeof(tLineCoding));
}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
ControlHandler(void *pvCBData, unsigned long ulEvent,
               unsigned long ulMsgValue, void *pvMsgData)
{
    //
    // Which event are we being asked to process?
    //
    switch(ulEvent)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bUSBDevConnected = true;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBDevConnected = false;
            break;
        }

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
        {
            GetLineCoding(pvMsgData);
            break;
        }

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
        {
            SetLineCoding(pvMsgData);
            break;
        }

        //
        // The following line control events can be ignored because there is
        // no physical serial port to manage.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        case USBD_CDC_EVENT_SEND_BREAK:
        case USBD_CDC_EVENT_CLEAR_BREAK:
        {
            break;
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }

        //
        // An unknown event occurred.
        //
        default:
        {
            break;
        }
    }

    //
    // Return control to USB stack
    //
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ulCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ulEvent)
    {
        case USB_EVENT_TX_COMPLETE:
        {
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;
        }

        //
        // We don't expect to receive any other events.
        //
        default:
        {
            break;
        }
    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ulCBData is the client-supplied callback data value for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
RxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ulEvent)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            g_bUSBRxAvailable = true;                       // mark that Rx is available
            break;
        }

        //
        // We are being asked how much unprocessed data we have still to
        // process.  Since there is no actual serial port and we are not
        // processing any RX data, just return 0.
        //
        case USB_EVENT_DATA_REMAINING:
        {
            return(0);
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.
        //
        default:
        {
            break;
        }
    }
    return(0);
}

//*****************************************************************************
//
// Initializes the USB serial device.
//
//*****************************************************************************
void
USBSerialInit(void)
{
    //
    // Initialize the transmit and receive buffers.
    //
    USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
    USBBufferInit((tUSBBuffer *)&g_sRxBuffer);

    //
    // Initialize the USB library CDC device function.
    //
    USBDCDCInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);
}

//*****************************************************************************
//
// This is called by the application main loop to perform regular processing.
// This is just a stub here because everything is event or interrupt driven.
//
//*****************************************************************************
void
USBSerialRun(void)
{
}

//*****************************************************************************
//
// Write a data record to the serial port.  An acquired data record is passed
// in and is composed into a binary packet and sent on the serial port.  The
// host PC, if connected will receive this packet via the virtual serial port,
// and can decode and display the data.
//
// The binary packet has the following format:
// - 16-bit header, value 0x5351
// - 32-bit seconds time stamp
// - 16-bit fractional seconds time stamp (1/32768 resolution)
// - 16-bit data item selection mask (which items are included in the record)
// - multiple 16-bit data item values, per selection mask
// - 16-bit checksum which when added to the 16-bit sum of the entire packet
// will result in 0.
//
// The entire packet is transmitted bytewise over the virtual serial port,
// little-endian format.
//
//*****************************************************************************
int
USBSerialWriteRecord(tLogRecord *pRecord)
{
    unsigned long ulIdx;
    unsigned short usChecksum;
    unsigned long ulItemCount;
    unsigned short *pusBuf;

    //
    // Check the arguments
    //
    ASSERT(pRecord);
    if(!pRecord)
    {
        return(1);
    }

    //
    // Check state for ready device
    //
    if(!g_bUSBDevConnected)
    {
        return(1);
    }

    //
    // Determine how many channels are to be logged
    //
    ulIdx = pRecord->usItemMask;
    ulItemCount = 0;
    while(ulIdx)
    {
        if(ulIdx & 1)
        {
            ulItemCount++;
        }
        ulIdx >>= 1;
    }

    //
    // Add to item count the equivalent number of 16-bit words for timestamp
    // and selection mask.
    //
    ulItemCount += 4;

    //
    // Put the header word in the USB buffer
    //
    usChecksum = 0x5351;
    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,
                   (unsigned char *)&usChecksum, 2);

    //
    // Compute the checksum over the entire record
    //
    pusBuf = (unsigned short *)pRecord;
    for(ulIdx = 0; ulIdx < ulItemCount; ulIdx++)
    {
        usChecksum += pusBuf[ulIdx];
    }

    //
    // Convert item count to bytes.  This now represents the entire record
    // size in bytes, not including the checksum.  The header has already
    // been sent
    //
    ulItemCount *= 2;

    //
    // Transmit the record, which includes the time stamp, selection mask
    // and all selected data item, to the USB buffer
    //
    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,
                   (unsigned char *)pusBuf, ulItemCount);

    //
    // Adjust the checksum so that when added (as unsigned short) to the
    // sum of the rest of the packet, the result will be 0
    //
    usChecksum = (unsigned short)(0x10000L - (unsigned long)usChecksum);

    //
    // Transmit the checksum which is the end of the packet
    //
    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,
                   (unsigned char *)&usChecksum, 2);

    //
    // Return success to the caller
    //
    return(0);
}

// * USBChalkBusPoll **********************************************************
// * Handle communication requests from ChalkBus master.                      *
// *                                                                          *
// * Communication protocol based off of a subset of the Modicon Modbus RTU   *
// * protocol developed by Modicon, Inc.                                      *
// *                                                                          *
// * Note: Among other changes, multi-byte value endianness has been changed  *
// *       to little endian in the ChalkBus implementation for efficient      *
// *       compatibility with the Cortex-M4 core's native little-endian byte  *
// *       order.                                                             *
// *                                                                          *
// * See structures above (cbus_*_t) for definition of packet structures for  *
// * different functions.                                                     *
// *                                                                          *
// * Function codes:                                                          *
// * 0x03: read registers (pc reading from lm4f)                              *
// * 0x10: multiple register set (pc writing to lm4f)                         *
// *                                                                          *
// * Exception signaling:                                                     *
// * Offending function code or-ed with 0x80 (msb set), see structure above   *
// * (cbus_slave_exception_t) for return format.                              *
// *                                                                          *
// * Register read/write from ChalkBusRegs[].                                 *
// * Note: This driver is not written to handle multi-packet transmissions,   *
// *       overall package size (header, data) must fit within a 64B USB bulk *
// *       transmission.                                                      *
// ****************************************************************************
void USBChalkBusPoll(void)
{
  unsigned long buffer_bytes;                               // bytes received from/accepted by buffer
  cbus_function_read_write_t* cbus_rw_header;               // header for reading/writing a 
  if(!g_bUSBRxAvailable)                                    // if no Rx is available 
  {
    return;                                                 // exit now (we only send solicited responses)
  }
                                                            // else new Rx is in the buffer, let's handle it
  buffer_bytes = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, (unsigned char*)ChalkBusBuff, CBUS_BUFFER_SIZE);
  //buffer_bytes = USBDCDCPacketRead(&g_sCDCDevice,unsigned char *pcData,unsigned long ulLength,tBoolean bLast)
                                                            // read data in from the receive buffer
  g_bUSBRxAvailable = false;                                // we have read the new Rx, no more available
  if(buffer_bytes == 0)                                     // cancel, there wasn't actually any data
  {
    return;
  }
                                                            // find out what kind of packet this is, validate it
  cbus_rw_header = (cbus_function_read_write_t*)ChalkBusBuff;
                                                            // guess that this function is either read or write to save doing this cast in both cases
  switch(*((unsigned char*)ChalkBusBuff))                   // read function code
  {
    case CBUS_FN_CODE_READ:                                 // function read
    case CBUS_FN_CODE_WRITE:                                // function write, same validation steps
      {
        if((CBUS_FN_HDR_SIZE_RD_WR > buffer_bytes) ||
           (CBUS_FN_HDR_SIZE_RD_WR + cbus_rw_header->data_bytes != buffer_bytes))
        {
          USBChalkBusSendException(cbus_rw_header->function_code, CBUS_EX_CODE_NAK);
                                                            // throw exception, too few bytes even for header or bytes expected != bytes received
          return;
        }
        if(cbus_rw_header->reg_count > CBUS_MAX_REG_COUNT)  // more registers requested than are allowed in a single request
        {
          USBChalkBusSendException(cbus_rw_header->function_code, CBUS_EX_CODE_NAK);
                                                            // throw exception, more registers requested than are allowed
          return;
        }
        if((cbus_rw_header->function_code == CBUS_FN_CODE_WRITE) &&
           (cbus_rw_header->reg_count * CBUS_DATA_WORD_SIZE != cbus_rw_header->data_bytes))
        {
          USBChalkBusSendException(cbus_rw_header->function_code, CBUS_EX_CODE_NAK);
                                                            // throw exception, register count not consistent with data bytes count
          return;
        }
        if( (cbus_rw_header->starting_reg >= CBUS_REG_COUNT) ||
           ((cbus_rw_header->starting_reg + cbus_rw_header->reg_count) > CBUS_REG_COUNT))
        {                                                   // if starting register or ending register beyond last register
          USBChalkBusSendException(cbus_rw_header->function_code, CBUS_EX_CODE_ADDRESS);
                                                            // throw exception, an address is out of range
          return;
        }
      }
      break;
    default:                                                // function unknown
      {
        USBChalkBusSendException(*((unsigned char*)ChalkBusBuff), CBUS_EX_CODE_FUNCTION);
                                                            // throw exception, this function code is unknown
        return;                                             // done
      }
  }
                                                            // handle now validated function
  switch(*((unsigned char*)ChalkBusBuff))                   // read function code
  {
    case CBUS_FN_CODE_READ:                                 // function read
      {
        unsigned short count;                               // used to copy data
        unsigned long* read_ptr;                            // source pointer (holding registers)
        read_ptr = ChalkBusRegs + (cbus_rw_header->starting_reg);
                                                            // initialize starting read pointer
        for(count = 0; count < (cbus_rw_header->reg_count); count++)
        {                                                   // copy regs a register at a time
          cbus_rw_header->data[count] = *read_ptr;          // copy data at read_ptr into outgoing packet
          read_ptr++;                                       // increment read_ptr to next register
        }
        cbus_rw_header->data_bytes = cbus_rw_header->reg_count * CBUS_DATA_WORD_SIZE;
                                                            // load number of bytes written into header before returning
        buffer_bytes = CBUS_FN_HDR_SIZE_RD_WR + cbus_rw_header->data_bytes;
                                                            // load count of bytes to be queued for tx to host below
      }
      break;
    case CBUS_FN_CODE_WRITE:                                // function write
      {
        unsigned short count;                               // used to copy data
        unsigned long* write_ptr;                           // destination pointer (holding registers)
        write_ptr = ChalkBusRegs + (cbus_rw_header->starting_reg);
                                                            // initialize starting write pointer
        for(count = 0; count < (cbus_rw_header->reg_count); count++)
        {                                                   // copy regs a register at a time
          *write_ptr = cbus_rw_header->data[count];         // copy data from incoming packet to write_ptr
          write_ptr++;                                      // increment write_ptr to next register
        }
        cbus_rw_header->data_bytes = 0;                     // returning zero data in response
        buffer_bytes = CBUS_FN_HDR_SIZE_RD_WR;              // acknowledge packet's only bytes to return is header with 0 data byte count
      }
      break;
    default:                                                // function unknown
      {
        return;                                             // unknown functions should never get here
      }
  } 
  if(g_bUSBDevConnected)                                    // send response if there is a device to listen
  {
    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,(unsigned char*)ChalkBusBuff, buffer_bytes);
                                                            // queue the data to send
  }
}
            
// * USBChalkBusSendException *************************************************
// * Sends an exception to the master.                                        *
// *                                                                          *
// * offending_function should be the function code that caused the exception *
// * code should be the exception code that should be sent to the master,     *
// *      it's value should be one of CBUS_EX_CODE_*.                         *
// ****************************************************************************
void USBChalkBusSendException(unsigned char offending_function, unsigned char code)
{
  cbus_slave_exception_t* cbus_exception;                   // will use this to build an exception
  cbus_exception = (cbus_slave_exception_t*)ChalkBusBuff;   // use ChalkBusBuff to build the exception
  cbus_exception->function_code = offending_function | CBUS_FN_CODE_EXCEPTION;
                                                            // exception signaled by setting msb of offending function code
  cbus_exception->exception_code = code;                    // load the exception code
  if(g_bUSBDevConnected)                                    // send exception if there is a device to listen
  {
    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,(unsigned char*)ChalkBusBuff, sizeof(cbus_slave_exception_t));
                                                            // queue the exception to send
  }
}
