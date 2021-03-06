//*****************************************************************************
//
// usbserial.h - Prototypes and definitions for USB serial module.
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
#ifndef __USBSERIAL_H__
#define __USBSERIAL_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif
  
//*****************************************************************************
//
// ChalkBus holding register size and index symbol definitions
//
//*****************************************************************************  
#define CBUS_REG_COUNT                  (255)               // count of holding registers
  
                                                            // indicies of data of interest
#define CBUS_IDX_RESERVED               (0)                 // index 0 is reserved
#define CBUS_IDX_RAW_ENC_L              (1)                 // read only position of left encoder (signed) [counts]
#define CBUS_IDX_RAW_VEL_L              (2)                 // read only velocity of left encoder (signed) [60counts/sec]
#define CBUS_IDX_SCALED_ENC_L           (3)                 // read only position of left encoder (signed) [mm]
#define CBUS_IDX_SCALED_VEL_L           (4)                 // read only velocity of left encoder (signed) [mm/sec]
#define CBUS_IDX_RAW_ENC_R              (5)                 // read only position of right encoder (signed) [counts]
#define CBUS_IDX_RAW_VEL_R              (6)                 // read only velocity of right encoder (signed) [60counts/sec]
#define CBUS_IDX_SCALED_ENC_R           (7)                 // read only position of right encoder (signed) [mm]
#define CBUS_IDX_SCALED_VEL_R           (8)                 // read only velocity of right encoder (signed) [mm/sec]
#define CBUS_IDX_YAW_RATE_MILLIDPS      (9)                 // ro yaw rate of robot (signed) [millidps]
#define CBUS_IDX_HEADING_MILLIDEGREES   (10)                // ro integrated heading of robot (signed) [millidegrees]
#define CBUS_IDX_EFFORT_L               (11)                // rw hbridge effort left (signed) [-8192, 8192], 50% power limit
#define CBUS_IDX_EFFORT_R               (12)                // rw hbridge effort left (signed) [-8192, 8192], 50% power limit
  
#define CBUS_IDX_CHLK_MODE              (13)                // rw enum 0=servos off, 1=manual mode, 2=automated mode
#define CBUS_IDX_CHLK_MAN_SVM_Z         (14)                // rw 700-2300 [us] proportional to position
#define CBUS_IDX_CHLK_MAN_SVM_CLAMP     (15)                // rw 700-2300 [us] proportional to position
#define CBUS_IDX_CHLK_AUTO_COMMAND      (16)                // rw enum 0=nop, 1=load(reload if chalk low), 2=store
#define CBUS_IDX_CHLK_STATUS            (17)                // ro enum manual, idle, reloading, storing, chalk_low
#define CBUS_IDX_CHLK_SW_STATUS         (18)                // ro enum 1=depressed, 0=released

  
//*****************************************************************************
//
// Module externally accessible variables
//
//*****************************************************************************  
extern unsigned long ChalkBusRegs[];                        // holding registers accessible with ChalkBus

//*****************************************************************************
//
// Module function prototypes.
//
//*****************************************************************************
extern void USBSerialInit(void);
extern void USBSerialRun(void);
extern int USBSerialWriteRecord(tLogRecord *pRecord);
extern void USBChalkBusPoll(void);                        // polling routine for handling ChalkBus requests (see comments)

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __USBSERIAL_H__
