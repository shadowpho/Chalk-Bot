//*****************************************************************************
//
// hibernate.c - Driver for the Hibernation module
//
// Copyright (c) 2007-2011 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 8049 of the Stellaris Peripheral Driver Library.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup hibernate_api
//! @{
//
//*****************************************************************************

#include "inc/hw_hibernate.h"
#include "inc/hw_ints.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/hibernate.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

//*****************************************************************************
//
// The delay in microseconds for writing to the Hibernation module registers.
//
//*****************************************************************************
#define DELAY_USECS             95

//*****************************************************************************
//
// The number of processor cycles to execute one pass of the delay loop.
//
//*****************************************************************************
#define LOOP_CYCLES             3

//*****************************************************************************
//
// The calculated number of delay loops to achieve the write delay.
//
//*****************************************************************************
static unsigned long g_ulWriteDelay;

//*****************************************************************************
//
//! \internal
//!
//! Polls until the write complete (WRC) bit in the hibernate control register
//! is set.
//!
//! \param None.
//!
//! On non-Fury-class devices, the hibernate module provides an indication when
//! any write is completed.  This is used to pace writes to the module.  This
//! function merely polls this bit and returns as soon as it is set.  At this
//! point, it is safe to perform another write to the module.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateWriteComplete(void)
{
    //
    // Add a delay here to enforce the required delay between write accesses to
    // certain Hibernation module registers.
    //
    if(CLASS_IS_FURY)
    {
        //
        // Delay a fixed time on Fury-class devices
        //
        SysCtlDelay(g_ulWriteDelay);
    }
    else
    {
        //
        // Spin until the write complete bit is set, for later devices.
        //
        while(!(HWREG(HIB_CTL) & HIB_CTL_WRC))
        {
        }
    }
}

//*****************************************************************************
//
//! Enables the Hibernation module for operation.
//!
//! \param ulHibClk is the rate of the clock supplied to the Hibernation
//! module.
//!
//! Enables the Hibernation module for operation.  This function should be
//! called before any of the Hibernation module features are used.
//!
//! The peripheral clock is the same as the processor clock.  This is the value
//! returned by SysCtlClockGet(), or it can be explicitly hard-coded if it is
//! constant and known (to save the code/execution overhead of a call to
//! SysCtlClockGet()).
//!
//! This function replaces the original HibernateEnable() API and performs the
//! same actions.  A macro is provided in <tt>hibernate.h</tt> to map the
//! original API to this API.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateEnableExpClk(unsigned long ulHibClk)
{
    //
    // Turn on the clock enable bit.
    //
    HWREG(HIB_CTL) |= HIB_CTL_CLK32EN;

    //
    // For Fury-class devices, compute the number of delay loops that must be
    // used to achieve the desired delay for writes to the hibernation
    // registers.  This value will be used in calls to SysCtlDelay().
    //
    if(CLASS_IS_FURY)
    {
        g_ulWriteDelay = (((ulHibClk / 1000) * DELAY_USECS) /
                          (1000L * LOOP_CYCLES));
        g_ulWriteDelay++;
    }
    else
    {
        //
        // Non-fury parts must wait for write complete following register
        // load (above).
        //
        HibernateWriteComplete();
    }
}

//*****************************************************************************
//
//! Disables the Hibernation module for operation.
//!
//! Disables the Hibernation module for operation.  After this function is
//! called, none of the Hibernation module features are available.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateDisable(void)
{
    //
    // Turn off the clock enable bit.
    //
    HWREG(HIB_CTL) &= ~HIB_CTL_CLK32EN;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Selects the clock input for the Hibernation module.
//!
//! \param ulClockInput specifies the clock input.
//!
//! Configures the clock input for the Hibernation module.  The configuration
//! option chosen depends entirely on hardware design.  The clock input for the
//! module will either be a 32.768 kHz oscillator or a 4.194304 MHz crystal.
//! The \e ulClockFlags parameter must be one of the following:
//!
//! - \b HIBERNATE_CLOCK_SEL_RAW - use the raw signal from a 32.768 kHz
//!   oscillator.
//! - \b HIBERNATE_CLOCK_SEL_DIV128 - use the crystal input, divided by 128.
//!
//! \note The \b HIBERNATE_CLOCK_SEL_DIV128 setting is not available on all
//! Stellaris devices.  Please consult the data sheet to determine if the
//! device that you are using supports the 4.194304 crystal as a source for the
//! Hibernation module.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateClockSelect(unsigned long ulClockInput)
{
    //
    // Check the arguments.
    //
    ASSERT((ulClockInput == HIBERNATE_CLOCK_SEL_RAW) ||
           (ulClockInput == HIBERNATE_CLOCK_SEL_DIV128));

    //
    // Set the clock selection bit according to the parameter.
    //
    HWREG(HIB_CTL) = ulClockInput | (HWREG(HIB_CTL) & ~HIB_CTL_CLKSEL);

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Configures the clock input for the Hibernation module.
//!
//! \param ulConfig is one of the possible configuration options for the clock
//! input listed below.
//!
//! This function is used to configure the clock input for the Hibernation
//! module.  The \e ulConfig parameter can be one of the following values:
//!
//! - \b HIBERNATE_OSC_DISABLE specifies that the internal oscillator
//! is powered off and either an externally supplied clock source or no clock
//! source is being used.
//! - \b HIBERNATE_OSC_HIGHDRIVE specifies a higher drive strength when a 24pF
//! filter capacitor is used with a crystal.
//! - \b HIBERNATE_OSC_LOWDRIVE specifies a lower drive strength when a 12pF
//! filter capacitor is used with a crystal.
//!
//! The \b HIBERNATE_OSC_DISABLE option is used to disable and power down the
//! internal oscillator if an external clock source or no clock source is used
//! instead of a 32.768 kHz crystal.  In the case where an external crystal is
//! used, either the \b HIBERNATE_OSC_HIGHDRIVE or \b HIBERNATE_OSC_LOWDRIVE is
//! used.  This optimizes the oscillator drive strength to match the size of
//! the filter capacitor that is used with the external crystal circuit.
//!
//! \note The ability to configure the clock input in the Hibernation
//! module is not available on all Stellaris devices.  Please consult the data
//! sheet for the Stellaris device that you are using to determine if this
//! feature is available.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateClockConfig(unsigned long ulConfig)
{
    unsigned long ulHIBCtl;

    ASSERT((ulConfig & (HIBERNATE_OSC_HIGHDRIVE | HIBERNATE_OSC_LOWDRIVE |
                        HIBERNATE_OSC_DISABLE)) == 0);

    ulHIBCtl = HWREG(HIB_CTL);

    //
    // Clear the current configuration bits.
    //
    ulHIBCtl &= ~(HIBERNATE_OSC_HIGHDRIVE | HIBERNATE_OSC_LOWDRIVE |
                  HIBERNATE_OSC_DISABLE);

    //
    // Set the new configuration bits.
    //
    ulHIBCtl |= ulConfig & (HIBERNATE_OSC_HIGHDRIVE | HIBERNATE_OSC_LOWDRIVE |
                            HIBERNATE_OSC_DISABLE);

    //
    // Set the hibernation clocking configuration.
    //
    HWREG(HIB_CTL) = ulHIBCtl;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Enables the RTC feature of the Hibernation module.
//!
//! Enables the RTC in the Hibernation module.  The RTC can be used to wake the
//! processor from hibernation at a certain time, or to generate interrupts at
//! certain times.  This function must be called before using any of the RTC
//! features of the Hibernation module.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateRTCEnable(void)
{
    //
    // Turn on the RTC enable bit.
    //
    HWREG(HIB_CTL) |= HIB_CTL_RTCEN;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Disables the RTC feature of the Hibernation module.
//!
//! Disables the RTC in the Hibernation module.  After calling this function
//! the RTC features of the Hibernation module will not be available.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateRTCDisable(void)
{
    //
    // Turn off the RTC enable bit.
    //
    HWREG(HIB_CTL) &= ~HIB_CTL_RTCEN;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Forces the Hibernation module to initiate a check of the battery voltage.
//!
//! This function forces the Hibernation module to initiate a check of the
//! battery voltage immediately rather than waiting for the next check interval
//! to pass.  After calling this function, the application should call the
//! () function and wait for the function to return a zero
//! value before calling the HibernateIntStatus() to check if the return code
//! has the \b HIBERNATE_INT_LOW_BAT set.  If \b HIBERNATE_INT_LOW_BAT is set
//! this indicates that battery level is low.  The application can also enable
//! the \b HIBERNATE_INT_LOW_BAT interrupt and wait for an interrupt to
//! indicate that the battery level is low.
//!
//! \note A hibernation request is held off if a battery check is in progress.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateBatCheckStart(void)
{
    //
    // Initiated a forced battery check.
    //
    HWREG(HIB_CTL) |= HIB_CTL_BATCHK;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Returns if a forced battery check has completed.
//!
//! This function returns if the forced battery check initiated by a call to
//! the HibernateBatCheckStart() function has completed.  This function will
//! return a non-zero value until the battery level check has completed.  Once
//! this function returns a value of zero, the hibernation module has completed
//! the battery check and the HibernateIntStatus() function can be used to
//! check if the battery was low by checking if the value returned has the
//! \b HIBERNATE_INT_LOW_BAT set.
//!
//! \return The value is zero when the battery level check has completed or
//! non-zero if the check is still in process.
//
//*****************************************************************************
unsigned long
HibernateBatCheckDone(void)
{
    //
    // Read the current state of the batter check.
    //
    return(HWREG(HIB_CTL) & HIB_CTL_BATCHK);
}

//*****************************************************************************
//
//! Configures the wake conditions for the Hibernation module.
//!
//! \param ulWakeFlags specifies which conditions should be used for waking.
//!
//! Enables the conditions under which the Hibernation module will wake.  The
//! \e ulWakeFlags parameter is the logical OR of any combination of the
//! following:
//!
//! - \b HIBERNATE_WAKE_PIN - wake when the external wake pin is asserted.
//! - \b HIBERNATE_WAKE_RTC - wake when one of the RTC matches occurs.
//! - \b HIBERNATE_WAKE_LOW_BAT - wake from hibernate due to a low battery
//! level being detected.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateWakeSet(unsigned long ulWakeFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(!(ulWakeFlags & ~(HIBERNATE_WAKE_PIN | HIBERNATE_WAKE_RTC |
                            HIBERNATE_WAKE_LOW_BAT)));

    //
    // Set the specified wake flags in the control register.
    //
    HWREG(HIB_CTL) = (ulWakeFlags |
                      (HWREG(HIB_CTL) & ~(HIBERNATE_WAKE_PIN
                                          | HIBERNATE_WAKE_RTC |
                                          HIBERNATE_WAKE_LOW_BAT)));

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Gets the currently configured wake conditions for the Hibernation module.
//!
//! Returns the flags representing the wake configuration for the Hibernation
//! module.  The return value is a combination of the following flags:
//!
//! - \b HIBERNATE_WAKE_PIN - wake when the external wake pin is asserted.
//! - \b HIBERNATE_WAKE_RTC - wake when one of the RTC matches occurs.
//! - \b HIBERNATE_WAKE_LOW_BAT - wake from hibernate due to a low battery
//! level being detected.
//!
//! \return Returns flags indicating the configured wake conditions.
//
//*****************************************************************************
unsigned long
HibernateWakeGet(void)
{
    //
    // Read the wake bits from the control register and return
    // those bits to the caller.
    //
    return(HWREG(HIB_CTL) & (HIBERNATE_WAKE_PIN | HIBERNATE_WAKE_RTC
                             | HIBERNATE_WAKE_LOW_BAT));
}

//*****************************************************************************
//
//! Configures the low battery detection.
//!
//! \param ulLowBatFlags specifies behavior of low battery detection.
//!
//! Enables the low battery detection and whether hibernation is allowed if a
//! low battery is detected.  If low battery detection is enabled, then a low
//! battery condition is indicated in the raw interrupt status register, and
//! can also trigger an interrupt.  Optionally, hibernation can be aborted if a
//! low battery is detected.
//!
//! The \e ulLowBatFlags parameter is one of the following values:
//!
//! - \b HIBERNATE_LOW_BAT_DETECT - detect a low battery condition.
//! - \b HIBERNATE_LOW_BAT_ABORT - detect a low battery condition, and abort
//!   hibernation if low battery is detected.
//!
//! The other setting in the \e ulLowBatFlags allows the caller to set one of
//! the following voltage level trigger values :
//!
//! - \b HIBERNATE_LOW_BAT_1_9V - voltage low level is 1.9V
//! - \b HIBERNATE_LOW_BAT_2_1V - voltage low level is 2.1V
//! - \b HIBERNATE_LOW_BAT_2_3V - voltage low level is 2.3V
//! - \b HIBERNATE_LOW_BAT_2_5V - voltage low level is 2.5V
//!
//! \b Example: Abort hibernate if the voltage level is below 2.1V.
//!
//! \verbatim
//! HibernateLowBatSet(HIBERNATE_LOW_BAT_ABORT | HIBERNATE_LOW_BAT_2_1V);
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************
void
HibernateLowBatSet(unsigned long ulLowBatFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(!(ulLowBatFlags & ~(HIB_CTL_VBATSEL_M | HIBERNATE_LOW_BAT_ABORT)));

    //
    // Set the low battery detect and abort bits in the control register,
    // according to the parameter.
    //
    HWREG(HIB_CTL) = (ulLowBatFlags |
                      (HWREG(HIB_CTL) & ~(HIB_CTL_VBATSEL_M
                                          | HIBERNATE_LOW_BAT_ABORT)));

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Gets the currently configured low battery detection behavior.
//!
//! Returns a value representing the currently configured low battery detection
//! behavior.
//!
//! The return value is a combination of the values described in the
//! HibernateLowBatSet() function.
//!
//! \return Returns a value indicating the configured low battery detection.
//
//*****************************************************************************
unsigned long
HibernateLowBatGet(void)
{
    //
    // Read the supported low bat bits from the control register and return
    // those bits to the caller.
    //
    return(HWREG(HIB_CTL) & (HIB_CTL_VBATSEL_M | HIBERNATE_LOW_BAT_ABORT));
}

//*****************************************************************************
//
//! Sets the value of the real time clock (RTC) counter.
//!
//! \param ulRTCValue is the new value for the RTC.
//!
//! Sets the value of the RTC.  The RTC will count seconds if the hardware is
//! configured correctly.  The RTC must be enabled by calling
//! HibernateRTCEnable() before calling this function.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateRTCSet(unsigned long ulRTCValue)
{
    //
    // Write the new RTC value to the RTC load register.
    //
    HWREG(HIB_RTCLD) = ulRTCValue;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Gets the value of the real time clock (RTC) counter.
//!
//! Gets the value of the RTC and returns it to the caller.
//!
//! \return Returns the value of the RTC.
//
//*****************************************************************************
unsigned long
HibernateRTCGet(void)
{
    //
    // Return the value of the RTC counter register to the caller.
    //
    return(HWREG(HIB_RTCC));
}

//*****************************************************************************
//
//! Sets the value of the RTC match 0 register.
//!
//! \param ulMatch is the value for the match register.
//!
//! Sets the match 0 register for the RTC.  The Hibernation module can be
//! configured to wake from hibernation, and/or generate an interrupt when the
//! value of the RTC counter is the same as the match register.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateRTCMatch0Set(unsigned long ulMatch)
{
    //
    // Write the new match value to the match register.
    //
    HWREG(HIB_RTCM0) = ulMatch;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Gets the value of the RTC match 0 register.
//!
//! Gets the value of the match 0 register for the RTC.
//!
//! \return Returns the value of the match register.
//
//*****************************************************************************
unsigned long
HibernateRTCMatch0Get(void)
{
    //
    // Return the value of the match register to the caller.
    //
    return(HWREG(HIB_RTCM0));
}

//*****************************************************************************
//
//! Sets the value of the RTC match 1 register.
//!
//! \param ulMatch is the value for the match register.
//!
//! Sets the match 1 register for the RTC.  The Hibernation module can be
//! configured to wake from hibernation, and/or generate an interrupt when the
//! value of the RTC counter is the same as the match register.
//!
//! \note The Hibernation RTC Match 1 feature is not available on all Stellaris
//! devices.  Please consult the data sheet for the Stellaris device that you
//! are using to determine if this feature is available.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateRTCMatch1Set(unsigned long ulMatch)
{
    //
    // Write the new match value to the match register.
    //
    HWREG(HIB_RTCM1) = ulMatch;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Gets the value of the RTC match 1 register.
//!
//! Gets the value of the match 1 register for the RTC.
//!
//! \note The Hibernation RTC Match 1 feature is not available on all Stellaris
//! devices.  Please consult the data sheet for the Stellaris device that you
//! are using to determine if this feature is available.
//!
//! \return Returns the value of the match register.
//
//*****************************************************************************
unsigned long
HibernateRTCMatch1Get(void)
{
    //
    // Return the value of the match register to the caller.
    //
    return(HWREG(HIB_RTCM1));
}

//*****************************************************************************
//
//! Sets the value of the RTC sub second match 0 register.
//!
//! \param ulMatch is the value for the sub second match register.
//!
//! Sets the sub second match 0 register for the RTC in 1/32768 of a second
//! increments.  The Hibernation module can be configured to wake from
//! hibernation, and/or generate an interrupt when the value of the RTC counter
//! is the same as the match combined with the sub second match register.
//!
//! \note The Hibernation sub second RTC Match 0 feature is not available on
//! all Stellaris devices.  Please consult the data sheet for the Stellaris
//! device that you are using to determine if this feature is available.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateRTCSSMatch0Set(unsigned long ulMatch)
{
    //
    // Write the new sub second match value to the sub second match register.
    //
    HWREG(HIB_RTCSS) = ulMatch << HIB_RTCSS_RTCSSM_S;

    //
    // Wait for write complete to be signaled on later devices.
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Returns the value of the RTC sub second match 0 register.
//!
//! This function returns the current value of the sub second match 0 register
//! for the RTC.  The value returned is in 1/32768 second increments.
//!
//! \note The Hibernation sub second RTC Match 0 feature is not available on
//! all Stellaris devices.  Please consult the data sheet for the Stellaris
//! device that you are using to determine if this feature is available.
//!
//! \return Returns the value of the sub section match register.
//
//*****************************************************************************
unsigned long
HibernateRTCSSMatch0Get(void)
{
    //
    // Read the current second RTC count.
    //
    return(HWREG(HIB_RTCSS) >> HIB_RTCSS_RTCSSM_S);
}

//*****************************************************************************
//
//! Returns the current value of the RTC sub second count.
//!
//! This function will return the current value of the sub second count for the
//! for the RTC in 1/32768 of a second increments.
//!
//! \note The Hibernation sub second RTC Match 0 feature is not available on
//! all Stellaris devices.  Please consult the data sheet for the Stellaris
//! device that you are using to determine if this feature is available.
//!
//! \return The current RTC sub second count in 1/32768 seconds.
//
//*****************************************************************************
unsigned long
HibernateRTCSSGet(void)
{
    //
    // Read the current second RTC count.
    //
    return(HWREG(HIB_RTCSS) & HIB_RTCSS_RTCSSC_M);
}

//*****************************************************************************
//
//! Sets the value of the RTC predivider trim register.
//!
//! \param ulTrim is the new value for the pre-divider trim register.
//!
//! Sets the value of the pre-divider trim register.  The input time source is
//! divided by the pre-divider to achieve a one-second clock rate.  Once every
//! 64 seconds, the value of the pre-divider trim register is applied to the
//! predivider to allow fine-tuning of the RTC rate, in order to make
//! corrections to the rate.  The software application can make adjustments to
//! the predivider trim register to account for variations in the accuracy of
//! the input time source.  The nominal value is 0x7FFF, and it can be adjusted
//! up or down in order to fine-tune the RTC rate.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateRTCTrimSet(unsigned long ulTrim)
{
    //
    // Check the arguments.
    //
    ASSERT(ulTrim < 0x10000);

    //
    // Write the new trim value to the trim register.
    //
    HWREG(HIB_RTCT) = ulTrim;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Gets the value of the RTC predivider trim register.
//!
//! Gets the value of the pre-divider trim register.  This function can be used
//! to get the current value of the trim register prior to making an adjustment
//! by using the HibernateRTCTrimSet() function.
//!
//! \return None.
//
//*****************************************************************************
unsigned long
HibernateRTCTrimGet(void)
{
    //
    // Return the value of the trim register to the caller.
    //
    return(HWREG(HIB_RTCT));
}

//*****************************************************************************
//
//! Stores data in the non-volatile memory of the Hibernation module.
//!
//! \param pulData points to the data that the caller wants to store in the
//! memory of the Hibernation module.
//! \param ulCount is the count of 32-bit words to store.
//!
//! Stores a set of data in the Hibernation module non-volatile memory.  This
//! memory is preserved when the power to the processor is turned off, and can
//! be used to store application state information which will be available when
//! the processor wakes.  Up to 64 32-bit words can be stored in the
//! non-volatile memory.  The data can be restored by calling the
//! HibernateDataGet() function.
//!
//! \note The amount of memory available in the Hibernation module varies
//! across Stellaris devices.  Please consult the data sheet for the Stellaris
//! device that you are using to determine the amount of memory available in
//! the Hibernation module.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateDataSet(unsigned long *pulData, unsigned long ulCount)
{
    unsigned long ulIdx;

    //
    // Check the arguments.
    //
    ASSERT(ulCount <= 64);
    ASSERT(pulData != 0);

    //
    // Loop through all the words to be stored, storing one at a time.
    //
    for(ulIdx = 0; ulIdx < ulCount; ulIdx++)
    {
        //
        // Write a word to the non-volatile storage area.
        //
        HWREG(HIB_DATA + (ulIdx * 4)) = pulData[ulIdx];

        //
        // Wait for write completion
        //
        HibernateWriteComplete();
    }
}

//*****************************************************************************
//
//! Reads a set of data from the non-volatile memory of the Hibernation module.
//!
//! \param pulData points to a location where the data that is read from the
//! Hibernation module will be stored.
//! \param ulCount is the count of 32-bit words to read.
//!
//! Retrieves a set of data from the Hibernation module non-volatile memory
//! that was previously stored with the HibernateDataSet() function.  The
//! caller must ensure that \e pulData points to a large enough memory block to
//! hold all the data that is read from the non-volatile memory.
//!
//! \note The amount of memory available in the Hibernation module varies
//! across Stellaris devices.  Please consult the data sheet for the Stellaris
//! device that you are using to determine the amount of memory available in
//! the Hibernation module.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateDataGet(unsigned long *pulData, unsigned long ulCount)
{
    unsigned long ulIdx;

    //
    // Check the arguments.
    //
    ASSERT(ulCount <= 64);
    ASSERT(pulData != 0);

    //
    // Loop through all the words to be restored, reading one at a time.
    //
    for(ulIdx = 0; ulIdx < ulCount; ulIdx++)
    {
        //
        // Read a word from the non-volatile storage area.  No delay is
        // required between reads.
        //
        pulData[ulIdx] = HWREG(HIB_DATA + (ulIdx * 4));
    }
}

//*****************************************************************************
//
//! Requests hibernation mode.
//!
//! This function requests the Hibernation module to disable the external
//! regulator, thus removing power from the processor and all peripherals.  The
//! Hibernation module will remain powered from the battery or auxiliary power
//! supply.
//!
//! The Hibernation module will re-enable the external regulator when one of
//! the configured wake conditions occurs (such as RTC match or external
//! \b WAKE pin).  When the power is restored the processor will go through a
//! normal power-on reset.  The processor can retrieve saved state information
//! with the HibernateDataGet() function.  Prior to calling the function to
//! request hibernation mode, the conditions for waking must have already been
//! set by using the HibernateWakeSet() function.
//!
//! Note that this function may return because some time may elapse before the
//! power is actually removed, or it may not be removed at all.  For this
//! reason, the processor will continue to execute instructions for some time
//! and the caller should be prepared for this function to return.  There are
//! various reasons why the power may not be removed.  For example, if the
//! HibernateLowBatSet() function was used to configure an abort if low
//! battery is detected, then the power will not be removed if the battery
//! voltage is too low.  There may be other reasons, related to the external
//! circuit design, that a request for hibernation may not actually occur.
//!
//! For all these reasons, the caller must be prepared for this function to
//! return.  The simplest way to handle it is to just enter an infinite loop
//! and wait for the power to be removed.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateRequest(void)
{
    //
    // Set the bit in the control register to cut main power to the processor.
    //
    HWREG(HIB_CTL) |= HIB_CTL_HIBREQ;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Enables interrupts for the Hibernation module.
//!
//! \param ulIntFlags is the bit mask of the interrupts to be enabled.
//!
//! Enables the specified interrupt sources from the Hibernation module.
//!
//! The \e ulIntFlags parameter must be the logical OR of any combination of
//! the following:
//!
//! - \b HIBERNATE_INT_WR_COMPLETE - write complete interrupt
//! - \b HIBERNATE_INT_PIN_WAKE - wake from pin interrupt
//! - \b HIBERNATE_INT_LOW_BAT - low battery interrupt
//! - \b HIBERNATE_INT_RTC_MATCH_0 - RTC match 0 interrupt
//! - \b HIBERNATE_INT_RTC_MATCH_1 - RTC match 1 interrupt
//!
//! \note The \b HIBERNATE_INT_RTC_MATCH_1 setting is not available on all
//! Stellaris devices.  Please consult the data sheet for the Stellaris device
//! that you are using to determine if the Hibernation RTC Match 1 feature is
//! available.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateIntEnable(unsigned long ulIntFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(!(ulIntFlags & ~(HIBERNATE_INT_PIN_WAKE | HIBERNATE_INT_LOW_BAT |
                            HIBERNATE_INT_RTC_MATCH_0 |
                            HIBERNATE_INT_RTC_MATCH_1 |
                            HIBERNATE_INT_WR_COMPLETE)));

    //
    // Set the specified interrupt mask bits.
    //
    HWREG(HIB_IM) |= ulIntFlags;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Disables interrupts for the Hibernation module.
//!
//! \param ulIntFlags is the bit mask of the interrupts to be disabled.
//!
//! Disables the specified interrupt sources from the Hibernation module.
//!
//! The \e ulIntFlags parameter has the same definition as the \e ulIntFlags
//! parameter to the HibernateIntEnable() function.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateIntDisable(unsigned long ulIntFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(!(ulIntFlags & ~(HIBERNATE_INT_PIN_WAKE | HIBERNATE_INT_LOW_BAT |
                            HIBERNATE_INT_RTC_MATCH_0 |
                            HIBERNATE_INT_RTC_MATCH_1 |
                            HIBERNATE_INT_WR_COMPLETE)));

    //
    // Clear the specified interrupt mask bits.
    //
    HWREG(HIB_IM) &= ~ulIntFlags;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Registers an interrupt handler for the Hibernation module interrupt.
//!
//! \param pfnHandler points to the function to be called when a hibernation
//! interrupt occurs.
//!
//! Registers the interrupt handler in the system interrupt controller.  The
//! interrupt is enabled at the global level, but individual interrupt sources
//! must still be enabled with a call to HibernateIntEnable().
//!
//! \sa IntRegister() for important information about registering interrupt
//! handlers.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateIntRegister(void (*pfnHandler)(void))
{
    //
    // Register the interrupt handler.
    //
    IntRegister(INT_HIBERNATE, pfnHandler);

    //
    // Enable the hibernate module interrupt.
    //
    IntEnable(INT_HIBERNATE);
}

//*****************************************************************************
//
//! Unregisters an interrupt handler for the Hibernation module interrupt.
//!
//! Unregisters the interrupt handler in the system interrupt controller.  The
//! interrupt is disabled at the global level, and the interrupt handler will
//! no longer be called.
//!
//! \sa IntRegister() for important information about registering interrupt
//! handlers.
//!
//! \return None.
//
//*****************************************************************************
void
HibernateIntUnregister(void)
{
    //
    // Disable the hibernate interrupt.
    //
    IntDisable(INT_HIBERNATE);

    //
    // Unregister the interrupt handler.
    //
    IntUnregister(INT_HIBERNATE);
}

//*****************************************************************************
//
//! Gets the current interrupt status of the Hibernation module.
//!
//! \param bMasked is false to retrieve the raw interrupt status, and true to
//! retrieve the masked interrupt status.
//!
//! Returns the interrupt status of the Hibernation module.  The caller can use
//! this to determine the cause of a hibernation interrupt.  Either the masked
//! or raw interrupt status can be returned.
//!
//! \return Returns the interrupt status as a bit field with the values as
//! described in the HibernateIntEnable() function.
//
//*****************************************************************************
unsigned long
HibernateIntStatus(tBoolean bMasked)
{
    //
    // Read and return the Hibernation module raw or masked interrupt status.
    //
    if(bMasked == true)
    {
        return(HWREG(HIB_MIS) & 0xf);
    }
    else
    {
        return(HWREG(HIB_RIS) & 0xf);
    }
}

//*****************************************************************************
//
//! Clears pending interrupts from the Hibernation module.
//!
//! \param ulIntFlags is the bit mask of the interrupts to be cleared.
//!
//! Clears the specified interrupt sources.  This must be done from within the
//! interrupt handler or else the handler is called again upon exit.
//!
//! The \e ulIntFlags parameter has the same definition as the \e ulIntFlags
//! parameter to the HibernateIntEnable() function.
//!
//! \note Because there is a write buffer in the Cortex-M3 processor, it may
//! take several clock cycles before the interrupt source is actually cleared.
//! Therefore, it is recommended that the interrupt source be cleared early in
//! the interrupt handler (as opposed to the very last action) to avoid
//! returning from the interrupt handler before the interrupt source is
//! actually cleared.  Failure to do so may result in the interrupt handler
//! being immediately reentered (because the interrupt controller still sees
//! the interrupt source asserted).
//!
//! \return None.
//
//*****************************************************************************
void
HibernateIntClear(unsigned long ulIntFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(!(ulIntFlags & ~(HIBERNATE_INT_PIN_WAKE | HIBERNATE_INT_LOW_BAT |
                            HIBERNATE_INT_RTC_MATCH_0 |
                            HIBERNATE_INT_RTC_MATCH_1)));

    //
    // Write the specified interrupt bits into the interrupt clear register.
    //
    HWREG(HIB_IC) |= ulIntFlags;

    //
    // Wait for write completion
    //
    HibernateWriteComplete();
}

//*****************************************************************************
//
//! Checks to see if the Hibernation module is already powered up.
//!
//! This function queries the control register to determine if the module is
//! already active.  This function can be called at a power-on reset to help
//! determine if the reset is due to a wake from hibernation or a cold start.
//! If the Hibernation module is already active, then it does not need to be
//! re-enabled and its status can be queried immediately.
//!
//! The software application should also use the HibernateIntStatus() function
//! to read the raw interrupt status to determine the cause of the wake.  The
//! HibernateDataGet() function can be used to restore state.  These
//! combinations of functions can be used by the software to determine if the
//! processor is waking from hibernation and the appropriate action to take as
//! a result.
//!
//! \return Returns \b true if the module is already active, and \b false if
//! not.
//
//*****************************************************************************
unsigned long
HibernateIsActive(void)
{
    //
    // Read the control register, and return true if the module is enabled.
    //
    return(HWREG(HIB_CTL) & HIB_CTL_CLK32EN ? 1 : 0);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
