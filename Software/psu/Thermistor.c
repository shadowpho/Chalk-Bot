/*******************************************************************************
* File Name: Thermistor.c
* Version 1.1
*
* Description:
* This c file contains the functions required to compute Temperature using
* thermistor
*
* Note:
*
********************************************************************************
* Copyright (2011), Cypress Semiconductor Corporation. All rights reserved.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/


/**************************************************************************************
*                                THEROY OF OPERATION
* This program reads the value of 2 different voltages to determine the ratio between
* Rthermistor and Rref. The resistance value of the thermistor is used to calculate the
* temperature given the following Steinhart-Hart equation
*
* 	1/T = A + B*ln(R) + C*ln(R)^3
*
* The Steinhart-Hart equation  is pre calculated using the lookup table 
* "Thermistor_TempTable". Please reference Application Note AN2017 for more information.
*
****************************************************************************************/


#include <device.h>
#include "Thermistor.h"
//#include <string.h>
//#include <STDIO.h>

/* Selects display mode for temperature */
uint8 TemperatureModeFlag = CELSIUS_MODE;

/* Look up table temperature from -40 to +125 degree C */
const uint32 Thermistor_TempTable[] = 
{ 
  328996,307906,288311,270096,253153,
  237386,222670,208964,196194,184288,
  173184,162822,153148,144112,135667,
  127773,120404,113506,107048,100997,
  95326,89988,84983,80288,75882,
  71745,67874,64235,60812,57593,
  54563,51698,49000,46460,44068,
  41813,39690,37687,35798,34014,
  32330,30737,29232,27810,26465,
  25193,23990,22851,21773,20752,
  19785,18868,17998,17174,16392,
  15650,14946,14278,13644,13041,
  12468,11923,11406,10913,10445,
  10000,9575,9172,8787,8421,
  8072,7739,7422,7119,6831,
  6555,6293,6042,5803,5574,
  5356,5147,4948,4757,4575,
  4400,4233,4074,3921,3775,
  3635,3501,3372,3249,3131,
  3018,2910,2807,2707,2612,
  2520,2432,2348,2267,2189,
  2114,2042,1973,1906,1842,
  1781,1722,1665,1611,1559,
  1509,1460,1413,1368,1325,
  1283,1243,1204,1167,1131,
  1096,1063,1030,999,969,
  940,912,885,859,834,
  810,786,764,742,720,
  700,680,661,643,625,
  607,591,575,559,544,
  529,515,501,488,475,
  463,451,439,427,416,
  406,395,385,376,366,
  357
};

/*******************************************************************************
* Function Name:  ThermistorADC_TemperatureCompute
********************************************************************************
* Summary:
* Reads the Vref and Vthermistor value and calculates the
* resistance of the thermistor. Once the thermistor resistance
* value is calculated temperature is measured by looking for thermistor
* resistance offset index in the look up table which stores temperature 
* from -40 to 125 C. Piece wise linear approximation is used to
* measure the temperature up to two decimal place. 
**
* Parameters:  
* void:  
*
* Return: 
* void
*
* Theory: 
* Temperature is calculated by measuring the Resistance of the thermistor
* using the following equation from the loop up table
* Rthermistor = Rref*(VThermistor-Vlow)/Vref-VThermistor)
*
* The resistance value of the thermistor is also used to calculate the temperature
* using following Steinhart-Hart equation the result of which in contained in 
* the lookup table.
* 1/T = A + B*ln(R) + C*ln(R)^3
* 
* Side Effects:
*  	
********************************************************************************/
int32 Thermistor_TemperatureCompute(void)
{   
    uint32 ThermistorResistance;		/* Thermistor resistance variable */  
	int32 Thermistor_Temperature;	    /* thermistor temperature * 10 */
    uint8 Count=0;
    int16 ADC_VRef, ADC_VThermistor, ADC_VRefLow = 0;	/* Voltages read by the ADC */
    uint32 TempTable_UpperLimit, TempTable_LowerLimit;	/* Linear Interpolation variables */
    int16 TempTable_Decimal;							/* Fractional portion of temperature */


    /* Set Amux channel0 and read voltage across reference resistor */
    AMux_1_Select(0);
    AcclADC_Stop();
    AcclADC_Start();
    AcclADC_StartConvert();
    AcclADC_IsEndConversion(AcclADC_WAIT_FOR_RESULT);
    ADC_VRef = AcclADC_GetResult16();

    /* Set Amux channel1 and read voltage across reference resistor */
    AMux_1_Select(1);
    AcclADC_Stop();
    AcclADC_Start();
    AcclADC_StartConvert();
    AcclADC_IsEndConversion(AcclADC_WAIT_FOR_RESULT);
    ADC_VThermistor = AcclADC_GetResult16();

    /* Resistance calculated with integer point math */
    ThermistorResistance  = (((int32)(ADC_VThermistor - ADC_VRefLow) * (int32)THERM_RREF) / ((int32)(ADC_VRef - ADC_VThermistor)));

    /* Find the whole number resistance value in the look up table and calculate the temperature */
    for(Count = 0; Thermistor_TempTable[Count] >= ThermistorResistance; Count++);

    /* Piece-wise linear approximation to find fractional value between 1C increments to 
	*  calculate temperature to 1 decimal place */
    TempTable_Decimal = 0;                
    TempTable_LowerLimit = Thermistor_TempTable[Count];
    TempTable_UpperLimit = Thermistor_TempTable[Count-1];

	TempTable_Decimal = (int16)(((TempTable_UpperLimit - ThermistorResistance) * 10) / (TempTable_UpperLimit - TempTable_LowerLimit));
	Thermistor_Temperature = ((Count - 40 - 1) * 10) + (TempTable_Decimal);
	
  	return Thermistor_Temperature;
}

