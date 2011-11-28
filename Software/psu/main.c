#include <device.h>
#include <string.h>

#include "Thermistor.h"
#include "i2c_devices.h"


extern uint8 TemperatureModeFlag;


uint8 SLAVE_ADDRESS = 54;
volatile uint32 barrel=0;
uint8 master_status;
uint8 read_buffer[512];
uint32 reg_stat;

CY_ISR(ALERT1_ISR)
{

	ALERT1_ClearPending();
	PSU_Enable_Write(PSU_Enable_Read() & 2); //clear enable bit 1
	 
}

CY_ISR(ALERT2_ISR)
{
	ALERT2_ClearPending();
	PSU_Enable_Write(PSU_Enable_Read() & 1); //clear enable bit 2

}
volatile uint8 reset=0;
CY_ISR(Button_Press_ISR)
{
	Button_ClearInterrupt();
	PSU_Enable_Write(0); //TURN BOTH OFF NOW!
	reset=1;
	I2C_1_Stop();
}

uint8 find_i2c_device(uint8 address)
{
	uint8 stat;
	uint8 dat;
	for(;address<100;address++)
	{
		stat=I2C_1_MasterClearStatus();
		dat=0; //read reg at byte 1.
		I2C_1_MasterWriteBuf(address,&dat,1,I2C_1_MODE_COMPLETE_XFER); //send write reg request at address
		
not_done:
		stat = I2C_1_MasterStatus();
		if(stat & I2C_1_MSTAT_XFER_INP)
			goto not_done;
		if(stat & I2C_1_MSTAT_ERR_XFER)
			continue;
		if(stat & I2C_1_MSTAT_ERR_ADDR_NAK)
			continue;
			
		stat=I2C_1_MasterReadBuf(address,&dat,1,I2C_1_MODE_COMPLETE_XFER); //send write reg request at address
		return address;
	}
	return 0; //not found
	
}
void main()
{
    uint8 ButtonPressFlag = 0;
	int32 temp=0;
	uint32 lowest=24,i;
	uint32 increment=2;
	uint32 battery_volts;
	volatile uint32 counter=0;
    
	CYGlobalIntDisable;
	/* Intitalize hardware */
  	LEDControlReg_Write(0xff);         /* Turn off the LEDs on PORT2(pin 0-3) and PORT4 pin(0-3) */
    PSU_Enable_Write(3);
//	AMux_1_Start();                 /* Enable THe analog mux input to the ADC */
//    AcclADC_Start();                /* Start ADC */
//    VDAC8_1_Start();                /* Start and configure the VDAC used to measure the Thermistor */
//    VDAC8_1_SetRange(VDAC8_1_RANGE_1V);
//    VDAC8_1_SetValue(200 );

	PWM_0_Start();
	PWM_1_Start();
	PWM_2_Start();
	PWM_3_Start();
	
	PWM_4_Start();
	PWM_5_Start();
	PWM_6_Start();
	PWM_7_Start();
	
//	VBATT_ADC_Start();
	//VBATT_ADC_StartConvert();
//	VBATT_ADC_Stop(); //debugging

//	UART_1_Start();
	I2C_1_Start();
	I2C_1_EnableInt();
	
	Button_ClearInterrupt();
	ALERT2_ClearPending();
	ALERT1_ClearPending();
	
	ALERT1_StartEx(ALERT1_ISR);	

	Button_Pressed_StartEx(Button_Press_ISR);
	ALERT2_StartEx(ALERT2_ISR);

	
	CYGlobalIntEnable;              /* Enable global interrupt */
    
	I2C_1_MasterClearStatus();
		while(set_ina226(CH1)!=CYRET_SUCCESS);
		while(set_tmp100(CH1)!=CYRET_SUCCESS);


	while(1)
	{
		if(0)
		//if(Status_Reg_1_Read()&1)
		{
			read_tmp100(CH1);
			read_ina226(CH1);
		}
		LEDControlReg_Write(((uint8)~(PSU_Enable_Read())));
		for(i=0;i<65000;i++);
		LEDControlReg_Write(((uint8)~(PSU_Enable_Read())) & ~(1<<7));
		for(i=0;i<65000;i++);
//		CyPmSaveClocks();
//		CyPmSleep(PM_SLEEP_TIME_NONE, PM_SLEEP_SRC_PICU);
//        CyPmRestoreClocks();
        
//		battery_volts = VBATT_ADC_GetResult32();
	
		if(reset==1)
		{
	
		for(i=0;i<65000;i++); //delay for half a second.
		PSU_Enable_Write(3); //Both PSU ON
		I2C_1_Start();
		I2C_1_EnableInt();
		while(set_ina226(CH1)!=CYRET_SUCCESS);
		while(set_tmp100(CH1)!=CYRET_SUCCESS);
		reset=0;
		}
		
	}
	
	
	


    while(1)
    {
        /* Calculate the current board temperature */
        temp = Thermistor_TemperatureCompute() / 10; //we get 24.1 as 241. We drop fractionals.
		uint32 barrels_above= (temp-lowest)/increment;
		if(temp<=lowest) barrels_above=0; //negative temperatures are too low!
		if(barrels_above>8) barrels_above=8;
		LEDControlReg_Write(1<<barrels_above);
		
    }
}
/* [] END OF FILE */

