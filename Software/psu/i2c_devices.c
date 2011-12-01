#include "i2c_devices.h"


volatile float tmp100_status[2];
volatile struct ina226_status_s ina226_status[2];

uint8 i2c_buffer[4]; //we want to be able to send 32 bytes per transfer. This will be used by ISR, so volatile.


//1 = success, 0 = keep polling, 2 = error
int8 translate_status(uint8 stat)
{
	if(stat == 0) return 0;
	
	if(stat & I2C_1_MSTAT_ERR_MASK )
	return 2;
	
	if((stat&I2C_1_MSTAT_RD_CMPLT) || (stat & I2C_1_MSTAT_WR_CMPLT))
	return 1;
	
	
	if(stat & I2C_1_MSTAT_XFER_INP)
	 return 0;
	 

	
	return 2;
}

//Side-effects : i2c_buffer MODIFIED. you probably want to read it.
//returns CYRET_UNKNOWN on error or CYRET_SUCCESS on success.
//blocking.
int read_16reg(uint8 address, uint8 reg)
{
	i2c_buffer[0] = reg; 
	static uint8 stat=0;
	stat = I2C_1_MasterClearStatus();
	I2C_1_MasterWriteBuf(address,i2c_buffer,1,I2C_1_MODE_NO_STOP);	
	stat=0;
	while(stat==0){
			stat = I2C_1_MasterStatus();
			stat = translate_status(stat);
			if(stat==2) return CYRET_UNKNOWN;
		   }
	I2C_1_MasterReadBuf(address,i2c_buffer,2,I2C_1_MODE_REPEAT_START);	
	
	do{
			stat = I2C_1_MasterStatus();
			if(stat&I2C_1_MSTAT_ERR_MASK) return CYRET_UNKNOWN;
	}while(stat&I2C_1_MSTAT_XFER_INP); 
	
	return CYRET_SUCCESS;
}
int check_chan(int channel)
{
	if(channel > CH2) return CYRET_BAD_PARAM;
	else return CYRET_SUCCESS;
}


int set_tmp100(int channel)
{
	uint8 stat;
	uint8 address = 0;
	if( check_chan(channel)!=CYRET_SUCCESS) return CYRET_BAD_PARAM;
	i2c_buffer[0] = 1; //register 1
	i2c_buffer[1] = TMP100_CNF;
	if(channel==CH1) address = CHAN1_TMP100;
	else if(channel==CH2) address = CHAN2_TMP100;
	else return CYRET_BAD_PARAM;
	//return i2c_send_man(address, i2c_buffer,1);
	stat = I2C_1_MasterClearStatus();
	I2C_1_MasterWriteBuf(address,i2c_buffer,2,I2C_1_MODE_COMPLETE_XFER);
	stat=0;
	while(stat==0){
			stat = I2C_1_MasterStatus();
			stat = translate_status(stat);
			if(stat==2) return CYRET_UNKNOWN;
		   }
	
	return CYRET_SUCCESS;
}

int set_ina226(int channel)
{
	uint8 stat;
	uint8 address = 0;
	if( check_chan(channel)!=CYRET_SUCCESS) return CYRET_BAD_PARAM;
	i2c_buffer[0] = 5; //register 5, shunt value 
	i2c_buffer[1] = INA226_SHUNT>>8;
	i2c_buffer[2] = INA226_SHUNT & 0xff;
	
	if(channel==CH1) address = CHAN1_INA226;
	else if(channel==CH2) address = CHAN2_INA226;
	else return CYRET_BAD_PARAM;
	
	stat = I2C_1_MasterClearStatus();
	I2C_1_MasterWriteBuf(address,i2c_buffer,3,I2C_1_MODE_COMPLETE_XFER);
	stat=0;
	while(stat==0){
			stat = I2C_1_MasterStatus();
			stat = translate_status(stat);
			if(stat==2) return CYRET_UNKNOWN;
		   }
		   
	i2c_buffer[0] = 7; //CURRENT SHUNT LIMIT
	i2c_buffer[1] = INA226_LIMIT>>8;    //high byte
	i2c_buffer[2] = INA226_LIMIT&0xff;
	
	stat = I2C_1_MasterClearStatus();
	I2C_1_MasterWriteBuf(address,i2c_buffer,3,I2C_1_MODE_COMPLETE_XFER);
	stat=0;
	while(stat==0){
			stat = I2C_1_MasterStatus();
			stat = translate_status(stat);
			if(stat==2) return CYRET_UNKNOWN;
		   }	   
	
	i2c_buffer[0] = 6; 
	i2c_buffer[1] = 0x80;//high bit on, overvoltage on shunt
	i2c_buffer[2] = 1;
	
	stat = I2C_1_MasterClearStatus();
	I2C_1_MasterWriteBuf(address,i2c_buffer,3,I2C_1_MODE_COMPLETE_XFER);
	stat=0;
	while(stat==0){
			stat = I2C_1_MasterStatus();
			stat = translate_status(stat);
			if(stat==2) return CYRET_UNKNOWN;
		   }
		   
	return CYRET_SUCCESS;
}

int read_tmp100(int channel)
{
	uint8 stat=0;
	uint8 address = 0;
	if( check_chan(channel)!=CYRET_SUCCESS) return CYRET_BAD_PARAM;
	if(channel==CH1) address = CHAN1_TMP100;
	else if(channel==CH2) address = CHAN2_TMP100;
	else return CYRET_BAD_PARAM;
	/*
	i2c_buffer[0] = 0; //register 0 is tmp
	stat = I2C_1_MasterClearStatus();
	I2C_1_MasterWriteBuf(address,i2c_buffer,1,I2C_1_MODE_NO_STOP);	
	stat=0;
	while(stat==0){
			stat = I2C_1_MasterStatus();
			stat = translate_status(stat);
			if(stat==2) return CYRET_UNKNOWN;
		   }
	I2C_1_MasterReadBuf(address,i2c_buffer,2,I2C_1_MODE_REPEAT_START);	
	
	do{
			stat = I2C_1_MasterStatus();
			if(stat&I2C_1_MSTAT_ERR_MASK) return CYRET_UNKNOWN;
	}while(stat&I2C_1_MSTAT_XFER_INP); 
	*/
	
	if(CYRET_SUCCESS!=read_16reg(address, 0 )) 
		return CYRET_UNKNOWN;
		
	tmp100_status[channel] = ((((int16)i2c_buffer[0]<<4) | ((int16)i2c_buffer[1]>>4))/ 2047.0 * 128.0);
	return CYRET_SUCCESS;
}

int read_ina226(int channel)
{
	uint8 stat=0;
	uint8 address = 0;
	int16 raw_value;
	if( check_chan(channel)!=CYRET_SUCCESS) return CYRET_BAD_PARAM;
	if(channel==CH1) address = CHAN1_INA226;
	else if(channel==CH2) address = CHAN2_INA226;
	else return CYRET_BAD_PARAM;
	
	if(CYRET_SUCCESS!=read_16reg(address, 1 )) //1 = SHUNT VOL
		return CYRET_UNKNOWN;
	raw_value = (((int16)i2c_buffer[0]<<8) | ((int16)i2c_buffer[1]));
	ina226_status[channel].shunt_voltage = (raw_value/ 32767.0 * 81.92);
	
	if(CYRET_SUCCESS!=read_16reg(address, 2 )) //2 == BUS VOLTAGE
		return CYRET_UNKNOWN;
	raw_value = (((int16)i2c_buffer[0]<<8) | ((int16)i2c_buffer[1]));
	ina226_status[channel].bus_voltage = (raw_value/ 32767.0 * 40.96);
	
	if(CYRET_SUCCESS!=read_16reg(address, 3 )) //3 == POWER
		return CYRET_UNKNOWN;
	raw_value = (((int16)i2c_buffer[0]<<8) | ((int16)i2c_buffer[1]));
	ina226_status[channel].power = (raw_value/ 32767.0 * 2 *25);
	
	if(CYRET_SUCCESS!=read_16reg(address, 4 )) //4 == CURRENT
		return CYRET_UNKNOWN;
	raw_value = (((int16)i2c_buffer[0]<<8) | ((int16)i2c_buffer[1]));
	ina226_status[channel].current = (raw_value/ 32767.0 * 2000);
	
	
	return CYRET_SUCCESS;


}