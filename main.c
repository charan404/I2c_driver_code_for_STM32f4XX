#include <stdio.h>
#include "I2C_DriverInit.h" 
#include "GPIO_Driver.h"
#define MASTER_OPERATION
GPIO_TypeDef *gpio;
i2c_handle_t handle;
int main()  
{
	uint8_t master_write_reg=0;
	uint32_t master_write_reg_len=0;
	uint8_t master_write_reg_buff[5]={10,20,50,40,60};
	uint8_t master_read_reg=0;
	 GPIO_ClockConfig(GPIOPORT_B);
	/*******enable the I2C clock***************/
	_Hal_I2C_1_PERIPHERAL_CLOCK_ENABLE();
	/*******Initialize the GPIO's for SDA SCL***************/
	I2c_GPIOS_Initialization();
	/********Configure the i2c parameters****/
	handle.Instnce=I2C_1;
	handle.Init.AckEnable=I2C_ACK_ENABLE;
	handle.Init.Clock_speed=400000;
	handle.Init.AddressingMode=I2C_ADDRMODE_7BIT;
	handle.Init.DutyCycle=I2C_FM_DUTY_2;
	handle.Init.generalcall_mode=0;
	handle.Init.NoStretchMode=I2C_ENABLE_CLK_STRETCH;
	handle.Init.OwnAddress1=0X18;
	handle.Init.I2C_operating_mode=I2C_ENABLE_FM;
	
	NVIC_EnableIRQ(I2C1_ER_IRQn); // enable the interrupt
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	
	hal_i2c_init(&handle);
	handle.State=HAL_I2C_STATE_READY;
	#ifdef MASTER_OPERATION
	/************************application
	 initialy master send command to slave and then length this indicates cmd->master ant to send data
	 len-> number of bytesw send by the master
	
	*************************/
	master_write_reg=0X11;// cmd
	master_write_reg_len =1;
	hal_i2c_master_tx(&handle,slave_addr_with_write_bit,&master_write_reg, master_write_reg_len); // master send cmd to slave
	
	while(handle.State!= HAL_I2C_STATE_READY);
	master_write_reg=0X5;
	hal_i2c_master_tx(&handle,slave_addr_with_write_bit, &master_write_reg, master_write_reg_len); // master send length to slave
	while(handle.State!= HAL_I2C_STATE_READY);
	
	
	master_write_reg_len =5;
	hal_i2c_master_tx(&handle,slave_addr_with_write_bit,master_write_reg_buff, master_write_reg_len);
	while(handle.State!= HAL_I2C_STATE_READY);
	
	master_write_reg_len =1;
	hal_i2c_master_rx(&handle,slave_addr_with_Read_bit,&master_read_reg, master_write_reg_len);
	while(handle.State!= HAL_I2C_STATE_READY);
	#endif
}
