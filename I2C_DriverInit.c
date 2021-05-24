#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include "I2C_DriverInit.h"
#include "GPIO_Driver.h" 
extern GPIO_TypeDef *gpio;
extern i2c_handle_t handle;
/************************************************************************************************************ 
@Brief SPI_GPIOS_Initialization api is used to set th MISO,MOSI,CLK pins

@retval None
************************************************************************************************************/
void I2c_GPIOS_Initialization()
{
	
	gpio = GPIOB;
	Gpio_pin_def GpioConfig,GpioConfig1,GpioConfig2;
	

  /******MISO pin Initialization******/	
	GPIO_Initialization(gpio,&GpioConfig1, GPIO_ALT_FUN_MODE, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_MEDIUM_SPEED,GPIO_PIN_AF4_I2C_1TO3,GPIO_PULL_UP,I2c_SCL_PIN);
	/******MOSI pin Initialization******/
	GPIO_Initialization(gpio,&GpioConfig2, GPIO_ALT_FUN_MODE, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_MEDIUM_SPEED,GPIO_PIN_AF4_I2C_1TO3,GPIO_PULL_UP,I2c_SDA_PIN);
}
/***********************************************************************************************************************
	@brief API used to enable the I2C Peripheral
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
***********************************************************************************************************************/
void hal_i2c_enable_peripheral(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |=I2C_REG_CR1_PERIPHERAL_ENABLE;
}

/***********************************************************************************************************************
	@brief API used to disable the I2C Peripheral
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
***********************************************************************************************************************/
void hal_i2c_Disable_peripheral(I2C_TypeDef *i2cx)
{
	i2cx->CR1 &=~I2C_REG_CR1_PERIPHERAL_ENABLE;
}
/***********************************************************************************************************************
@brief API used to manage the I2C Peripheral
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
param no_stretch : clock stretch enable or disable
@retval None
***********************************************************************************************************************/
void hal_i2c_manage_clock_stretching(I2C_TypeDef *i2cx,uint32_t no_stretch)
{
	 if(no_stretch)// disables the clock stretch
	 {
		 i2cx->CR1 |=I2C_REG_CR1_NOSTRETCH;
	 }
	 else
	 {
		 i2cx->CR1 &=~I2C_REG_CR1_NOSTRETCH;
	 }
}
/***********************************************************************************************************************
@brief API used to set on address the I2C Peripheral
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
param own_address : set the address of i2c device to be configured
@retval None
***********************************************************************************************************************/
void hal_i2c_set_own_address1(I2C_TypeDef *i2cx,uint32_t own_address)
{
  i2cx->OAR1 &=~(0X7f<<1);
	i2cx->OAR1 |= (own_address<<1); 
}

/***********************************************************************************************************************
@brief API used to set  addressing mode the I2C Peripheral
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
param own_addressing_mode : set the addressing mode of i2c device to be configured
@retval None
***********************************************************************************************************************/
void hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx,uint32_t address_mode)
{
   if(address_mode==I2C_ADDRMODE_10BIT)
	 {
			i2cx->OAR1 |=I2C_REG_OAR1_ADDRMODE;
	 }
	 else
	 {
		 i2cx->OAR1 &=~I2C_REG_OAR1_ADDRMODE;
	 }
}

/***********************************************************************************************************************
@brief API used to set  addressing mode the I2C Peripheral
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
param DutyCycle : set the DutyCycleof i2c device to be configured
@retval None
***********************************************************************************************************************/
void hal_i2c_set_fm_mode_duty_cycle(I2C_TypeDef *i2cx,uint32_t DutyCycle)
{
	if(DutyCycle==I2C_FM_DUTY_16BY9)//16:9
	 {
			i2cx->CCR |=I2C_REG_CCR_DUTY;
	 }
	 else// 1:2
	 {
		 i2cx->CCR &=~I2C_REG_CCR_DUTY;
	 }
}


/***********************************************************************************************************************
	@brief API used to initialization of I2C Peripheral Clock 
}@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@param DutyCycle : set the DutyCycle of i2c device to be configured
@param clkspeed  : I2c clock speed
@retval None
***********************************************************************************************************************/
void hal_i2c_configure_ccr(I2C_TypeDef *i2cx,uint32_t Pclk1,uint32_t clock_speed,uint32_t DutyCycle,uint8_t Operat_mode)
{
			double Tlow=0,Thigh=0,SCL =0,Tpclk =0;
		uint32_t CCR_value =0;
	
	  Tpclk = (1/(Pclk1*1000000.0));
		if(Operat_mode==I2C_ENABLE_SM)
		{
			i2cx->CCR&= ~I2C_REG_CCR_ENABLE_FM; // stanard mode
			Thigh = ((1/clock_speed)/2);
			
			CCR_value=(uint32_t)(Thigh/Tpclk);
			i2cx->CCR|=CCR_value<<0;
		}
		else if(Operat_mode==I2C_ENABLE_FM)
		{
			i2cx->CCR |= I2C_REG_CCR_ENABLE_FM; //// Fast mode
			
			if(DutyCycle==I2C_FM_DUTY_2)
			{
				i2cx->CCR &= ~I2C_REG_CCR_DUTY;
				
				Thigh = ((1/clock_speed)/3);
				CCR_value=(uint32_t)(Thigh/Tpclk);
			  i2cx->CCR|=CCR_value<<0;
				
			}
			else if(DutyCycle==I2C_FM_DUTY_16BY9)
			{
				i2cx->CCR |= I2C_REG_CCR_DUTY;
				Thigh = ((1/clock_speed)/2.7);
				CCR_value=(uint32_t)(Thigh/Tpclk);
			  i2cx->CCR|=CCR_value<<0;
				
			}
			
		}
	
}
void hal_i2c_rise_time_configuration(I2C_TypeDef *i2cx,uint32_t Pclk1,uint32_t clock_speed,uint8_t Operat_mode)
{
	double Tpclk = 0,Tscl=0;
	uint8_t RiseTime=0;
	Tpclk = (1/(Pclk1*1000000.0));
	Tscl =(1/clock_speed);
	RiseTime=((uint32_t)(Tscl/Tpclk))+1;
	i2cx->TRISE|=RiseTime; 
	
}	

void hal_i2c_clock_init(I2C_TypeDef *i2cx,uint32_t clock_speed,uint32_t DutyCycle,uint8_t Operat_mode)
{
	uint32_t Pclk =I2C_PERIPHERAL_CLK_FREQ_10MHZ;
	i2cx->CR2 &=~(0X3F);
	i2cx->CR2 |=(Pclk&0X3F);
	hal_i2c_configure_ccr(i2cx,Pclk,clock_speed,DutyCycle,Operat_mode);
	hal_i2c_rise_time_configuration(i2cx,Pclk,clock_speed,Operat_mode);
}




/***********************************************************************************************************************
	@brief API used to generate start condition
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
***********************************************************************************************************************/
void hal_i2c_Start_condition_gen(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |=I2C_REG_CR1_START;
}
/***********************************************************************************************************************
	@brief API used to generate stop condition
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
***********************************************************************************************************************/
void hal_i2c_Stop_condition_gen(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |=I2C_REG_CR1_STOP;
}

/***********************************************************************************************************************
@brief API used to enable or disable the buffer interrupts(TXE,RXNE the I2C Peripheral
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@param enable : enable or disbale the i2c buffer interrupts
@retval None
***********************************************************************************************************************/
void hal_i2c_configure_buffer_interrupts(I2C_TypeDef *i2cx,uint32_t enable)
{
	if(enable)
	 {
			i2cx->CR2 |=I2C_REG_CR2_BUF_INTR_ENABLE;
	 }
	 else
	 {
		 i2cx->CR2 &=~I2C_REG_CR2_BUF_INTR_ENABLE;
	 }
}
/***********************************************************************************************************************
@brief API used to enable or disable the error interrupts(TXE,RXNE the I2C Peripheral
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@param enable : enable or disbale the i2c buffer interrupts
@retval None
***********************************************************************************************************************/
void hal_i2c_configure_error_interrupts(I2C_TypeDef *i2cx,uint32_t enable)
{
	if(enable)
	 {
			i2cx->CR2 |=I2C_REG_CR2_ERROR_INTR_ENABLE;
	 }
	 else
	 {
		 i2cx->CR2 &=~I2C_REG_CR2_ERROR_INTR_ENABLE;
	 }
}
/***********************************************************************************************************************
@brief API used to enable or disable the event interrupts(TXE,RXNE the I2C Peripheral
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@param enable : enable or disbale the i2c event interrupts
@retval None
***********************************************************************************************************************/
void hal_i2c_configure_event_interrupts(I2C_TypeDef *i2cx,uint32_t enable)
{
	if(enable)
	 {
			i2cx->CR2 |=I2C_REG_CR2_EVENT_INTR_ENABLE;
	 }
	 else
	 {
		 i2cx->CR2 &=~I2C_REG_CR2_EVENT_INTR_ENABLE;
	 }
}
/***********************************************************************************************************************
@brief API used check bus state
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@retval bus free 0 or busy 1
***********************************************************************************************************************/
uint8_t I2c_bus_busy(I2C_TypeDef *i2cx)
{
	if(i2cx->SR2&I2C_REG_SR2_BUS_BUSY_FLAG)
	{
		 return 1;
	}
	else
	{
		return 0;
	}
	
}
/***********************************************************************************************************************
@brief call this api wait until sb bit set
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
***********************************************************************************************************************/
void I2c_wait_until_sb_set(I2C_TypeDef *i2cx)
{
	while(!(i2cx->SR2&I2C_REG_SR1_SB_FLAG));
	
}
/***********************************************************************************************************************
@brief call this api wait until Addr bit set
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
***********************************************************************************************************************/
void I2c_wait_until_Addr_set(I2C_TypeDef *i2cx)
{
	while(!(i2cx->SR2&I2C_REG_SR1_ADDR_SEND_FLAG));
	
}
/************************************************************************************************************************
	@brief Initialize the I2C Peripheral
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
void hal_i2c_init(i2c_handle_t *handle)
{
	/***********I2c clock init**********************/
	hal_i2c_clock_init(handle->Instnce,handle->Init.Clock_speed,handle->Init.DutyCycle,handle->Init.I2C_operating_mode);
	
	/***********set I2c addressing mode**********************/
	hal_i2c_set_addressing_mode(handle->Instnce,handle->Init.AddressingMode);
	/***********Enable the ACKing**********************/
	handle->Instnce->CR1 |= I2C_REG_CR1_ACK;// enable the acking 
	
	/***********enable clcok stretching**********************/
	hal_i2c_manage_clock_stretching(handle->Instnce,handle->Init.NoStretchMode);
	/***********Configure the on address**********************/
	hal_i2c_set_own_address1(handle->Instnce,handle->Init.OwnAddress1);
	/***********Finaly enable the i2c peripheral**********************/
	hal_i2c_enable_peripheral(handle->Instnce);
}
/************************************************************************************************************************
	@brief API to do master addrs trasfer
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@retval None

************************************************************************************************************************/
void hal_i2c_send_addr_first(i2c_handle_t *handle,uint8_t SlaveAddr)
{
	handle->Instnce->DR= SlaveAddr;
}
/************************************************************************************************************************
	@brief API to do master data trasfer
@param *i2cx: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
 void I2c_clear_addr_flag(I2C_TypeDef *i2cx)
 {
	 uint32_t clear_sr1,clear_sr2;
	 clear_sr1=i2cx->SR1; // whenever read the sr1 and sr2 registers the data in  that registers cleared automatically
	 clear_sr2=i2cx->SR2;
	 
 }
 /************************************************************************************************************************
	@brief API to do master data trasfer
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
 void hal_i2c_clear_stop_flag(I2C_TypeDef *i2cx)
 {
	 uint32_t clear_sr1,clear_sr2;
	 clear_sr1=i2cx->SR1; // whenever read the sr1 and sr2 registers the data in  that registers cleared automatically
	 i2cx->CR1 |= 1<<0;  
	 
 }
/************************************************************************************************************************
	@brief API to do master data trasfer
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@param SlaveAddr: Address we want to tx
@param *buffer: holds the pointer to tx buffer
@param len: len of the data to be tx
@retval None
************************************************************************************************************************/
void hal_i2c_master_tx(i2c_handle_t *handle,uint8_t SlaveAddr, uint8_t *buffer, uint8_t len)
{
	handle->pBufPtr = buffer;
	handle->XferCount=len;
	handle->XferSize=len;
	handle->State= HAL_I2C_STATE_BUSY_TX;
	/**************************Enable the peripheral******************************************/
	hal_i2c_enable_peripheral(handle->Instnce);
	/**************************master generate start ondition*********************************/
	hal_i2c_Start_condition_gen(handle->Instnce);
	/************************* wait for SB flag set******************************************/
	I2c_wait_until_sb_set(handle->Instnce);
	/*************************send slave address**********************************************/
	hal_i2c_send_addr_first(handle,SlaveAddr);
	/*************************wait for addr flag set*****************************************/
	I2c_wait_until_Addr_set(handle->Instnce);
	/*************************clear the addr flag********************************************/
  I2c_clear_addr_flag(handle->Instnce);
	 
	/****************enable the buffer ,event and error interrupts are enable****************/
	hal_i2c_configure_buffer_interrupts(handle->Instnce,1);
	
	hal_i2c_configure_error_interrupts(handle->Instnce,1);
	
	hal_i2c_configure_event_interrupts(handle->Instnce,1);

}

/************************************************************************************************************************
	@brief API to do master data reception
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@param SlaveAddr: Address who send the data along with r/w
@param *buffer: holds the pointer to rx buffer
@param len: len of the data to be rx
@retval None
************************************************************************************************************************/
void hal_i2c_master_rx(i2c_handle_t *handle,uint8_t SlaveAddr, uint8_t *buffer, uint8_t len)
{
	handle->pBufPtr = buffer;
	handle->XferCount=len;
	handle->XferSize=len;
	handle->State= HAL_I2C_STATE_BUSY_RX;
	/**************************Enable the peripheral******************************************/
	hal_i2c_enable_peripheral(handle->Instnce);
	/*********disable the POS ****************************************************************/
	handle->Instnce->CR1 &=~I2C_REG_CR1_POS;
	handle->Instnce->CR1 |= I2C_REG_CR1_ACK;// enable the acking 
	/**************************master generate start ondition*********************************/
	hal_i2c_Start_condition_gen(handle->Instnce);
	/************************* wait for SB flag set******************************************/
	I2c_wait_until_sb_set(handle->Instnce);
	/*************************send slave address**********************************************/
	hal_i2c_send_addr_first(handle,SlaveAddr);
	/*************************wait for addr flag set*****************************************/
	I2c_wait_until_Addr_set(handle->Instnce);
	/****************single byte trasmission or reception *************************************************/
	if(len==1)
	{
		/***disable the acking becouse single byte trasmission or reception if send again ack to
		slave slave think that master want one more data so to avois this disable the acking**/
		handle->Instnce->CR1 &= ~I2C_REG_CR1_ACK;
		/**************************master generate stop ondition*********************************/
	  hal_i2c_Stop_condition_gen(handle->Instnce);
	}
	/*************************clear the addr flag********************************************/
  I2c_clear_addr_flag(handle->Instnce);
	 
	/****************enable the buffer ,event and error interrupts are enable****************/
	hal_i2c_configure_buffer_interrupts(handle->Instnce,1);
	
	hal_i2c_configure_error_interrupts(handle->Instnce,1);
	
	hal_i2c_configure_event_interrupts(handle->Instnce,1);
	
}

/************************************************************************************************************************
	@brief API to do slave data RECEPTION	
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@param *buffer: holds the pointer to rx buffer
@param len: len of the data to be rx
@retval None
************************************************************************************************************************/
void hal_i2c_slave_rx(i2c_handle_t *handle,uint8_t *buffer, uint8_t len)
{
	handle->XferCount=len;
	handle->XferSize=len;
	handle->State= HAL_I2C_STATE_BUSY_RX;
	
	/*********disable the POS ****************************************************************/
	handle->Instnce->CR1 &=~I2C_REG_CR1_POS;
	/**************************Enable the peripheral******************************************/
	hal_i2c_enable_peripheral(handle->Instnce);
	
	handle->Instnce->CR1 |= I2C_REG_CR1_ACK;// enable the acking
	
	/****************enable the buffer ,event and error interrupts are enable****************/
	hal_i2c_configure_buffer_interrupts(handle->Instnce,1);
	
	hal_i2c_configure_error_interrupts(handle->Instnce,1);
	
	hal_i2c_configure_event_interrupts(handle->Instnce,1);
	
}

/************************************************************************************************************************
	@brief API to do slave data trasmission
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@param *buffer: holds the pointer to Tx buffer
@param len: len of the data to be tx
@retval None
************************************************************************************************************************/
void hal_i2c_slave_tx(i2c_handle_t *handle,uint8_t *buffer, uint8_t len)
{
	handle->XferCount=len;
	handle->XferSize=len;
	handle->State= HAL_I2C_STATE_BUSY_TX;
	
	/*********disable the POS ****************************************************************/
	handle->Instnce->CR1 &=~I2C_REG_CR1_POS;
	/**************************Enable the peripheral******************************************/
	hal_i2c_enable_peripheral(handle->Instnce);
	
	handle->Instnce->CR1 |= I2C_REG_CR1_ACK;// enable the acking
	
	/****************enable the buffer ,event and error interrupts are enable****************/
	hal_i2c_configure_buffer_interrupts(handle->Instnce,1);
	
	hal_i2c_configure_error_interrupts(handle->Instnce,1);
	
	hal_i2c_configure_event_interrupts(handle->Instnce,1);
}
/************************************************************************************************************************
@brief API Handle the master TXE interrupt
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_master_handles_TXE_interrupt(i2c_handle_t *i2cx)
{
	i2cx->Instnce->DR= (*i2cx->pBufPtr++); // data load into Data register
	i2cx->XferCount--; 
	if(i2cx->XferCount==0) //trasmission of data completed
	{
		i2cx->Instnce->CR2&=~I2C_REG_CR2_BUF_INTR_ENABLE; // disable the buffer interrupt
	}
}
/************************************************************************************************************************
@brief API Handle the slave TXE interrupt
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_slave_handles_TXE_interrupt(i2c_handle_t *i2cx)
{
	
	if(i2cx->XferCount!=0) //trasmission of data completed
	{
		i2cx->Instnce->DR= (*i2cx->pBufPtr++); // data load into Data register
	  i2cx->XferCount--; 
	}
}
/************************************************************************************************************************
@brief API Handle the master RXNE interrupt
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_master_handles_RXNE_interrupt(i2c_handle_t *i2cx)
{
	if(i2cx->XferCount!=0)
	{
		(*i2cx->pBufPtr++)=i2cx->Instnce->DR; // data load from Data register
		i2cx->XferCount--;  
	
		//i2cx->Instnce->CR2&=~I2C_REG_CR2_BUF_INTR_ENABLE; // disable the buffer interrupt
	}
}
/************************************************************************************************************************
@brief API Handle the slave RXNE interrupt
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_slave_handles_RXNE_interrupt(i2c_handle_t *i2cx)
{
	if(i2cx->XferCount!=0)
	{
		(*i2cx->pBufPtr++)=i2cx->Instnce->DR; // data load from Data register
		i2cx->XferCount--;  
	
		//i2cx->Instnce->CR2&=~I2C_REG_CR2_BUF_INTR_ENABLE; // disable the buffer interrupt
	}
}
/************************************************************************************************************************
@brief API is called after master complete the tx data
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_i2c_master_tx_complete(i2c_handle_t *i2cx)
{
	// call application callback here if need
}
/************************************************************************************************************************
@brief API is called after master complete the rx data
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_i2c_master_rx_complete(i2c_handle_t *i2cx)
{
	// call application callback here if need
}
/************************************************************************************************************************
@brief API Handle the master BTF(data byte trasmission finish) interrupt. this intterrupt occure when data and shift 
registers are empty.
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_master_TX_handles_BTF_interrupt(i2c_handle_t *i2cx)
{
	
	if(i2cx->XferCount!=0)
	{
		i2cx->Instnce->DR= (*i2cx->pBufPtr++); // data load into Data register
		i2cx->XferCount--;
	}
	else
	{		
	
		i2cx->Instnce->CR2&=~I2C_REG_CR2_BUF_INTR_ENABLE; // disable the buffer interrupt
		i2cx->Instnce->CR2&=~I2C_REG_CR2_EVENT_INTR_ENABLE; // disable the event interrupt
		i2cx->Instnce->CR2&=~I2C_REG_CR2_ERROR_INTR_ENABLE; // disable the Error interrupt
		/************genearte stop condition**************************/
		hal_i2c_Stop_condition_gen(i2cx->Instnce);
		/***********change the I2c state******************************/
		i2cx->State=HAL_I2C_STATE_READY;
		/*******in this function you can call application call back function********/
		hal_i2c_master_tx_complete(i2cx);
	}
	
}
/************************************************************************************************************************
@brief API Handle the slave BTF(data byte trasmission finish) interrupt. this intterrupt occure when data and shift 
registers are empty.
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_slave_TX_handles_BTF_interrupt(i2c_handle_t *i2cx)
{
	
	if(i2cx->XferCount!=0)
	{
		i2cx->Instnce->DR= (*i2cx->pBufPtr++); // data load into Data register
		i2cx->XferCount--;
	}

	
}
/************************************************************************************************************************
@brief API Handle the master BTF(data byte trasmission finish) interrupt. this intterrupt occure when data and shift 
registers are full.
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_master_RX_handles_BTF_interrupt(i2c_handle_t *i2cx)
{
	
	if(i2cx->XferCount!=0)
	{
		(*i2cx->pBufPtr++)=i2cx->Instnce->DR ; // data load from Data register
		i2cx->XferCount--;
	}
	else
	{		
	
		i2cx->Instnce->CR2&=~I2C_REG_CR2_BUF_INTR_ENABLE; // disable the buffer interrupt
		i2cx->Instnce->CR2&=~I2C_REG_CR2_EVENT_INTR_ENABLE; // disable the event interrupt
		i2cx->Instnce->CR2&=~I2C_REG_CR2_ERROR_INTR_ENABLE; // disable the Error interrupt
		/************genearte stop condition**************************/
		hal_i2c_Stop_condition_gen(i2cx->Instnce);
		/***********change the I2c state******************************/
		i2cx->State=HAL_I2C_STATE_READY;
		/*******in this function you can call application call back function********/
		hal_i2c_master_rx_complete(i2cx);
	}
	
}
/************************************************************************************************************************
@brief API Handle the slave BTF(data byte trasmission finish) interrupt. this intterrupt occure when data and shift 
registers are full.
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_slave_RX_handles_BTF_interrupt(i2c_handle_t *i2cx)
{
	
	if(i2cx->XferCount!=0)
	{
		(*i2cx->pBufPtr++)=i2cx->Instnce->DR ; // data load from Data register
		i2cx->XferCount--;
	}

	
}
/************************************************************************************************************************
@brief API Handle the master BTF(data byte trasmission finish) interrupt. this intterrupt occure when data and shift 
registers are empty.
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_slave_handle_stop_condition(i2c_handle_t *i2cx)
{	
	
		i2cx->Instnce->CR2&=~I2C_REG_CR2_BUF_INTR_ENABLE; // disable the buffer interrupt
		i2cx->Instnce->CR2&=~I2C_REG_CR2_EVENT_INTR_ENABLE; // disable the event interrupt
		i2cx->Instnce->CR2&=~I2C_REG_CR2_ERROR_INTR_ENABLE; // disable the Error interrupt
		/************clear stop flag**************************/
		hal_i2c_clear_stop_flag(i2cx->Instnce);
	  i2cx->Instnce->CR1 &= ~I2C_REG_CR1_ACK;// disable the acking
		/***********change the I2c state******************************/
		i2cx->State=HAL_I2C_STATE_READY;
		/*******in this function you can call application call back function********/
		//hal_i2c_master_rx_complete(i2cx);
	
	
}
/************************************************************************************************************************
@brief API handles the slave ack failure
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
static void hal_i2c_slave_handle_ack_failure(i2c_handle_t *i2cx)
{	
	
		i2cx->Instnce->CR2&=~I2C_REG_CR2_BUF_INTR_ENABLE; // disable the buffer interrupt
		i2cx->Instnce->CR2&=~I2C_REG_CR2_EVENT_INTR_ENABLE; // disable the event interrupt
		i2cx->Instnce->CR2&=~I2C_REG_CR2_ERROR_INTR_ENABLE; // disable the Error interrupt
		/************clear AF flag**************************/
		i2cx->Instnce->SR1 &= ~I2C_REG_SR1_AF_FAILURE_FLAG;
	  i2cx->Instnce->CR1 &= ~I2C_REG_CR1_ACK;// disable the acking
		/***********change the I2c state******************************/
		i2cx->State=HAL_I2C_STATE_READY;
		/*******in this function you can call application call back function********/
		//hal_i2c_master_rx_complete(i2cx);
	
	
}
/************************************************************************************************************************
	@brief API used to handle the i2c event interrupts
@param *hi2c: pointer to i2c_handle_t structure that contains the I2c communication information
@retval None
************************************************************************************************************************/
void hal_i2c_handle_evt_interrupt(i2c_handle_t *hi2c)
{
	uint32_t temp1=0,temp2=0,temp3=0,temp4=0;
	if((hi2c->Instnce->SR2&0X1)==1)// master
	{
			temp4 = (hi2c->Instnce->SR2&I2C_REG_SR2_TRA_FLAG);
		if(temp4)// Master tyrasmission
		{
			temp1 = (hi2c->Instnce->SR1& I2C_REG_SR1_TXE_FLAG);
			temp2 = (hi2c->Instnce->CR2&I2C_REG_CR2_BUF_INTR_ENABLE);
			temp3 = (hi2c->Instnce->SR1& I2C_REG_SR1_BTF_FLAG);
			temp4 = (hi2c->Instnce->CR2&I2C_REG_CR2_EVENT_INTR_ENABLE);
			if((temp1 && temp2)&&(!temp3))
			{
				hal_master_handles_TXE_interrupt(hi2c);
			}
			else if((temp3)&&(temp4))
			{
				hal_master_TX_handles_BTF_interrupt(hi2c);
			}
	  }
		else // master reception
		{
			temp1 = (hi2c->Instnce->SR1& I2C_REG_SR1_RXNE_FLAG);
			temp2 = (hi2c->Instnce->CR2&I2C_REG_CR2_BUF_INTR_ENABLE);
			temp3 = (hi2c->Instnce->SR1& I2C_REG_SR1_BTF_FLAG);
			temp4 = (hi2c->Instnce->CR2&I2C_REG_CR2_EVENT_INTR_ENABLE);
			if((temp1&&temp2)&&(!temp3))
			{
				hal_master_handles_RXNE_interrupt(hi2c);
			}
			else if(temp1&&temp2)
			{
				hal_master_RX_handles_BTF_interrupt(hi2c);
			}
		}
	}
	else// slave
	{
		temp1 = (hi2c->Instnce->SR1& I2C_REG_SR1_ADDR_FLAG);
		temp2 = (hi2c->Instnce->CR2&I2C_REG_CR2_EVENT_INTR_ENABLE);
		temp3 = (hi2c->Instnce->SR1& I2C_REG_SR1_STOP_DETECTION_FLAG);
		temp4 = (hi2c->Instnce->SR2&I2C_REG_SR2_TRA_FLAG);
		
		if(temp1&&temp2)
		{
			/*************************clear the addr flag********************************************/
			I2c_clear_addr_flag(hi2c->Instnce);
			
		}
		else if(temp3&&temp2)
		{
			hal_slave_handle_stop_condition(hi2c); // stop flah handle
		}
		else if(temp4) // slave txe
		{
			temp1 = (hi2c->Instnce->SR1& I2C_REG_SR1_TXE_FLAG);
			temp2 = (hi2c->Instnce->CR2&I2C_REG_CR2_BUF_INTR_ENABLE);
			temp3 = (hi2c->Instnce->SR1& I2C_REG_SR1_BTF_FLAG);
			temp4 = (hi2c->Instnce->CR2&I2C_REG_CR2_EVENT_INTR_ENABLE);
			if((temp1 && temp2)&&(!temp3))
			{
				hal_slave_handles_TXE_interrupt(hi2c);
			}
			else if((temp3)&&(temp4))
			{
				hal_slave_TX_handles_BTF_interrupt(hi2c);
			}
		}
		else
		{
			temp1 = (hi2c->Instnce->SR1& I2C_REG_SR1_RXNE_FLAG);
			temp2 = (hi2c->Instnce->CR2&I2C_REG_CR2_BUF_INTR_ENABLE);
			temp3 = (hi2c->Instnce->SR1& I2C_REG_SR1_BTF_FLAG);
			temp4 = (hi2c->Instnce->CR2&I2C_REG_CR2_EVENT_INTR_ENABLE);
			if((temp1&&temp2)&&(!temp3))
			{
				hal_slave_handles_RXNE_interrupt(hi2c);
			}
			else if(temp1&&temp2)
			{
				hal_slave_RX_handles_BTF_interrupt(hi2c);
			}
		}
	}
}
void hal_i2c_error_callback(i2c_handle_t *hi2c)
{
	
	while(1);
}
/************************************************************************************************************************
	@brief API used to handle the i2c error interrupts
@param *hi2c: pointer to i2c_handle_t structure that contains the I2c communication information
@retval None
************************************************************************************************************************/
void hal_i2c_handle_error_interrupt(i2c_handle_t *hi2c)
{
	uint32_t temp1 = 0,temp2 = 0,temp3 = 0;
  /**********Bus error interrupt handle*****************************************************/
	temp1=hi2c->Instnce->SR1&I2C_REG_SR1_BUS_ERROR_FLAG;
	temp2 = (hi2c->Instnce->CR2&I2C_REG_CR2_ERROR_INTR_ENABLE);
	if(temp1&&temp2)
	{
		hi2c->ErrorCode |= HAL_I2C_ERROR_BERR;
		hi2c->Instnce->SR1 &=~I2C_REG_SR1_BUS_ERROR_FLAG;
	}
	 /**********Arbhitration error interrupt handle*****************************************************/
	temp1=hi2c->Instnce->SR1&I2C_REG_SR1_ARLO_FLAG;
	temp2 = (hi2c->Instnce->CR2&I2C_REG_CR2_ERROR_INTR_ENABLE);
	if(temp1&&temp2)
	{
		hi2c->ErrorCode |= HAL_I2C_ERROR_ARLO;
		hi2c->Instnce->SR1 &=~I2C_REG_SR1_ARLO_FLAG;
	}
	 /**********ACK failure error interrupt handle*****************************************************/
	temp1=hi2c->Instnce->SR1&I2C_REG_SR1_AF_FAILURE_FLAG;
	temp2 = (hi2c->Instnce->CR2&I2C_REG_CR2_ERROR_INTR_ENABLE);
	if(temp1&&temp2)
	{
		temp1=hi2c->Instnce->SR1&I2C_REG_SR1_ARLO_FLAG;
		temp2 = hi2c->XferCount;
		temp3 = hi2c->State;
		if((!temp1)&&(temp2==0)&&(temp3== HAL_I2C_STATE_BUSY_TX))
		{
			hal_i2c_slave_handle_ack_failure(hi2c);
		}
		else
		{
			hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
			hi2c->Instnce->SR1 &=~I2C_REG_SR1_AF_FAILURE_FLAG;
			
		}
  }
	 /**********over flow error interrupt handle*****************************************************/
	temp1=hi2c->Instnce->SR1&I2C_REG_SR1_OVER_FLAG;
	temp2 = (hi2c->Instnce->CR2&I2C_REG_CR2_ERROR_INTR_ENABLE);
	if(temp1&&temp2)
	{
		hi2c->ErrorCode |= HAL_I2C_ERROR_OVR;
		hi2c->Instnce->SR1 &=~I2C_REG_SR1_OVER_FLAG;
	}
	
	if(hi2c->ErrorCode!= HAL_I2C_ERROR_NONE)
	{
		
		hi2c->State= HAL_I2C_STATE_READY;
		hi2c->Instnce->CR1 &=~ I2C_REG_CR1_POS;
		hal_i2c_error_callback(hi2c);
	}
	 
}
/**************event handler ***********/
void I2C1_EV_IRQHandler()
{

hal_i2c_handle_evt_interrupt(&handle);

}	
 /**************errors handler ***********/                                          
void I2C1_ER_IRQHandler()
{
	hal_i2c_handle_error_interrupt(&handle);
	
}