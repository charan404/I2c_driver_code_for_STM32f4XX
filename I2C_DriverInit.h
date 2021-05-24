#ifndef _I2C_DIVERINIT_H_
#define _I2C_DIVERINIT_H_
#include "stm32f446xx.h"

#define GPIO_PIN_1	6
#define GPIO_PIN_2	9
#define I2c_SDA_PIN GPIO_PIN_2
#define I2c_SCL_PIN GPIO_PIN_1

#define slave_addr_with_write_bit 0X18
#define slave_addr_with_Read_bit 0X19


/*************************************************************************************************************************
     Bit definations of I2C Control Register 1(I2C_CR1)
*************************************************************************************************************************/
#define I2C_REG_CR1_POS		((uint32_t )1<<11)

#define I2C_REG_CR1_ACK		((uint32_t )1<<10)
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

#define I2C_REG_CR1_STOP		((uint32_t)1<<9)

#define I2C_REG_CR1_START		((uint32_t)1<<8)

#define I2C_REG_CR1_NOSTRETCH		((uint32_t)1<<7)
#define I2C_ENABLE_CLK_STRETCH		0
#define I2C_DISABLE_CLK_STRETCH		1

#define I2C_REG_CR1_PERIPHERAL_ENABLE		((uint32_t)1<<0)

/*************************************************************************************************************************
     Bit definations of I2C Control Register 2(I2C_CR2)
*************************************************************************************************************************/
#define I2C_REG_CR2_BUF_INTR_ENABLE			((uint32_t)1<<10)
#define I2C_REG_CR2_EVENT_INTR_ENABLE		((uint32_t)1<<9)
#define I2C_REG_CR2_ERROR_INTR_ENABLE		((uint32_t)1<<10)

#define I2C_PERIPHERAL_CLK_FREQ_2MHZ		((uint32_t)2 )
#define I2C_PERIPHERAL_CLK_FREQ_3MHZ		((uint32_t)3 )
#define I2C_PERIPHERAL_CLK_FREQ_4MHZ		((uint32_t)4 )
#define I2C_PERIPHERAL_CLK_FREQ_5MHZ		((uint32_t)5 )
#define I2C_PERIPHERAL_CLK_FREQ_6MHZ		((uint32_t)6 )
#define I2C_PERIPHERAL_CLK_FREQ_7MHZ		((uint32_t)7 )
#define I2C_PERIPHERAL_CLK_FREQ_8MHZ		((uint32_t)8 )
#define I2C_PERIPHERAL_CLK_FREQ_9MHZ		((uint32_t)9 )
#define I2C_PERIPHERAL_CLK_FREQ_10MHZ		((uint32_t)10 )
#define I2C_PERIPHERAL_CLK_FREQ_11MHZ		((uint32_t)11 )
#define I2C_PERIPHERAL_CLK_FREQ_12MHZ		((uint32_t)12 )
#define I2C_PERIPHERAL_CLK_FREQ_13MHZ		((uint32_t)13 )
#define I2C_PERIPHERAL_CLK_FREQ_14MHZ		((uint32_t)14 )
#define I2C_PERIPHERAL_CLK_FREQ_15MHZ		((uint32_t)15 )
#define I2C_PERIPHERAL_CLK_FREQ_16MHZ		((uint32_t)16 )

/*************************************************************************************************************************
     Bit definations of I2C_OAR1 Register
*************************************************************************************************************************/
#define I2C_REG_OAR1_ADDRMODE			((uint32_t)1<<15)
#define I2C_ADDRMODE_7BIT					0
#define I2C_ADDRMODE_10BIT				1

#define I2C_REG_OAR1_14BIT		((uint32_t)1<<14)
#define I2C_REG_OAR1_7BIT_ADDRESS_POS		1

/*************************************************************************************************************************
     Bit definations of I2C_SR1 Register
*************************************************************************************************************************/
#define I2C_REG_SR1_TIMEOUT_FLAG					((uint32_t)1<<14)
#define I2C_REG_SR1_OVER_FLAG							((uint32_t)1<<11)
#define I2C_REG_SR1_AF_FAILURE_FLAG				((uint32_t)1<<10)
#define I2C_REG_SR1_ARLO_FLAG							((uint32_t)1<<9)
#define I2C_REG_SR1_BUS_ERROR_FLAG				((uint32_t)1<<8)
#define I2C_REG_SR1_TXE_FLAG							((uint32_t)1<<7)
#define I2C_REG_SR1_RXNE_FLAG							((uint32_t)1<<6)
#define I2C_REG_SR1_STOP_DETECTION_FLAG		((uint32_t)1<<4) // FOR SLAVE
#define I2C_REG_SR1_BTF_FLAG							((uint32_t)1<<2)
#define I2C_REG_SR1_ADDR_FLAG							((uint32_t)1<<1)
#define I2C_REG_SR1_ADDR_SEND_FLAG				((uint32_t)1<<1)// FOR MASTER
#define I2C_REG_SR1_ADDR_MATCHED_FLAG			((uint32_t)1<<1) // FOR SLAVE
#define I2C_REG_SR1_SB_FLAG								((uint32_t)1<<0)

/*************************************************************************************************************************
     Bit definations of I2C_SR2 Register
*************************************************************************************************************************/
#define I2C_REG_SR2_BUS_BUSY_FLAG								((uint32_t)1<<1)
#define I2C_BUS_IS_BUSY													1
#define I2C_BUS_IS_FREE													0

#define I2C_REG_SR2_MSL_FLAG										((uint32_t)1<<0)
#define I2C_MASTER_MODE													1
#define I2C_SLAVE_MODE													0


#define I2C_REG_SR2_TRA_FLAG										((uint32_t)1<<2)
#define I2C_TX_MODE															1
#define I2C_RX_MODE													    0
/*************************************************************************************************************************
     Bit definations of I2C_CCR Register
*************************************************************************************************************************/
#define I2C_REG_CCR_ENABLE_FM										((uint32_t)1<<15)
#define I2C_ENABLE_FM												1
#define I2C_ENABLE_SM										    0

#define I2C_REG_CCR_DUTY												((uint32_t)1<<14)
#define I2C_FM_DUTY_16BY9												1
#define I2C_FM_DUTY_2   												0

/*************************************************************************************************************************
      I2C STATE STRUCTURE DEFINATION
*************************************************************************************************************************/
typedef enum{
	
	HAL_I2C_STATE_RESET			= 0X0, //I2C NOT YET INITIALIZED OR DISABLED
	HAL_I2C_STATE_READY			= 0X1, //I2C INITIALIZED AND READY FOR USE
	HAL_I2C_STATE_BUSY			= 0X2, //I2C INTERNAL PROCESS IS GOINGON
	HAL_I2C_STATE_BUSY_TX		= 0X3, //DATA TRASMISSION PROCESS IS GOINGON
	HAL_I2C_STATE_BUSY_RX		= 0X4, //DATA RECEPTION PROCESS IS GOINGON
	HAL_I2C_STATE_ERROR 		= 0X5 //I2C IN ERROR STATE
	
}Hal_I2C_state_t;
/***********************************************************************************************************************
 I2C ERROR CODES
***********************************************************************************************************************/
#define HAL_I2C_ERROR_NONE		((uint32_t)0x00000000)  // no error
#define HAL_I2C_ERROR_BERR		((uint32_t)0x00000001)  // BERR error
#define HAL_I2C_ERROR_ARLO		((uint32_t)0x00000002)  // ARLO error
#define HAL_I2C_ERROR_AF			((uint32_t)0x00000004)  // AF error
#define HAL_I2C_ERROR_OVR			((uint32_t)0x00000008)  // OVR error
#define HAL_I2C_ERROR_DMA			((uint32_t)0x00000010)  // DMA error
#define HAL_I2C_ERROR_TIMEOUT	((uint32_t)0x00000020)  // TIMEOUT error

// I2C peripheral base address
#define I2C_1 I2C1
#define I2C_2 I2C2
#define I2C_3 I2C3
// macros enable the clock for different i2c peripheral
#define _Hal_I2C_1_PERIPHERAL_CLOCK_ENABLE() (RCC->APB1ENR |= (1<<21))
#define _Hal_I2C_2_PERIPHERAL_CLOCK_ENABLE() (RCC->APB1ENR |= (1<<22))
#define _Hal_I2C_3_PERIPHERAL_CLOCK_ENABLE() (RCC->APB1ENR |= (1<<23))
/***********************************************************************************************************************
 I2C Configuration structure defination
***********************************************************************************************************************/
typedef struct{
	
	uint32_t Clock_speed; // specifies the clock freuency
	uint32_t DutyCycle;   // i2c fast mode duty cycle
	uint32_t OwnAddress1; // first device own address
	uint32_t AddressingMode; // 7 bit or 10 bit addressing mode selection
	uint32_t DualAddressMode; // dual addressing mode is selected
	uint32_t OwnAddress2; // second device own address if dual addressing mode
	uint32_t generalcall_mode; //generl call mode
	uint32_t NoStretchMode; // specifies if no stretch mode selected
	uint32_t AckEnable; // handle ack enable or disable
	uint8_t  I2C_operating_mode; // standared or fast mode
}I2C_init_t;
/***********************************************************************************************************************
 I2C handle structure defination
***********************************************************************************************************************/
typedef struct{
	I2C_TypeDef *Instnce; // I2C register base address
	I2C_init_t	Init;		// I2C configure parameters
	uint8_t			*pBufPtr; //	pointer to i2c trasfer buffer
	uint32_t		XferSize; // transfer size
  uint32_t	XferCount; // transfer count
	Hal_I2C_state_t State;  // I2C communication state
	uint32_t  ErrorCode; // used to hold the error code status
}i2c_handle_t;

#define SET 1
#define RESET !SET

void I2c_GPIOS_Initialization();

/************************************************************************************************************************
	@brief Initialize the I2C Peripheral
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@retval None
************************************************************************************************************************/
void hal_i2c_init(i2c_handle_t *handle);

/************************************************************************************************************************
	@brief API to do master data trasfer
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@param SlaveAddr: Address we want to tx
@param *buffer: holds the pointer to tx buffer
@param len: len of the data to be tx
@retval None
************************************************************************************************************************/
void hal_i2c_master_tx(i2c_handle_t *handle,uint8_t SlaveAddr, uint8_t *buffer, uint8_t len);

/************************************************************************************************************************
	@brief API to do master data reception
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@param SlaveAddr: Address who send the data
@param *buffer: holds the pointer to rx buffer
@param len: len of the data to be rx
@retval None
************************************************************************************************************************/
void hal_i2c_master_rx(i2c_handle_t *handle,uint8_t SlaveAddr, uint8_t *buffer, uint8_t len);

/************************************************************************************************************************
	@brief API to do slave data trasmission
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@param *buffer: holds the pointer to Tx buffer
@param len: len of the data to be tx
@retval None
************************************************************************************************************************/
void hal_i2c_slave_tx(i2c_handle_t *handle,uint8_t *buffer, uint8_t len);

/************************************************************************************************************************
	@brief API to do slave data RECEPTION	
@param *handle: Handle to the i2c peripheral ,which the application wats to initialization
@param *buffer: holds the pointer to rx buffer
@param len: len of the data to be rx
@retval None
************************************************************************************************************************/
void hal_i2c_slave_rx(i2c_handle_t *handle,uint8_t *buffer, uint8_t len);

/************************************************************************************************************************
	@brief API used to handle the i2c error interrupts
@param *hi2c: pointer to i2c_handle_t structure that contains the I2c communication information
@retval None
************************************************************************************************************************/
void hal_i2c_handle_error_interrupt(i2c_handle_t *hi2c);

/************************************************************************************************************************
	@brief API used to handle the i2c event interrupts
@param *hi2c: pointer to i2c_handle_t structure that contains the I2c communication information
@retval None
************************************************************************************************************************/
void hal_i2c_handle_evt_interrupt(i2c_handle_t *hi2c);

#endif //_I2C_DIVERINIT_H_