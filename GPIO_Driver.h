#ifndef __GPIO_DRIVER__
#define __GPIO_DRIVER__
#include "stm32f446xx.h"
							
#define   GPIO_PORTA    GPIOA 
#define   GPIO_PORTB    GPIOB
#define   GPIO_PORTC    GPIOC
#define   GPIO_PORTD    GPIOD
#define   GPIO_PORTE    GPIOE
#define   GPIO_PORTF    GPIOF
#define   GPIO_PORTG    GPIOG
#define   GPIO_PORTH    GPIOH
/*********GPIO Mode******************/
#define GPIO_INPUT_MODE				((uint32_t)0X00)
#define GPIO_OUTPUT_MODE			((uint32_t)0X01)
#define GPIO_ALT_FUN_MODE			((uint32_t)0X02)
#define GPIO_ANALOG_MODE			((uint32_t)0X03)

/*********GPIO SPEED****************/
#define GPIO_LOW_SPEED				((uint32_t)0X00)
#define GPIO_MEDIUM_SPEED			((uint32_t)0X01)
#define GPIO_HIGH_SPEED				((uint32_t)0X02)
#define GPIO_VERY_HIGH_SPEED	((uint32_t)0X03)
/*********GPIO OUTPUT TYPE**********/
#define GPIO_OUTPUT_TYPE_PUSH_PULL  ((uint32_t)0X00)
#define GPIO_OUTPUT_TYPE_OPEN_DRAIN ((uint32_t)0X00)
/*************GPIO PULL UP & PULL DOWN********/
#define GPIO_NO_PULL_UP_DOWN  ((uint32_t)0X00)
#define GPIO_PULL_UP				  ((uint32_t)0X01)
#define GPIO_PULL_DOWN			  ((uint32_t)0X02)
#define GPIO_RESERVED				  ((uint32_t)0X03)
/*************GPIO alternate funationality********/
#define GPIO_PIN_AF0_SYS									0	
#define GPIO_PIN_AF1_TIMER1TO2						1
#define GPIO_PIN_AF2_TIMER3TO5						2
#define GPIO_PIN_AF3_TIMER8TO11						3
#define GPIO_PIN_AF4_I2C_1TO3		  				4
#define GPIO_PIN_AF5_SPI_1TO2		  				5
#define GPIO_PIN_AF6_SPI_3   		  				6
#define GPIO_PIN_AF7_USART1TO3	  				7
#define GPIO_PIN_AF8_USART4TO6	  				8
#define GPIO_PIN_AF9_CAN1TO2_TIM12TO14		9
#define GPIO_PIN_AF10_OTG_FS_OTG_HS	 			10
#define GPIO_PIN_AF11_ETH	  							11
#define GPIO_PIN_AF12_FSMC_SDIO_OTG_HS	  12
#define GPIO_PIN_AF13_DCMI	  						13
#define GPIO_PIN_AF14_	  								14
#define GPIO_PIN_AF15_EVENTOUT	  				15
/****************periperal clock******************/
#define HAL_GPIO_PORTA_PERIPERAL_CLK_ENABLE()	 (RCC->AHB1ENR|=(1<<0))
#define HAL_GPIO_PORTB_PERIPERAL_CLK_ENABLE()	 (RCC->AHB1ENR|=(1<<1))
#define HAL_GPIO_PORTC_PERIPERAL_CLK_ENABLE()	 (RCC->AHB1ENR|=(1<<2))
#define HAL_GPIO_PORTD_PERIPERAL_CLK_ENABLE()	 (RCC->AHB1ENR|=(1<<3))
#define HAL_GPIO_PORTE_PERIPERAL_CLK_ENABLE()	 (RCC->AHB1ENR|=(1<<4))
#define HAL_GPIO_PORTF_PERIPERAL_CLK_ENABLE()	 (RCC->AHB1ENR|=(1<<5))
#define HAL_GPIO_PORTG_PERIPERAL_CLK_ENABLE()	 (RCC->AHB1ENR|=(1<<6))
#define HAL_GPIO_PORTH_PERIPERAL_CLK_ENABLE()	 (RCC->AHB1ENR|=(1<<7))
enum GPIO_port{
	GPIOPORT_A=0,
	GPIOPORT_B, 
	GPIOPORT_C,
	GPIOPORT_D,
	GPIOPORT_E,
	GPIOPORT_F,
	GPIOPORT_G,
	GPIOPORT_H
};
typedef struct{
	uint32_t GpioPinNo;
	uint32_t GpioMode;
	uint32_t GpioSpeed;
	uint32_t	Gpio_outtype;
	uint32_t	Gpio_PullUpDown;
	uint32_t  GPIO_AFT;
}Gpio_pin_def;

enum Trigger_type{
	RISING_EDGE_TRIGGER =0,
	FALLING_EDGE_TRIGGERING
};
int GPIO_Init(GPIO_TypeDef *GpioConfig,Gpio_pin_def *GPIO_PinConfig);
int GPIO_Write(GPIO_TypeDef *GpioConfig,int data,int GPIO_PIN_NO);
int GPIO_Read(GPIO_TypeDef *GpioConfig,int GPIO_PIN_NO);
void Gpio_ClearInterrupt(uint32_t PinNo);
void Gpio_interrupt_enable(uint32_t PinNo,IRQn_Type Irq_no);
void SetGpio_trigger_type(uint32_t PinNo,uint32_t GPIO_Intr_TriggerType);
void GPIO_ClockConfig(uint32_t GPIO_PORT);
void GPIO_SetUp_InterruptSystem(IRQn_Type Irq_no,uint32_t GPIO_Intr_TriggerType,uint32_t PinNo);
void GPIO_Initialization(GPIO_TypeDef *GpioConfig,Gpio_pin_def *GPIO_PinConfig,uint32_t mode,uint32_t outputtype,uint32_t speed,uint32_t alternate,uint32_t pupd,uint32_t PinNo);
#endif
