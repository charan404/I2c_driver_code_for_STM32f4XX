/**
*  Driver code for STM32F446RE board 
*  GPIO driver code 
*  @Developed by 
*  @Name:M.Charn,Embedded software Developer
*	 @Gmail:charanm4407@gmail.com
*/

#include "GPIO_Driver.h"
#include "stm32f446xx.h"
static void GPIOX_AlternateConfig(GPIO_TypeDef *GpioConfig,uint32_t GPIO_Alternate,uint32_t PinNo);
static void GPIOX_ModeConfig(GPIO_TypeDef *GpioConfig,uint32_t MODE,uint32_t PinNo);
static void GPIOX_pushupdownConfig(GPIO_TypeDef *GpioConfig,uint32_t GPIO_PushUpDown,uint32_t PinNo);
static void GPIOX_SpeedConfig(GPIO_TypeDef *GpioConfig,uint32_t GPIO_speed,uint32_t PinNo);
static void GPIOX_OutputTypeConfig(GPIO_TypeDef *GpioConfig,uint32_t OutputType,uint32_t PinNo);


/***************************************************
clock enable 
******************************************************/
void GPIO_ClockConfig(uint32_t GPIO_PORT)
{
	
 if(GPIO_PORT==GPIOPORT_A)
	{
		HAL_GPIO_PORTA_PERIPERAL_CLK_ENABLE();
	}
	else if(GPIO_PORT==GPIOPORT_B)
	{
		HAL_GPIO_PORTB_PERIPERAL_CLK_ENABLE();
	}
	else if(GPIO_PORT==GPIOPORT_C)
	{
		HAL_GPIO_PORTC_PERIPERAL_CLK_ENABLE();
	}
	else if(GPIO_PORT==GPIOPORT_D)
	{
		HAL_GPIO_PORTD_PERIPERAL_CLK_ENABLE();
	}
	else if(GPIO_PORT==GPIOPORT_E)
	{
		HAL_GPIO_PORTE_PERIPERAL_CLK_ENABLE();
	}
	else if(GPIO_PORT==GPIOPORT_F)
	{
		HAL_GPIO_PORTF_PERIPERAL_CLK_ENABLE();
	}
	else if(GPIO_PORT==GPIOPORT_G)
	{
		HAL_GPIO_PORTG_PERIPERAL_CLK_ENABLE();
	}
	else if(GPIO_PORT==GPIOPORT_H)
	{
		HAL_GPIO_PORTH_PERIPERAL_CLK_ENABLE();
	}
	
}
/**************************************************
 set input parameters to GPIO
***************************************************/
void GPIO_Initialization(GPIO_TypeDef *GpioConfig,Gpio_pin_def *GPIO_PinConfig,uint32_t mode,uint32_t outputtype,uint32_t speed,uint32_t alternate,uint32_t pupd,uint32_t PinNo)
{
	GPIO_PinConfig->GpioMode=mode;
	GPIO_PinConfig->GpioPinNo=PinNo;
	GPIO_PinConfig->GpioSpeed=speed;
	GPIO_PinConfig->GPIO_AFT=alternate;
	GPIO_PinConfig->Gpio_outtype=outputtype;
	GPIO_PinConfig->Gpio_PullUpDown=pupd;
	GPIO_Init(GpioConfig,GPIO_PinConfig);
}
/***********************************************************************
*@brief GPIO data writing
 @Param GPIO instance
 @param GPIO write data
 @Param GPIO PIN number
 @retvan GPIO Read data
************************************************************************/
int GPIO_Write(GPIO_TypeDef *GpioConfig,int data,int GPIO_PIN_NO)
{
	if(data)
	{
		GpioConfig->ODR|=(1<<GPIO_PIN_NO);
	}
	else {
		GpioConfig->ODR&=~(1<<GPIO_PIN_NO);
	}
	return 0;
}
/***********************************************************************
*@brief GPIO data reading
 @Param GPIO instance 
 @Param GPIO PIN number
 @retvan GPIO Read data
************************************************************************/
int GPIO_Read(GPIO_TypeDef *GpioConfig,int GPIO_PIN_NO)
{
	int data=0;
	data=(GpioConfig->IDR>>GPIO_PIN_NO&0X00000001);
	return data;
}
/**************************************************************************************************************
@Brief This api is used Initialize the GPIO
@Param Gpio instance 
@Param GPIO Instance user define
@retval None
***************************************************************************************************************/
int GPIO_Init(GPIO_TypeDef *GpioConfig,Gpio_pin_def *GPIO_PinConfig)
{
	/**********set GPIO mode*********/
	GPIOX_AlternateConfig(GpioConfig,GPIO_PinConfig->GPIO_AFT,GPIO_PinConfig->GpioPinNo);
	
	GPIOX_ModeConfig(GpioConfig,GPIO_PinConfig->GpioMode,GPIO_PinConfig->GpioPinNo);
	
	GPIOX_pushupdownConfig(GpioConfig,GPIO_PinConfig->Gpio_PullUpDown,GPIO_PinConfig->GpioPinNo);
	
	GPIOX_SpeedConfig(GpioConfig,GPIO_PinConfig->GpioSpeed,GPIO_PinConfig->GpioPinNo);
	
	GPIOX_OutputTypeConfig(GpioConfig, GPIO_PinConfig->Gpio_outtype,GPIO_PinConfig->GpioPinNo);
	//GpioConfig->MODER |=(GPIO_OUTPUT_MODE<<(2*5));
	//GpioConfig->OTYPER |=(GPIO_OUTPUT_TYPE_PUSH_PULL<<5);
	//GpioConfig->OSPEEDR|=(GPIO_LOW_SPEED<<(2*5));
	//GpioConfig->PUPDR|=(GPIO_NO_PULL_UP_DOWN<<(2*5));
	//GpioConfig->AFR[0]|=(aft<<5*4);
	
	return 0;
}
/**************************************************************************************************************
@Brief This api is used to  GPIO input or output mode
@Param Gpio instance 
@Param GPIO Mode
@Param reprasents GPIO pin Number
@retval None
***************************************************************************************************************/


static void GPIOX_ModeConfig(GPIO_TypeDef *GpioConfig,uint32_t MODE,uint32_t PinNo)
{
   	GpioConfig->MODER|=(MODE<<(2*PinNo));
	
}
/**************************************************************************************************************
@Brief This api is used to set output type to  GPIO
@Param Gpio instance 
@Param GPIO output type type
@Param reprasents GPIO pin Number
@retval None
***************************************************************************************************************/
static void GPIOX_OutputTypeConfig(GPIO_TypeDef *GpioConfig,uint32_t OutputType,uint32_t PinNo)
{
	GpioConfig->OTYPER|=(OutputType<<PinNo);
	
}
/**************************************************************************************************************
@Brief This api is used to set speed to  GPIO
@Param Gpio instance 
@Param GPIO Speed
@Param reprasents GPIO pin Number
@retval None
***************************************************************************************************************/


static void GPIOX_SpeedConfig(GPIO_TypeDef *GpioConfig,uint32_t GPIO_speed,uint32_t PinNo)
{
	GpioConfig->OSPEEDR|=(GPIO_speed<<(2*PinNo));
}
/**************************************************************************************************************
@Brief This api is used to set  to pull up or pull down mode GPIO
@Param Gpio instance 
@Param  Set GPIO pull up down registers internal or external
@Param reprasents GPIO pin Number
@retval None
***************************************************************************************************************/
static void GPIOX_pushupdownConfig(GPIO_TypeDef *GpioConfig,uint32_t GPIO_PushUpDown,uint32_t PinNo)
{
	GpioConfig->PUPDR|=(GPIO_PushUpDown<<(2*PinNo));
}
/**************************************************************************************************************
@Brief This api is used to set alternate function to  GPIO
@Param Gpio instance 
@Param GPIO alternate function type
@Param reprasents GPIO pin Number
@retval None
***************************************************************************************************************/

static void GPIOX_AlternateConfig(GPIO_TypeDef *GpioConfig,uint32_t GPIO_Alternate,uint32_t PinNo)
{
	if(PinNo<=7)
		{
			GpioConfig->AFR[0]|=(GPIO_Alternate<<(4*PinNo));
		}
		else 
		{
			
			GpioConfig->AFR[1]|=(GPIO_Alternate<<(4*(PinNo-8)));
		}
}
/**************************************************************************************************************
* @Brief this api is used to set the trigger type to GPIO's
* @Param reprasents GPIO pin Number
* @Param repraents GPIO trigger type riing or falling edge trigger
* @retval None
***************************************************************************************************************/
void SetGpio_trigger_type(uint32_t PinNo,uint32_t GPIO_Intr_TriggerType  )
{
		if(GPIO_Intr_TriggerType == RISING_EDGE_TRIGGER)
		{
			EXTI->RTSR |= (1<<PinNo);
		}
		else if(GPIO_Intr_TriggerType == FALLING_EDGE_TRIGGERING)
		{
			EXTI->FTSR |=(1<<PinNo);
			
		}
}
/**************************************************************************************************************
@Brief This api is used to enable GPIO interrupt
@Param reprasents GPIO pin Number
@Param Irq_no is enable in NVIC
@retval None
***************************************************************************************************************/
void Gpio_interrupt_enable(uint32_t PinNo,IRQn_Type Irq_no)
{
	EXTI->IMR|=(1<PinNo);
	NVIC_EnableIRQ(Irq_no);
	
}
/*****************************************************************************************************
@Brief This api is used to clear the GPIO interrupt
@Param reprasents GPIO pin Number
@retval None
******************************************************************************************************/
void Gpio_ClearInterrupt(uint32_t PinNo)
{
	if((EXTI->PR &(1<<PinNo)))
	{
		EXTI->PR|=(1<<PinNo);
	}
}
/****************************************************************************************************************
@Brief Connect GPIO interrupt to NVIC through EXTI
@Param 	Irq_no is enable in NVIC
@Param 	Repraents GPIO trigger type riing or falling edge trigger
@Param	Reprasents GPIO pin Number
@Retval None
****************************************************************************************************************/
void GPIO_SetUp_InterruptSystem(IRQn_Type Irq_no,uint32_t GPIO_Intr_TriggerType,uint32_t PinNo)
{
	SetGpio_trigger_type(PinNo,GPIO_Intr_TriggerType);
  Gpio_interrupt_enable(PinNo,Irq_no);
	
}