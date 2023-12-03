/*============================================================================================
 * Module : GPIO
 *
 * File Name : stm32f103c8_gpio_driver.c
 *
 * Author: Abdullah Maher
 *
 * Description : Source File Of STM32F103C8 GPIO Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8_gpio_driver.h"


/*===============================================================================
 *                            		  MACROS	                                 *
 ================================================================================*/

/*===============================================================================
 *                              Global Variables                                 *
 ================================================================================*/



/*===============================================================================
 *                          Private Function Prototypes	   		                 *
 ================================================================================*/

/**===============================================================================
 * Function Name  : GET_PIN_POSITION.
 * Brief          : Function To Get the Position of a Specific Pin in a Specific Port
 * Parameter (in) : GPIOx x Could Be A,B,C,D and E.
 * Parameter (in) : a_PinNumber the Number Of Pin.
 * Return         : Position of Pin.
 * Note           : Not API                                                                           */
static uint8_t GET_PIN_POSITION(uint8_t a_PinNumber);


/*===============================================================================
 *                              API Definitions                                  *
 ================================================================================*/

void MCAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_PinConfig_t* p_PinConfig)
{
	vuint32_t* CRHL = NULL_PTR;
	uint8_t tempPinConfig = 0;/*Temporary Variable to Assign CRHL by (MODE + CNF) Bits*/

	/*Specifies Which Configuration Register Will Be Used (CRL 0>>>7) Or (CRH 8>>>15)*/
	CRHL = ((p_PinConfig->GPIO_PinNumber) < GPIO_PIN8)?(&GPIOx->CRL):(&GPIOx->CRH);

	/*Clear Default Value*/
	(*CRHL) &= (~((0xF) << GET_PIN_POSITION(p_PinConfig->GPIO_PinNumber)));

	/*If The Pin Is OUTPUT*/
	if((p_PinConfig->GPIO_Mode == GPIO_MODE_OUTPUT_PUSHPULL) || (p_PinConfig->GPIO_Mode == GPIO_MODE_OUTPUT_OPENDRAIN) || (p_PinConfig->GPIO_Mode == GPIO_MODE_OUTPUT_AF_PUSHPULL) || (p_PinConfig->GPIO_Mode == GPIO_MODE_OUTPUT_AF_OPENDRAIN))
	{
		/*Macros of OUTPUT MODE Assigned With Values To be Manipulated to get The Values that Set CNFy[1:0] MODEy[1:0] */
		tempPinConfig = ( ( (((p_PinConfig->GPIO_Mode) - 4) << 2) | (p_PinConfig->GPIO_Output_Speed) ) & 0x0F );
	}
	else/*Else The Pin Is INPUT*/
	{
		if((p_PinConfig->GPIO_Mode == GPIO_MODE_INPUT_FLOATING) || (p_PinConfig->GPIO_Mode == GPIO_MODE_INPUT_AF_FLOATING))
		{
			/*CNFy[1:0] = 01: Floating input*/
			tempPinConfig = ( (((GPIO_MODE_INPUT_FLOATING) <<2) | 0x0 ) & 0x0F );

		}else if((p_PinConfig->GPIO_Mode == GPIO_MODE_INPUT_ANALOG))
		{
			/*CNFy[1:0] = 00: Analog mode */
			tempPinConfig = ( (((GPIO_MODE_INPUT_ANALOG) <<2) | 0x0 ) & 0x0F );

		}else
		{
			/*CNFy[1:0] = 10: Input with pull-up / pull-down */
			tempPinConfig = ( (((GPIO_MODE_INPUT_PULLUP) <<2) | 0x0 ) & 0x0F );

			/*For PULLUP set xODR bit 1 , PULLDOWN set xODR bit 0 */
			if(p_PinConfig->GPIO_Mode == GPIO_MODE_INPUT_PULLDOWN)
			{
				CLEAR_BIT((GPIOx->ODR),(p_PinConfig->GPIO_PinNumber));
			}else
			{
				SET_BIT((GPIOx->ODR),(p_PinConfig->GPIO_PinNumber));
			}
		}
	}
	/*Assign the CRH or CRL With Pin Configuration*/
	(*CRHL) |= ( (tempPinConfig) << (GET_PIN_POSITION(p_PinConfig->GPIO_PinNumber)));
}


void MCAL_GPIO_DeInit(GPIO_TypeDef *GPIOx)
{
	if(GPIOx == GPIOA)
	{
		/*Bit2 IOPARST: IO port A reset, Set and cleared by software.*/
		SET_BIT((RCC->APB2RSTR),(GPIO_PIN2));
		CLEAR_BIT((RCC->APB2RSTR),(GPIO_PIN2));

	}else if(GPIOx == GPIOB)
	{
		/*Bit3 IOPBRST: IO port B reset, Set and cleared by software.*/
		SET_BIT((RCC->APB2RSTR),(GPIO_PIN3));
		CLEAR_BIT((RCC->APB2RSTR),(GPIO_PIN3));

	}else if(GPIOx == GPIOC)
	{
		/*Bit4 IOPCRST: IO port C reset, Set and cleared by software.*/
		SET_BIT((RCC->APB2RSTR),(GPIO_PIN4));
		CLEAR_BIT((RCC->APB2RSTR),(GPIO_PIN4));

	}else if(GPIOx == GPIOD)
	{
		/*Bit5 IOPDRST: IO port D reset, Set and cleared by software.*/
		SET_BIT((RCC->APB2RSTR),(GPIO_PIN5));
		CLEAR_BIT((RCC->APB2RSTR),(GPIO_PIN5));

	}else if(GPIOx == GPIOE)
	{
		/*Bit6 IOPERST: IO port E reset, Set and cleared by software.*/
		SET_BIT((RCC->APB2RSTR),(GPIO_PIN6));
		CLEAR_BIT((RCC->APB2RSTR),(GPIO_PIN6));
	}

}


uint8_t MCAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8_t a_PinNumber)
{
	uint8_t pinValue;
	pinValue = GET_BIT((GPIOx->IDR),(a_PinNumber));

	return pinValue;
}


void MCAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint8_t a_PinNumber, uint8_t a_Value)
{
	WRI_BIT((GPIOx->ODR),(a_PinNumber),(a_Value));
}

uint16_t MCAL_GPIO_ReadPort(GPIO_TypeDef *GPIOx)
{
	uint16_t portValue;
	portValue = (uint16_t)(GPIOx->IDR);

	return 	portValue;
}

void MCAL_GPIO_WritePort(GPIO_TypeDef *GPIOx,uint16_t a_Value)
{
	(GPIOx->ODR) = (uint16_t)a_Value;
}

void MCAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint8_t a_PinNumber)
{
	TOGGLE_BIT((GPIOx->ODR),(a_PinNumber));
}

uint8_t MCAL_GPIO_LockPin(GPIO_TypeDef *GPIOx, uint8_t a_PinNumber)
{
	/*Set The Bit The Desired Bit To Be Locked*/
	volatile uint32_t temp = (1<<16) ;

	SET_BIT((temp),(a_PinNumber));

	/*Writing The Sequence to Lock Desired Bit :
	Write 1
	Write 0
	Write 1
	Read 0
	Read 1 (this read is optional but confirms that the lock is active)*/

	(GPIOx->LCKR) = temp;            /*Write 1*/
	(GPIOx->LCKR) = (1<<a_PinNumber);/*Write 0*/
	(GPIOx->LCKR) = temp;			 /*Write 1*/
	temp = (GPIOx->LCKR);			 /*Read 0*/

	if(GET_BIT((GPIOx->LCKR),(16)))
		return TRUE;
	else
		return FALSE;

}

/*===============================================================================
 *                        Private Function Definitions                           *
 ================================================================================*/

static uint8_t GET_PIN_POSITION(uint8_t a_PinNumber)
{
	switch(a_PinNumber)
	{
	case GPIO_PIN0:
	case GPIO_PIN8:
		return 0;
		break;
	case GPIO_PIN1:
	case GPIO_PIN9:
		return 4;
		break;
	case GPIO_PIN2:
	case GPIO_PIN10:
		return 8;
		break;
	case GPIO_PIN3:
	case GPIO_PIN11:
		return 12;
		break;
	case GPIO_PIN4:
	case GPIO_PIN12:
		return 16;
		break;
	case GPIO_PIN5:
	case GPIO_PIN13:
		return 20;
		break;
	case GPIO_PIN6:
	case GPIO_PIN14:
		return 24;
		break;
	case GPIO_PIN7:
	case GPIO_PIN15:
		return 28;
		break;
	default:
		break;
	}
	return 0;
}
