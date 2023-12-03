/*============================================================================================
 * Module : EXTI
 *
 * File Name : stm32f103c8_exti_driver.c
 *
 * Author: Abdullah Maher
 *
 * Description : Source File Of STM32F103C8 EXTI Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8_exti_driver.h"
#include "stm32f103c8_gpio_driver.h"

/*===============================================================================
 *                            		  MACROS	                                 *
 ================================================================================*/
#define AFIO_EXTI_GPIOx(x)  ((x == GPIOA)? 0 : (x == GPIOB)? 1 : (x == GPIOC)? 2 : (x == GPIOD)? 3 : 0)


/*===============================================================================
 *                              Global Variables                                 *
 ================================================================================*/
/* Array of 15 Pointer To Function */
void (* gp_EXTI_ISR_CallBack[15])(void);

/*===============================================================================
 *                        Private Function Prototypes	   		                 *
 ================================================================================*/

/**===============================================================================
 * Function Name  : EXTI_initUpdate.
 * Brief          : Function To Init Or Update The EXTIx.
 * Parameter (in) : Pointer To EXTI Configuration.
 * Return         : None.
 * Note           : Not API                                                                           */
static void EXTI_initUpdate(EXTI_PinConfig_t *p_EXTI_Config);

/**===============================================================================
 * Function Name  : NVIC_enable.
 * Brief          : Function To Enable IRQ From EXTI Line.
 * Parameter (in) : IRQ Number.
 * Return         : None.
 * Note           : Not API                                                                           */
static void NVIC_enable(uint8_t a_IRQ);
/**===============================================================================
 * Function Name  : NVIC_disable.
 * Brief          : Function To Disable IRQ From EXTI Line.
 * Parameter (in) : IRQ Number.
 * Return         : None.
 * Note           : Not API                                                                           */
static void NVIC_disable(uint8_t a_IRQ);


/*===============================================================================
 *                              API Definitions                                  *
 ================================================================================*/

void MCAL_EXTI_GPIO_Init(EXTI_PinConfig_t *p_EXTI_Config)
{
	EXTI_initUpdate(p_EXTI_Config);
}


void MCAL_EXTI_GPIO_DeInit()
{
	/* Reset is Done By Return Every Register to Reset Value */
	EXTI->IMR = 0x00000000;
	EXTI->EMR = 0x00000000;
	EXTI->RTSR = 0x00000000;
	EXTI->FTSR = 0x00000000;
	EXTI->SWIER = 0x00000000;
	EXTI->PR = 0xFFFFFFFF;  /* This Register's bits are cleared by writing a ‘1’ into the bits */

	/* Disable All IRQ From NVCI*/
	NVIC_EXTI0_DI();
	NVIC_EXTI1_DI();
	NVIC_EXTI2_DI();
	NVIC_EXTI3_DI();
	NVIC_EXTI4_DI();
	NVIC_EXTI9_5_DI();
	NVIC_EXTI15_10_DI();
}


void MCAL_EXTI_GPIO_Update(EXTI_PinConfig_t *p_EXTI_Config)
{
	EXTI_initUpdate(p_EXTI_Config);
}


/*===============================================================================
 *                        Private Function Definitions                           *
 ================================================================================*/

static void EXTI_initUpdate(EXTI_PinConfig_t *p_EXTI_Config)
{
	uint8_t AFIO_EXTICR_index;
	uint8_t AFIO_EXTICR_position;
	GPIO_PinConfig_t tempPinConfig;

	/* Setup The GPIO to Be Input Floating */
	tempPinConfig.GPIO_PinNumber = p_EXTI_Config->EXTIx_Pin.GPIO_PinNumber;
	tempPinConfig.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
	MCAL_GPIO_Init(p_EXTI_Config->EXTIx_Pin.GPIOx, &tempPinConfig);


	/* Setup AFIO To Route Between EXTI Line With GPIOx */
	AFIO_EXTICR_index = ((p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber)/4); /* Select the EXRICRx Depending On EXTIx */
	AFIO_EXTICR_position =(((p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber)%4) * 4); /* Get The Position Of EXTIx In the EXRICRx */

	AFIO->EXTICR[AFIO_EXTICR_index] &= (~(0xF << AFIO_EXTICR_position));
	AFIO->EXTICR[AFIO_EXTICR_index] |= (((AFIO_EXTI_GPIOx(p_EXTI_Config->EXTIx_Pin.GPIOx)) & 0xF) << AFIO_EXTICR_position );


	/* Setup The Interrupt Trigger Case if Raising or Falling or Both */
	CLEAR_BIT((EXTI->RTSR),(p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber));
	CLEAR_BIT((EXTI->FTSR),(p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber));

	if(p_EXTI_Config->EXTI_TriggerCase == EXTI_RISING_TRIG)
	{
		/*Set Raising Enable Bit*/
		SET_BIT((EXTI->RTSR),(p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber));

	}else if(p_EXTI_Config->EXTI_TriggerCase == EXTI_FALLING_TRIG)
	{
		/*Set Falling Enable Bit*/
		SET_BIT((EXTI->FTSR),(p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber));
	}else if(p_EXTI_Config->EXTI_TriggerCase == EXTI_RISING_FALLING_TRIG)
	{
		/*Set Both Falling and Raising Bits*/
		SET_BIT((EXTI->RTSR),(p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber));
		SET_BIT((EXTI->FTSR),(p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber));
	}

	/* Enable Or Disable The IRQ in EXTI(MASK) and NVIC */
	if(p_EXTI_Config->EXTI_IRQ == EXTI_IRQ_ENABLE)
	{
		/*Enable Interrupt MASK Bit*/
		SET_BIT((EXTI->IMR),(p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber));
		/* Enable IRQ from EXTI*/
		NVIC_enable(p_EXTI_Config->EXTIx_Pin.IVT_IRQ_Number);

	}else if(p_EXTI_Config->EXTI_IRQ == EXTI_IRQ_DISABLE)
	{
		/*Disable Interrupt MASK Bit*/
		CLEAR_BIT((EXTI->IMR),(p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber));
		/* Disable IRQ from EXTI*/
		NVIC_disable(p_EXTI_Config->EXTIx_Pin.IVT_IRQ_Number);
	}

	/* Set The Interrupt Handling Callback Function  */
	gp_EXTI_ISR_CallBack[p_EXTI_Config->EXTIx_Pin.EXTI_LineNumber] = p_EXTI_Config->p_EXTI_ISR_CallBack;
}


static void NVIC_enable(uint8_t a_IRQ)
{
	switch(a_IRQ)
	{
	case EXTI0_IRQ: NVIC_EXTI0_EN();		break;
	case EXTI1_IRQ: NVIC_EXTI1_EN();		break;
	case EXTI2_IRQ: NVIC_EXTI2_EN();		break;
	case EXTI3_IRQ: NVIC_EXTI3_EN();		break;
	case EXTI4_IRQ: NVIC_EXTI4_EN();		break;
	case EXTI5_IRQ: NVIC_EXTI9_5_EN();		break;
	case EXTI10_IRQ: NVIC_EXTI15_10_EN();	break;
	}
}


static void NVIC_disable(uint8_t a_IRQ)
{
	switch(a_IRQ)
	{
	case EXTI0_IRQ: NVIC_EXTI0_DI();		break;
	case EXTI1_IRQ: NVIC_EXTI1_DI();		break;
	case EXTI2_IRQ: NVIC_EXTI2_DI();		break;
	case EXTI3_IRQ: NVIC_EXTI3_DI();		break;
	case EXTI4_IRQ: NVIC_EXTI4_DI();		break;
	case EXTI5_IRQ: NVIC_EXTI9_5_DI();		break;
	case EXTI10_IRQ: NVIC_EXTI15_10_DI();	break;
	}
}


/*===============================================================================
 *                       		 ISR Functions  		                         *
 ================================================================================*/

void EXTI0_IRQHandler(void)
{
	/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
	SET_BIT(EXTI->PR,EXTI0);
	/* Call The ISR CallBack Function */
	(*gp_EXTI_ISR_CallBack[0])();
}

void EXTI1_IRQHandler(void)
{
	/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
	SET_BIT(EXTI->PR,EXTI1);
	/* Call The ISR CallBack Function */
	(*gp_EXTI_ISR_CallBack[1])();
}

void EXTI2_IRQHandler(void)
{
	/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
	SET_BIT(EXTI->PR,EXTI2);
	/* Call The ISR CallBack Function */
	(*gp_EXTI_ISR_CallBack[2])();
}

void EXTI3_IRQHandler(void)
{
	/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
	SET_BIT(EXTI->PR,EXTI3);
	/* Call The ISR CallBack Function */
	(*gp_EXTI_ISR_CallBack[3])();
}

void EXTI4_IRQHandler(void)
{
	/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
	SET_BIT(EXTI->PR,EXTI4);
	/* Call The ISR CallBack Function */
	(*gp_EXTI_ISR_CallBack[4])();
}

void EXTI9_5_IRQHandler(void)
{
	if(BIT_IS_SET(EXTI->PR,EXTI5))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI5);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[5])();
	}else if(BIT_IS_SET(EXTI->PR,EXTI6))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI6);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[6])();
	}else if(BIT_IS_SET(EXTI->PR,EXTI7))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI7);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[7])();
	}else if(BIT_IS_SET(EXTI->PR,EXTI8))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI8);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[8])();
	}else if(BIT_IS_SET(EXTI->PR,EXTI9))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI9);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[9])();
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(BIT_IS_SET(EXTI->PR,EXTI10))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI10);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[10])();
	}else if(BIT_IS_SET(EXTI->PR,EXTI11))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI11);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[11])();
	}else if(BIT_IS_SET(EXTI->PR,EXTI12))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI12);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[12])();
	}else if(BIT_IS_SET(EXTI->PR,EXTI13))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI13);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[13])();
	}else if(BIT_IS_SET(EXTI->PR,EXTI14))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI14);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[14])();
	}else if(BIT_IS_SET(EXTI->PR,EXTI15))
	{
		/* Clear The Pending Bit By Writing a ‘1’ Into The BiT */
		SET_BIT(EXTI->PR,EXTI15);
		/* Call The ISR CallBack Function */
		(*gp_EXTI_ISR_CallBack[15])();
	}
}

