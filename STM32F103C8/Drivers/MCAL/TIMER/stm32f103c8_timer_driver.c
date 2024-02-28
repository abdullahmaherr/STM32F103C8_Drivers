/*============================================================================================
 * Module : TIMER
 *
 * File Name : stm32f103c8_timer_driver.c
 *
 * Author: Abdullah Maher
 *
 * Description : Source File Of STM32F103C8 TIMER Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8_timer_driver.h"
#include "stm32f103c8_rcc_driver.h"
#include "stm32f103c8_gpio_driver.h"

/*===============================================================================
 *                            		  MACROS	                                 *
 ================================================================================*/

/*===============================================================================
 *                              Global Variables                                 *
 ================================================================================*/
static TIM_Config_t g_TIM_Config[4];

enum TimersNumbers
{
	TIMER1,TIMER2,TIMER3,TIMER4
};
/*===============================================================================
 *                          Private Function Prototypes	   		                 *
 ================================================================================*/


/*===============================================================================
 *                              API Definitions                                  *
 ================================================================================*/

/**===============================================================================
 * Function Name  : MCAL_Tim_Init.
 * Brief          : Function To Initialize Timer.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Parameter (in) : Pointer to TIM_Config_t Containing Configuration of The Timer.
 * Return         : None.
 * Note           : None.																				*/
void MCAL_Tim_Init(TIM_TypeDef* TIMx, TIM_Config_t* p_TIM_Config)
{
	/* Assign Control Register With Configuration */
	TIMx->CR1 = (p_TIM_Config->Preload | p_TIM_Config->OnePulseMode | p_TIM_Config->CounterMode);

	/* Assign Period Value */
	TIMx->ARR = p_TIM_Config->Period - 1;

	/* Assign Prescaler Value */
	TIMx->PSC = p_TIM_Config->Prescaler - 1;

	/* Assign Capture/Compare Value */
	TIMx->CCR1 = p_TIM_Config->ChannelCompareValue[TIMER1];
	TIMx->CCR2 = p_TIM_Config->ChannelCompareValue[TIMER2];
	TIMx->CCR3 = p_TIM_Config->ChannelCompareValue[TIMER3];
	TIMx->CCR4 = p_TIM_Config->ChannelCompareValue[TIMER4];

	/* Assign Channels Mode */
	TIMx->CCMR1 = (p_TIM_Config->ChannelMode[CHANNEL1] | (p_TIM_Config->ChannelMode[CHANNEL2] << 8));
	TIMx->CCMR2 = (p_TIM_Config->ChannelMode[CHANNEL3] | (p_TIM_Config->ChannelMode[CHANNEL4] << 8));

	/* Enable Channels */
	TIMx->CCER = p_TIM_Config->ActiveChannels;
	SET_BIT(TIMx->BDTR, 15); /* Main output enable */

	/* Enable/Disable IRQ */
	TIMx->DIER = p_TIM_Config->TIM_IRQ;
	if(TIM_IRQ_DISABLE != p_TIM_Config->TIM_IRQ)
	{
		if(TIM1 == TIMx)
		{
			NVIC_TIM1_EN();

		}else if(TIM2 == TIMx)
		{
			NVIC_TIM2_EN();

		}else if(TIM3 == TIMx)
		{
			NVIC_TIM3_EN();

		}else if(TIM4 == TIMx)
		{
			NVIC_TIM4_EN();
		}
	}

	/* Enable Clock for the Instant */
	if(TIM1 == TIMx)
	{
		/* Assign Global Configuration Struct */
		g_TIM_Config[TIMER1] = *p_TIM_Config;

		/* Assign Repetition Counter Value (Advanced Timers Only)  */
		TIMx->RCR = p_TIM_Config->RepetitionCounter - 1;

		/* Enable Clock */
		MCAL_RCC_enableCLK(RCC_APB2_BUS, RCC_TIM1_ID);

	}else if(TIM2 == TIMx)
	{
		/* Assign Global Configuration Struct */
		g_TIM_Config[TIMER2] = *p_TIM_Config;

		/* Enable Clock */
		MCAL_RCC_enableCLK(RCC_APB1_BUS, RCC_TIM2_ID);

	}else if(TIM3 == TIMx)
	{
		/* Assign Global Configuration Struct */
		g_TIM_Config[TIMER3] = *p_TIM_Config;

		/* Enable Clock */
		MCAL_RCC_enableCLK(RCC_APB1_BUS, RCC_TIM3_ID);

	}else if(TIM4 == TIMx)
	{
		/* Assign Global Configuration Struct */
		g_TIM_Config[TIMER4] = *p_TIM_Config;

		/* Enable Clock */
		MCAL_RCC_enableCLK(RCC_APB1_BUS, RCC_TIM4_ID);
	}
}

/**===============================================================================
 * Function Name  : MCAL_Tim_Init.
 * Brief          : Function To Reset Timer.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Return         : None.
 * Note           : None.																				*/
void MCAL_Tim_Deinit(TIM_TypeDef* TIMx)
{
	if(TIM1 == TIMx)
	{
		/* Disable Clock */
		MCAL_RCC_disableCLK(RCC_APB2_BUS, RCC_TIM1_ID);

		NVIC_TIM1_DI();

	}else if(TIM2 == TIMx)
	{
		/* Disable Clock */
		MCAL_RCC_disableCLK(RCC_APB1_BUS, RCC_TIM2_ID);

		NVIC_TIM2_DI();

	}else if(TIM3 == TIMx)
	{
		/* Disable Clock */
		MCAL_RCC_disableCLK(RCC_APB1_BUS, RCC_TIM3_ID);

		NVIC_TIM3_DI();

	}else if(TIM4 == TIMx)
	{
		/* Disable Clock */
		MCAL_RCC_disableCLK(RCC_APB1_BUS, RCC_TIM4_ID);

		NVIC_TIM4_DI();
	}
}

/**===============================================================================
 * Function Name  : MCAL_Tim_Start.
 * Brief          : Function To Start Timer Counting.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Return         : None.
 * Note           : None.																				*/
void MCAL_Tim_Start(TIM_TypeDef* TIMx)
{
	/* Start Counter */
	if(BIT_IS_CLEAR(TIMx->CR1,0))
	{
		SET_BIT(TIMx->CR1,0);
	}
}

/**===============================================================================
 * Function Name  : MCAL_Tim_Stop.
 * Brief          : Function To Stop Timer Counting.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Return         : None.
 * Note           : None.																				*/
void MCAL_Tim_Stop(TIM_TypeDef* TIMx)
{
	/* Stop Counter */
	if(BIT_IS_SET(TIMx->CR1,0))
	{
		CLEAR_BIT(TIMx->CR1,0);
	}
}

/**===============================================================================
 * Function Name  : MCAL_Tim_PWM.
 * Brief          : Function To Generate PWM.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Return         : None.
 * Note           : None.																				*/



/*===============================================================================
 *                       		 ISR Functions  		                         *
 ================================================================================*/

void TIM1_UP_IRQHandler(void)
{
	/* Call The ISR CallBack Function */
	if(g_TIM_Config[TIMER1].p_TIM_ISR != NULL)
		(*g_TIM_Config[TIMER1].p_TIM_ISR)();
}

void TIM1_CC_IRQHandler(void)
{
	/* Call The ISR CallBack Function */
	if(g_TIM_Config[TIMER1].p_TIM_ISR != NULL)
		(*g_TIM_Config[TIMER1].p_TIM_ISR)();
}

void TIM2_IRQHandler(void)
{
	/* Call The ISR CallBack Function */
	if(g_TIM_Config[TIMER2].p_TIM_ISR != NULL)
		(*g_TIM_Config[TIMER2].p_TIM_ISR)();
}

void TIM3_IRQHandler(void)
{
	/* Call The ISR CallBack Function */
	if(g_TIM_Config[TIMER3].p_TIM_ISR != NULL)
		(*g_TIM_Config[TIMER3].p_TIM_ISR)();
}

void TIM4_IRQHandler(void)
{
	/* Call The ISR CallBack Function */
	if(g_TIM_Config[TIMER4].p_TIM_ISR != NULL)
		(*g_TIM_Config[TIMER4].p_TIM_ISR)();
}
