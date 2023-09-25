/*============================================================================================
 * Module : RCC
 *
 * File Name : stm32f103c8_rcc_driver.h
 *
 * Author: Abdullah Maher
 *
 * Description : Header File Of STM32F103C8 RCC Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

#ifndef INC_STM32F103C8_RCC_DRIVER_H_
#define INC_STM32F103C8_RCC_DRIVER_H_


/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/

#include "stm32f103c8.h"


/*===============================================================================
 *                            User Type Definitions                              *
 ================================================================================*/


/*===============================================================================
 *                       Macros Configuration References                         *
 ================================================================================*/


/*===============================================================================
 *           		   Clock/Reset Enable/Disable Macros 	 		             *
 ================================================================================*/

/* AHB Bus Clock Enable*/
#define RCC_DMA_CLK_EN()					(SET_BIT(RCC->AHBENR,0))
#define RCC_SRAM_CLK_EN()					(SET_BIT(RCC->AHBENR,2))

/*APB1 Bus Clock Enable*/
#define RCC_TIM2_CLK_EN()					(SET_BIT(RCC->APB1ENR,0))
#define RCC_TIM3_CLK_EN()					(SET_BIT(RCC->APB1ENR,1))
#define RCC_TIM4_CLK_EN()					(SET_BIT(RCC->APB1ENR,2))
#define RCC_WWD_CLK_EN()					(SET_BIT(RCC->APB1ENR,11))
#define RCC_SPI2_CLK_EN()					(SET_BIT(RCC->APB1ENR,14))
#define RCC_USART2_CLK_EN()					(SET_BIT(RCC->APB1ENR,17))
#define RCC_USART3_CLK_EN()					(SET_BIT(RCC->APB1ENR,18))
#define RCC_I2C1_CLK_EN()					(SET_BIT(RCC->APB1ENR,21))
#define RCC_I2C2_CLK_EN()					(SET_BIT(RCC->APB1ENR,22))
#define RCC_USB_CLK_EN()					(SET_BIT(RCC->APB1ENR,23))
#define RCC_CAN_CLK_EN()					(SET_BIT(RCC->APB1ENR,25))
#define RCC_BKP_CLK_EN()					(SET_BIT(RCC->APB1ENR,27))

/*APB2 Bus Clock Enable*/
#define RCC_AFIO_CLK_EN()					(SET_BIT(RCC->APB2ENR,0))
#define RCC_GPIOA_CLK_EN()					(SET_BIT(RCC->APB2ENR,2))
#define RCC_GPIOB_CLK_EN()					(SET_BIT(RCC->APB2ENR,3))
#define RCC_GPIOC_CLK_EN()					(SET_BIT(RCC->APB2ENR,4))
#define RCC_GPIOD_CLK_EN()					(SET_BIT(RCC->APB2ENR,5))
#define RCC_GPIOE_CLK_EN()					(SET_BIT(RCC->APB2ENR,6))
#define RCC_ADC1_CLK_EN()					(SET_BIT(RCC->APB2ENR,9))
#define RCC_ADC2_CLK_EN()					(SET_BIT(RCC->APB2ENR,10))
#define RCC_TIM1_CLK_EN()					(SET_BIT(RCC->APB2ENR,11))
#define RCC_SPI1_CLK_EN()					(SET_BIT(RCC->APB2ENR,12))
#define RCC_USART1_CLK_EN()					(SET_BIT(RCC->APB2ENR,14))


/*APB1 Bus Reset*/
#define RCC_TIM2_RESET()					(SET_BIT(RCC->APB1RSTR,0))
#define RCC_TIM3_RESET()					(SET_BIT(RCC->APB1RSTR,1))
#define RCC_TIM4_RESET()					(SET_BIT(RCC->APB1RSTR,2))
#define RCC_WWD_RESET()						(SET_BIT(RCC->APB1RSTR,11))
#define RCC_SPI2_RESET()					(SET_BIT(RCC->APB1RSTR,14))
#define RCC_USART2_RESET()					(SET_BIT(RCC->APB1RSTR,17))
#define RCC_USART3_RESET()					(SET_BIT(RCC->APB1RSTR,18))
#define RCC_I2C1_RESET()					(SET_BIT(RCC->APB1RSTR,21))
#define RCC_I2C2_RESET()					(SET_BIT(RCC->APB1RSTR,22))
#define RCC_USB_RESET()						(SET_BIT(RCC->APB1RSTR,23))
#define RCC_CAN_RESET()						(SET_BIT(RCC->APB1RSTR,25))
#define RCC_BKP_RESET()						(SET_BIT(RCC->APB1RSTR,27))

/*APB2 Bus Reset*/
#define RCC_AFIO_RESET()					(SET_BIT(RCC->APB2RSTR,0))
#define RCC_GPIOA_RESET()					(SET_BIT(RCC->APB2RSTR,2))
#define RCC_GPIOB_RESET()					(SET_BIT(RCC->APB2RSTR,3))
#define RCC_GPIOC_RESET()					(SET_BIT(RCC->APB2RSTR,4))
#define RCC_GPIOD_RESET()					(SET_BIT(RCC->APB2RSTR,5))
#define RCC_GPIOE_RESET()					(SET_BIT(RCC->APB2RSTR,6))
#define RCC_ADC1_RESET()					(SET_BIT(RCC->APB2RSTR,9))
#define RCC_ADC2_RESET()					(SET_BIT(RCC->APB2RSTR,10))
#define RCC_TIM1_RESET()					(SET_BIT(RCC->APB2RSTR,11))
#define RCC_SPI1_RESET()					(SET_BIT(RCC->APB2RSTR,12))
#define RCC_USART1_RESET()					(SET_BIT(RCC->APB2RSTR,14))


/*===============================================================================
 *                                	   APIs 		   		                     *
 ================================================================================*/

/**===============================================================================
 * Function Name  : MCAL_RCC_initSysClk.
 * Brief          : Function To Initiate The System Clock.
 * Parameter (in) : .
 * Parameter (in) :.
 * Return         : None.
 * Note           : None.																			*/
void MCAL_RCC_initSysClk();


#endif /* INC_STM32F103C8_RCC_DRIVER_H_ */
