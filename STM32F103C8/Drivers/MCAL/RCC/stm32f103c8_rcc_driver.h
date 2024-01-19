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
/* Configure System Clock Switch */
#define SYS_CLK_SRC					RCC_HSI_CLK    		/* Specifies The Clock Source According to @ref RCC Clock Source */

/* Configure HSE if SYSCLK SRC Is HSE */
#define HSE_CLK_SRC					RCC_HSE_CRYSTAL		/* Specifies The HSE Clock Source According to @ref HSE Clock Source */

/* Configure PLL if SYSCLK SRC Is PLL */
#define PLL_CLK_SRC					RCC_PLLSRC_HSI_DIV2		/* Specifies The PLLSRC Clock Source According to @ref PLLSRC Clock Source */
#define PLLXTPRE_CLK_SRC			RCC_PLLXTPRE_HSE_DIV1	/* Specifies The PLLXTPRE Clock Source According to @ref PLLXTPRE Clock Source */
#define PLL_MULL					RCC_PLLMULL_CLK2	/* Specifies The PLLMULL Value According to @ref PLLMULL Value */
															/* Caution: The PLL Output Frequency Must Not Exceed 72 MHz. */

/* Configure AHB Prescaler */
#define AHB_CLK_PRESCALER			RCC_AHB_SYSCLK_DIV1		/* Specifies The AHB Prescaler According to @ref AHB Prescaler */

/* Configure APB1 Prescaler */
#define APB_LOW_CLK_PRESCALER			RCC_APB_HCLK_DIV1		/* Specifies The APB1 Prescaler According to @ref APB Prescaler */

/* Configure APB2 Prescaler */
#define APB_HIGH_CLK_PRESCALER			RCC_APB_HCLK_DIV1		/* Specifies The APB2 Prescaler According to @ref APB Prescaler */

/*===============================================================================
 *                       Macros Configuration References                         *
 ================================================================================*/


/* @ref RCC Clock Source, System Clock Switch */
#define RCC_HSI_CLK						(0x00000000UL)			/* 8Mhz */
#define RCC_HSE_CLK						(0x00000001UL)			/* 4-16Mhz */
#define RCC_PLL_CLK						(0x00000002UL)

/* @ref HSE Clock Source */
#define RCC_HSE_CRYSTAL						0		/* ByPass Is Disabled */
#define RCC_HSE_RC							1		/* ByPass Is Enabled */

/* @ref PLLSRC Clock Source */
#define	RCC_PLLSRC_HSI_DIV2					0		/* PLLSRC Clock Source Is HSI Divided By 2 */
#define	RCC_PLLSRC_PLLXTPRE					1		/* PLLSRC Clock Source Is PLLXTPRE */

/* @ref PLLXTPRE Clock Source */
#define RCC_PLLXTPRE_HSE_DIV1				0		/* PLLXTPRE Not Divides The HSE */
#define RCC_PLLXTPRE_HSE_DIV2				1		/* PLLXTPRE Divides The HSE By 2 */

/* @ref PLLMULL Value */ /* Caution: The PLL Output Frequency Must Not Exceed 72 MHz. */
#define RCC_PLLMULL_CLK2					((uint32_t)0x00000000)	/* PLLSRC x2 */
#define RCC_PLLMULL_CLK3					((uint32_t)0x00000001)	/* PLLSRC x3 */
#define RCC_PLLMULL_CLK4					((uint32_t)0x00000002)	/* PLLSRC x4 */
#define RCC_PLLMULL_CLK5					((uint32_t)0x00000003)	/* PLLSRC x5 */
#define RCC_PLLMULL_CLK6					((uint32_t)0x00000004)	/* PLLSRC x6 */
#define RCC_PLLMULL_CLK7					((uint32_t)0x00000005)	/* PLLSRC x7 */
#define RCC_PLLMULL_CLK8					((uint32_t)0x00000006)	/* PLLSRC x8 */
#define RCC_PLLMULL_CLK9					((uint32_t)0x00000007)	/* PLLSRC x9 */
#define RCC_PLLMULL_CLK10					((uint32_t)0x00000008)	/* PLLSRC x10 */
#define RCC_PLLMULL_CLK11					((uint32_t)0x00000009)	/* PLLSRC x11 */
#define RCC_PLLMULL_CLK12					((uint32_t)0x0000000A)	/* PLLSRC x12 */
#define RCC_PLLMULL_CLK13					((uint32_t)0x0000000B)	/* PLLSRC x13 */
#define RCC_PLLMULL_CLK14					((uint32_t)0x0000000C)	/* PLLSRC x14 */
#define RCC_PLLMULL_CLK15					((uint32_t)0x0000000D)	/* PLLSRC x15 */
#define RCC_PLLMULL_CLK16					((uint32_t)0x0000000E)	/* PLLSRC x16 */

/* @ref AHB Prescaler */
#define RCC_AHB_SYSCLK_DIV1					((uint32_t)0x00000000)	/* SYSCLK */
#define RCC_AHB_SYSCLK_DIV2					((uint32_t)0x00000080)	/* SYSCLK Divided By 2 */
#define RCC_AHB_SYSCLK_DIV4					((uint32_t)0x00000090)	/* SYSCLK Divided By 4 */
#define RCC_AHB_SYSCLK_DIV8					((uint32_t)0x000000A0)	/* SYSCLK Divided By 8 */
#define RCC_AHB_SYSCLK_DIV16				((uint32_t)0x000000B0)	/* SYSCLK Divided By 16 */
#define RCC_AHB_SYSCLK_DIV64				((uint32_t)0x000000C0)	/* SYSCLK Divided By 64 */
#define RCC_AHB_SYSCLK_DIV128				((uint32_t)0x000000D0)	/* SYSCLK Divided By 128 */
#define RCC_AHB_SYSCLK_DIV256				((uint32_t)0x000000E0)	/* SYSCLK Divided By 256 */
#define RCC_AHB_SYSCLK_DIV512				((uint32_t)0x000000F0)	/* SYSCLK Divided By 512 */

/* @ref APB Prescaler */
#define RCC_APB_HCLK_DIV1					((uint32_t)0x00000000)	/* AHBCLK */
#define RCC_APB_HCLK_DIV2					((uint32_t)0x00000004)	/* AHBCLK Divided By 2 */
#define RCC_APB_HCLK_DIV4					((uint32_t)0x00000005)	/* AHBCLK Divided By 4 */
#define RCC_APB_HCLK_DIV8					((uint32_t)0x00000006)	/* AHBCLK Divided By 8 */
#define RCC_APB_HCLK_DIV16					((uint32_t)0x00000007)	/* AHBCLK Divided By 16 */

/*===============================================================================
 *           		    	   	   Generic Macros  		  	                     *
 ================================================================================*/

/* @ref RCC Clock Value */
#define RCC_HSI_CLK_VAL						((uint32_t)8000000)			/* 8Mhz */
#define RCC_HSE_CLK_VAL						((uint32_t)16000000)		/* 4-16Mhz */
#define RCC_PLL_CLK_VAL						((uint32_t)16000000)


/*RCC Buses IDs*/
#define RCC_AHB_BUS 					0
#define RCC_APB1_BUS 					1
#define RCC_APB2_BUS 					2

/* RCC AHB Bus Peripheral's ID */
#define RCC_DMA1_ID	   			        0
#define RCC_DMA2_ID	          			1
#define RCC_SRAM_ID           			2
#define RCC_FLITF_ID	         		4
#define RCC_CRC_ID		    		    6
#define RCC_FSMC_ID	           			8
#define RCC_SDIO_ID	           			10

/* RCC APB1 Bus Peripheral's ID */
#define RCC_TIM2_ID	   			        0
#define RCC_TIM3_ID	          			1
#define RCC_TIM4_ID	           			2
#define RCC_WWDG_ID	         			11
#define RCC_SPI2_ID		    			14
#define RCC_USART2_ID          			17
#define RCC_USART3_ID          			18
#define RCC_I2C1_ID	           			21
#define RCC_I2C2_ID	           			22
#define RCC_CAN_ID	           			25
#define RCC_PKB_ID	           			27

/* RCC APB2 Bus Peripheral's ID */
#define RCC_AFIO_ID	  			        0
#define RCC_GPIOA_ID           			2
#define RCC_GPIOB_ID           			3
#define RCC_GPIOC_ID           			4
#define RCC_GPIOD_ID           			5
#define RCC_GPIOE_ID           			6
#define RCC_ADC1_ID	           			9
#define RCC_ADC2_ID	           			10
#define RCC_TIM1_ID	           			11
#define RCC_SPI1_ID	           			12
#define RCC_USART1_ID          			14


/*===============================================================================
 *                                	   APIs 		   		                     *
 ================================================================================*/
/**===============================================================================
 * Function Name  : MCAL_RCC_initSYSClk.
 * Brief          : Function To Initialize The SYSCLK Clock.
 * Parameter (in) : None.
 * Return         : None.
 * Note           : None.																			*/
void MCAL_RCC_initSYSClk(void);

/**===============================================================================
 * Function Name  : MCAL_RCC_enableCLK.
 * Brief          : Function To Enable The Clock of Peripheral.
 * Parameter (in) : Bus.
 * Parameter (in) : Peripheral.
 * Return         : None.
 * Note           : None.																			*/
void MCAL_RCC_enableCLK(uint8_t a_BusID, uint8_t a_PeriphID);

/**===============================================================================
 * Function Name  : MCAL_RCC_disableCLK.
 * Brief          : Function To Disable The Clock of Peripheral.
 * Parameter (in) : Bus.
 * Parameter (in) : Peripheral.
 * Return         : None.
 * Note           : None.																			*/
void MCAL_RCC_disableCLK(uint8_t a_BusID, uint8_t a_PeriphID);
/**===============================================================================
 * Function Name  : MCAL_RCC_reset.
 * Brief          : Function To Reset The Peripheral.
 * Parameter (in) : Bus.
 * Parameter (in) : Peripheral.
 * Return         : None.
 * Note           : None.																			*/
void MCAL_RCC_reset(uint8_t a_BusID, uint8_t a_PeriphID);

/**===============================================================================
 * Function Name  : MCAL_RCC_getSYSCLK.
 * Brief          : Function To Get The System Clock.
 * Parameter (in) : None.
 * Return         : System Clock Frequency.
 * Note           : None.																			*/
uint32_t MCAL_RCC_getSYSCLK(void);

/**===============================================================================
 * Function Name  : MCAL_RCC_getHCLK.
 * Brief          : Function To Get The AHB Clock.
 * Parameter (in) : None.
 * Return         : AHB Clock Frequency.
 * Note           : None.																			*/
uint32_t MCAL_RCC_getHCLK(void);

/**===============================================================================
 * Function Name  : MCAL_RCC_GetPCLK1.
 * Brief          : Function To Get The APB1 Clock.
 * Parameter (in) : None.
 * Return         : APB1 Clock Frequency.
 * Note           : None.																			*/
uint32_t MCAL_RCC_GetPCLK1(void);

/**===============================================================================
 * Function Name  : MCAL_RCC_getHCLK.
 * Brief          : Function To Get The APB2 Clock.
 * Parameter (in) : None.
 * Return         : APB2 Clock Frequency.
 * Note           : None.																			*/
uint32_t MCAL_RCC_GetPCLK2(void);

#endif /* INC_STM32F103C8_RCC_DRIVER_H_ */
