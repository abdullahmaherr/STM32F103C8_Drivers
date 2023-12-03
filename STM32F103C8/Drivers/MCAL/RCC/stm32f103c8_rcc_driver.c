/*============================================================================================
 * Module : RCC
 *
 * File Name : stm32f103c8_rcc_driver.c
 *
 * Author: Abdullah Maher
 *
 * Description : Source File Of STM32F103C8 RCC Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "util.h"
#include "stm32f103c8_rcc_driver.h"


/*===============================================================================
 *                            		  MACROS	                                 *
 ================================================================================*/

/*===============================================================================
 *                              Global Variables                                 *
 ================================================================================*/


/*===============================================================================
 *                          Private Function Prototypes	   		                 *
 ================================================================================*/



/*===============================================================================
 *                              API Definitions                                  *
 ================================================================================*/
void MCAL_RCC_initSYSClk(void)
{
	/* Clear Default Value For Control Register */
	CLEAR_BIT((RCC->CR),0);

	/* System Clock Switch*/
	RCC->CFGR |= SYS_CLK_SRC;

	/* AHB Prescaler */
	(RCC->CFGR) |= AHB_CLK_PRESCALER;

	/* APB1 Prescaler */
	(RCC->CFGR) |= (APB_LOW_CLK_PRESCALER<<8);

	/* APB2 Prescaler */
	(RCC->CFGR) |= (APB_HIGH_CLK_PRESCALER<<11);

#if SYS_CLK_SRC == RCC_HSI_CLK

	/* Internal High-Speed Clock Enable */
	SET_BIT((RCC->CR),0);

#elif SYS_CLK_SRC == RCC_HSE_CLK

	/* ByPass Enable/Disable Regarding to HSE Clock Source */
	WRI_BIT((RCC->CR),18,HSE_CLK_SRC);

	/* External High-Speed Clock Enable */
	SET_BIT((RCC->CR),16);

	/* Clock Security System Enable For HSE */
	SET_BIT((RCC->CR),19);

#elif SYS_CLK_SRC == RCC_PLL_CLK

	/* Select PLLSRC Clock Source Regarding to PLL_CLK_SRC */
	WRI_BIT((RCC->CFGR),16,PLL_CLK_SRC);

#if PLL_CLK_SRC == RCC_PLLSRC_PLLXTPRE

	/* Select PLLXTPRE Clock Source Regarding to PLLXTPRE_CLK_SRC */
	WRI_BIT((RCC->CFGR),17,PLLXTPRE_CLK_SRC);

	/* ByPass Enable/Disable Regarding to HSE Clock Source */
	WRI_BIT((RCC->CR),18,HSE_CLK_SRC);

	/* External high-speed clock enable */
	SET_BIT((RCC->CR),16);

	/* Clock Security System Enable For HSE*/
	SET_BIT((RCC->CR),19);

#elif PLL_CLK_SRC == RCC_PLLSRC_HSI_DIV2

	/* Internal high-speed clock enable */
	SET_BIT((RCC->CR),0);

#endif

	/* Select PLLMUL Value */
	(RCC->CFGR) |= (PLL_MULL<<18);

	/* PLL Clock Enable */
	SET_BIT((RCC->CR),24);

#else

#error("Your SYSCLK Is False")

#endif

}

void MCAL_RCC_enableCLK(uint8_t a_BusID, uint8_t a_PeriphID)
{
	switch (a_BusID) {
	case RCC_AHB_BUS:
		SET_BIT((RCC->AHBENR),(a_PeriphID));
		break;
	case RCC_APB1_BUS:
		SET_BIT((RCC->APB1ENR),(a_PeriphID));
		break;
	case RCC_APB2_BUS:
		SET_BIT((RCC->APB2ENR),(a_PeriphID));
		break;
	default:
		break;
	}
}

void MCAL_RCC_disableCLK(uint8_t a_BusID, uint8_t a_PeriphID)
{
	switch (a_BusID) {
	case RCC_AHB_BUS:
		CLEAR_BIT((RCC->AHBENR),(a_PeriphID));
		break;
	case RCC_APB1_BUS:
		CLEAR_BIT((RCC->APB1ENR),(a_PeriphID));
		break;
	case RCC_APB2_BUS:
		CLEAR_BIT((RCC->APB2ENR),(a_PeriphID));
		break;
	default:
		break;
	}
}

void MCAL_RCC_reset(uint8_t a_BusID, uint8_t a_PeriphID)
{
	switch (a_BusID) {
	case RCC_APB1_BUS:
		SET_BIT((RCC->APB1RSTR),(a_PeriphID));
		CLEAR_BIT((RCC->APB1RSTR),(a_PeriphID));
		break;
	case RCC_APB2_BUS:
		SET_BIT((RCC->APB2RSTR),(a_PeriphID));
		CLEAR_BIT((RCC->APB2RSTR),(a_PeriphID));
		break;
	default:
		break;
	}
}
