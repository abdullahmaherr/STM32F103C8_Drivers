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

const uint8_t APBPrescaler[8U] = {0,0,0,0,1,2,3,4}; /* For Every Shift Right Multiply by 2 */

const uint8_t AHBPrescaler[16U] = {0,0,0,0,0,0,0,0,1,2,3,4,6,7,8,9}; /* For Every Shift Right Multiply by 2 */

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

uint32_t MCAL_RCC_getSYSCLK(void)
{
	switch (((RCC->CFGR >> 2) & 0b11))/* Read System clock switch status */
	{
	case RCC_HSI_CLK:
		return RCC_HSI_CLK_VAL; /* Return HSI Clock Frequency Value */
		break;
	case RCC_HSE_CLK:
		return RCC_HSE_CLK_VAL;/* To Do */ /* Return HSE Clock Frequency Value */
		break;
	case RCC_PLL_CLK:
		return RCC_PLL_CLK_VAL;/* To Do */  /* Return PLL Clock Frequency Value */
		break;
	default:
		return 0;
		break;
	}
}

uint32_t MCAL_RCC_getHCLK(void)
{
	/* Divide System Clock Frequency by AHB Prescaler Value */
	return(MCAL_RCC_getSYSCLK() >> AHBPrescaler[((RCC->CFGR >> 4) & 0b1111)]);
}

uint32_t MCAL_RCC_GetPCLK1(void)
{
	/* Divide AHB Clock Frequency by APB1 Prescaler Value */
	return(MCAL_RCC_getHCLK() >> APBPrescaler[((RCC->CFGR >> 8) & 0b111)]);
}

uint32_t MCAL_RCC_GetPCLK2(void)
{
	/* Divide AHB Clock Frequency by APB2 Prescaler Value */
	return(MCAL_RCC_getHCLK() >> APBPrescaler[((RCC->CFGR >> 11) & 0b111)]);
}
