/*============================================================================================
 * Module : STM32F103C8
 *
 * File Name : stm32f103c8.h
 *
 * Author: Abdullah Maher
 *
 * Description : Header File Of STM32F103C8
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

#ifndef INC_STM32F103C8_H_
#define INC_STM32F103C8_H_


/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include"stdlib.h"
#include "util.h"
#include"std_types.h"



/*===============================================================================
 *            	    	  Base Addresses For Memories                            *
 ================================================================================*/
#define FLASH_MEMORY_BASE							0X08000000UL
#define SYSTEM_MEMORY_BASE							0X1FFFF000UL
#define SRAM_MEMORY_BASE							0X20000000UL

#define PERIPHERALS_BASE							0X08000000UL

#define CORTEX_M3_INTENRAL_PERIPHERALS_BASE			0X08000000UL

/*===============================================================================
 *            	   	  Base Addresses For Core Peripherals                         *
 ================================================================================*/

/************************************NVIC**************************************/
#define NVIC_BASE									0xE000E100UL

#define NVIC_ISER0						*((vuint32_t*)(NVIC_BASE + 0x00))
#define NVIC_ISER1						*((vuint32_t*)(NVIC_BASE + 0x04))
#define NVIC_ISER2						*((vuint32_t*)(NVIC_BASE + 0x08))
#define NVIC_ICER0						*((vuint32_t*)(NVIC_BASE + 0x80))
#define NVIC_ICER1						*((vuint32_t*)(NVIC_BASE + 0x84))
#define NVIC_ICER2						*((vuint32_t*)(NVIC_BASE + 0x88))


/*===============================================================================
 *            	   	  Base Addresses For MCU Peripherals                         *
 ================================================================================*/

/********************************AHB Bus Peripherals******************************/
/*RCC*/
#define RCC_BASE									0x40021000UL

/********************************APB1 Bus Peripherals******************************/

/*USART*/
#define USART2_BASE									0x40004400UL
#define USART3_BASE									0x40004800UL

/*SPI*/
#define SPI2_BASE									0x40003800UL
/********************************APB2 Bus Peripherals******************************/

/*GPIO*/ /*Note That in LQFP48 GPIOA and GPIOB are fully included, GPIOC and GPIOD Partially Included, GPIOE Not Included*/
#define GPIOA_BASE									0x40010800UL
#define GPIOB_BASE									0x40010C00UL
#define GPIOC_BASE									0x40011000UL
#define GPIOD_BASE									0x40011400UL
#define GPIOE_BASE									0x40011800UL

/*EXTI*/
#define EXTI_BASE									0x40010400UL

/*AFIO*/
#define AFIO_BASE									0x40010000UL

/*USART*/
#define USART1_BASE									0x40013800UL

/*SPI*/
#define SPI1_BASE									0x40013000UL

/*===============================================================================
 *            	   				Peripheral Registers                             *
 ================================================================================*/

/*RCC*/
typedef struct
{
	vuint32_t CR;
	vuint32_t CFGR;
	vuint32_t CIR;
	vuint32_t APB2RSTR;
	vuint32_t APB1RSTR;
	vuint32_t AHBENR;
	vuint32_t APB2ENR;
	vuint32_t APB1ENR;
	vuint32_t BDCR;
	vuint32_t CSR;
}RCC_TypeDef;

/*AFIO*/
typedef struct
{
	vuint32_t EVCR;
	vuint32_t MAPR;
	vuint32_t EXTICR[4];
	uint32_t RESERVED0;
	vuint32_t MAPR2;
}AFIO_TypeDef;

/*GPIO*/
typedef struct
{
	vuint32_t CRL;
	vuint32_t CRH;
	vuint32_t IDR;
	vuint32_t ODR;
	vuint32_t BSRR;
	vuint32_t BRR;
	vuint32_t LCKR;
}GPIO_TypeDef;

/*EXTI*/
typedef struct
{
	vuint32_t IMR;
	vuint32_t EMR;
	vuint32_t RTSR;
	vuint32_t FTSR;
	vuint32_t SWIER;
	vuint32_t PR;
}EXTI_TypeDef;

/*USART*/
typedef struct
{
	vuint32_t SR;
	vuint32_t DR;
	vuint32_t BRR;
	vuint32_t CR1;
	vuint32_t CR2;
	vuint32_t CR3;
	vuint32_t GTPR;
}USART_TypeDef;

/*SPI*/
typedef struct
{
	vuint32_t CR1;
	vuint32_t CR2;
	vuint32_t SR;
	vuint32_t DR;
	vuint32_t CRCPR;
	vuint32_t RXCRCR;
	vuint32_t TXCRCR;
	vuint32_t I2SCFGR;
	vuint32_t I2SPR;
}SPI_TypeDef;

/*===============================================================================
 *            	   				Peripheral Instants                              *
 ================================================================================*/

/*RCC*/
#define RCC							((RCC_TypeDef *)(RCC_BASE))

/*AFIO*/
#define AFIO						((AFIO_TypeDef *)(AFIO_BASE))

/*GPIO*/
#define GPIOA						((GPIO_TypeDef *)(GPIOA_BASE))
#define GPIOB						((GPIO_TypeDef *)(GPIOB_BASE))
#define GPIOC						((GPIO_TypeDef *)(GPIOC_BASE))
#define GPIOD						((GPIO_TypeDef *)(GPIOD_BASE))
#define GPIOE						((GPIO_TypeDef *)(GPIOE_BASE))

/*EXTI*/
#define EXTI						((EXTI_TypeDef *)(EXTI_BASE))

/*USART*/
#define USART1						((USART_TypeDef *)(USART1_BASE))
#define USART2						((USART_TypeDef *)(USART2_BASE))
#define USART3						((USART_TypeDef *)(USART3_BASE))

/*SPI*/
#define SPI1						((SPI_TypeDef *)(SPI1_BASE))
#define SPI2						((SPI_TypeDef *)(SPI2_BASE))

/*===============================================================================
 *           		   		NVIC IRQ Enable/Disable Macros 	 		             *
 ================================================================================*/

/* Enable Interrupt Request */
#define NVIC_EXTI0_EN()					(SET_BIT(NVIC_ISER0,EXTI0_IRQ))
#define NVIC_EXTI1_EN()					(SET_BIT(NVIC_ISER0,EXTI1_IRQ))
#define NVIC_EXTI2_EN()					(SET_BIT(NVIC_ISER0,EXTI2_IRQ))
#define NVIC_EXTI3_EN()					(SET_BIT(NVIC_ISER0,EXTI3_IRQ))
#define NVIC_EXTI4_EN()					(SET_BIT(NVIC_ISER0,EXTI4_IRQ))
#define NVIC_EXTI9_5_EN()				(SET_BIT(NVIC_ISER0,EXTI5_IRQ))
#define NVIC_EXTI15_10_EN()				(SET_BIT(NVIC_ISER1,(EXTI10_IRQ-32)))

#define NVIC_USART1_EN()				(SET_BIT(NVIC_ISER1,(USART1_IRQ-32)))
#define NVIC_USART2_EN()				(SET_BIT(NVIC_ISER1,(USART2_IRQ-32)))
#define NVIC_USART3_EN()				(SET_BIT(NVIC_ISER1,(USART3_IRQ-32)))

#define NVIC_SPI1_EN()					(SET_BIT(NVIC_ISER1,(SPI1_IRQ-32)))
#define NVIC_SPI2_EN()					(SET_BIT(NVIC_ISER1,(SPI2_IRQ-32)))


/* Disable Interrupt Request */
#define NVIC_EXTI0_DI()					(SET_BIT(NVIC_ICER0,EXTI0_IRQ))
#define NVIC_EXTI1_DI()					(SET_BIT(NVIC_ICER0,EXTI1_IRQ))
#define NVIC_EXTI2_DI()					(SET_BIT(NVIC_ICER0,EXTI2_IRQ))
#define NVIC_EXTI3_DI()					(SET_BIT(NVIC_ICER0,EXTI3_IRQ))
#define NVIC_EXTI4_DI()					(SET_BIT(NVIC_ICER0,EXTI4_IRQ))
#define NVIC_EXTI9_5_DI()				(SET_BIT(NVIC_ICER0,EXTI5_IRQ))
#define NVIC_EXTI15_10_DI()				(SET_BIT(NVIC_ICER1,(EXTI10_IRQ-32)))

#define NVIC_USART1_DI()				(SET_BIT(NVIC_ICER1,(USART1_IRQ-32)))
#define NVIC_USART2_DI()				(SET_BIT(NVIC_ICER1,(USART2_IRQ-32)))
#define NVIC_USART3_DI()				(SET_BIT(NVIC_ICER1,(USART3_IRQ-32)))

#define NVIC_SPI1_DI()					(SET_BIT(NVIC_ICER1,(SPI1_IRQ-32)))
#define NVIC_SPI2_DI()					(SET_BIT(NVIC_ICER1,(SPI2_IRQ-32)))

/*===============================================================================
 *           		   	   	   Interrupt Vector Table  		                     *
 ================================================================================*/

/* EXTI */
#define EXTI0_IRQ							6
#define EXTI1_IRQ							7
#define EXTI2_IRQ							8
#define EXTI3_IRQ							9
#define EXTI4_IRQ							10
#define EXTI5_IRQ							23
#define EXTI6_IRQ							23
#define EXTI7_IRQ							23
#define EXTI8_IRQ							23
#define EXTI9_IRQ							23
#define EXTI10_IRQ							40
#define EXTI11_IRQ							40
#define EXTI12_IRQ							40
#define EXTI13_IRQ							40
#define EXTI14_IRQ							40
#define EXTI15_IRQ							40

/*USART*/
#define USART1_IRQ							37
#define USART2_IRQ							38
#define USART3_IRQ							39

/*SPI*/
#define SPI1_IRQ							35
#define SPI2_IRQ							36


#endif /* INC_STM32F103C8_H_ */
