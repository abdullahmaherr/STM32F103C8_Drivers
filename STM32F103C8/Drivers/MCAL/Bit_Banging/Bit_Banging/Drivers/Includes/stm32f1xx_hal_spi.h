/*============================================================================================
 * Module : SPI
 *
 * File Name : stm32f1xx_hal_spi.h
 *
 * Author: Abdullah Maher
 *
 * Description : Header File Of Bit Banging SPI
 *
 * Created on: Aug 18, 2024
 =============================================================================================*/


#ifndef INCLUDES_STM32F1XX_HAL_SPI_H_
#define INCLUDES_STM32F1XX_HAL_SPI_H_

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8.h"
#include "util.h"
#include "stm32f1xx_hal_gpio_driver.h"


/*===============================================================================
 *                            User Type Definitions                              *
 ================================================================================*/
typedef struct{
	GPIO_TypeDef GPIOx;			/* Specifies The GPIO Pins for SPI */

	uint16_t SPI_SCK_Pin;		/* Specifies The SPI SCK GPIO Pin */

	uint16_t SPI_MOSI_Pin;		/* Specifies The SPI MOSI GPIO Pin */

	uint16_t SPI_MISO_Pin;		/* Specifies The SPI MISO GPIO Pin */

	uint16_t SPI_NSS_Pin;		/* Specifies The SPI NSS GPIO Pin */

}SPI_GPIO_PINS;
typedef struct
{
	uint16_t SPI_Mode;					/* Specifies The SPI Mode Master Or Slave Regarding to @ref SPI Mode*/

	uint8_t SPI_DataSize;				/* Specifies The SPI DataSize 1Byte Or 2Byte Regarding to @ref DataSize */

	SPI_GPIO_PINS SPI_Pins;				/* Specifies The SPI GPIO Pins (The 4 pins will be specified in sequence as SCK, MOSI, MISO, NSS) @ref SPI GPI PINS */

}SPI_Config_t;

/* @ref SPI Mode */
#define SPI_MODE_MASTER										(0x00000004U)
#define SPI_MODE_SLAVE										(0x00000000U)

/* @ref DataSize */
#define SPI_DATA_SIZE_8										(0x08U)	/* 8-Bit Data Frame Format */
#define SPI_DATA_SIZE_16									(0x10U)	/* 16-Bit Data Frame Format */

/* @ref SPI GPI PINS */
#define SPI_GPIOA_PIN1				{GPIOA,GPIO_PIN0,GPIO_PIN1,GPIO_PIN2,GPIO_PIN3}
#define SPI_GPIOA_PIN4				{GPIOA,GPIO_PIN4,GPIO_PIN5,GPIO_PIN6,GPIO_PIN7}
#define SPI_GPIOA_PIN8				{GPIOA,GPIO_PIN8,GPIO_PIN9,GPIO_PIN10,GPIO_PIN11}
#define SPI_GPIOA_PIN12				{GPIOA,GPIO_PIN12,GPIO_PIN13,GPIO_PIN14,GPIO_PIN15}

#define SPI_GPIOB_PIN1				{GPIOB,GPIO_PIN0,GPIO_PIN1,GPIO_PIN2,GPIO_PIN3}
#define SPI_GPIOB_PIN4				{GPIOB,GPIO_PIN4,GPIO_PIN5,GPIO_PIN6,GPIO_PIN7}
#define SPI_GPIOB_PIN8				{GPIOB,GPIO_PIN8,GPIO_PIN9,GPIO_PIN10,GPIO_PIN11}
#define SPI_GPIOB_PIN12				{GPIOB,GPIO_PIN12,GPIO_PIN13,GPIO_PIN14,GPIO_PIN15}

#define SPI_GPIOC_PIN1				{GPIOC,GPIO_PIN0,GPIO_PIN1,GPIO_PIN2,GPIO_PIN3}
#define SPI_GPIOC_PIN4				{GPIOC,GPIO_PIN4,GPIO_PIN5,GPIO_PIN6,GPIO_PIN7}
#define SPI_GPIOC_PIN8				{GPIOC,GPIO_PIN8,GPIO_PIN9,GPIO_PIN10,GPIO_PIN11}
#define SPI_GPIOC_PIN12				{GPIOC,GPIO_PIN12,GPIO_PIN13,GPIO_PIN14,GPIO_PIN15}

#define SPI_GPIOD_PIN1				{GPIOD,GPIO_PIN0,GPIO_PIN1,GPIO_PIN2,GPIO_PIN3}
#define SPI_GPIOD_PIN4				{GPIOD,GPIO_PIN4,GPIO_PIN5,GPIO_PIN6,GPIO_PIN7}
#define SPI_GPIOD_PIN8				{GPIOD,GPIO_PIN8,GPIO_PIN9,GPIO_PIN10,GPIO_PIN11}
#define SPI_GPIOD_PIN12				{GPIOD,GPIO_PIN12,GPIO_PIN13,GPIO_PIN14,GPIO_PIN15}

/*===============================================================================
 *                                	   APIs 		   		                     *
 ================================================================================*/


/**===============================================================================
 * Function Name  : HAL_SPI_TransmitReceive.
 * Brief          : Function To Transmit and Receive a Data in .
 * Parameter (in) : Pointer to The SPI Configuration.
 * Parameter (in) : Pointer to The Data Buffer.
 * Return         : None.
 * Note           : None.																					*/
void HAL_SPI_startTransmitReceive(SPI_Config_t* p_SPI_Config, uint16_t* p_Buffer);

#endif /* INCLUDES_STM32F1XX_HAL_SPI_H_ */
