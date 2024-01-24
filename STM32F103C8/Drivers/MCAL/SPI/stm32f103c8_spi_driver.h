/*============================================================================================
 * Module : SPI
 *
 * File Name : stm32f103c8_spi_driver.h
 *
 * Author: Abdullah Maher
 *
 * Description : Header File Of STM32F103C8 SPI Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

#ifndef INC_STM32F103C8_SPI_DRIVER_H_
#define INC_STM32F103C8_SPI_DRIVER_H_

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8.h"

/*===============================================================================
 *           		    	   	   Generic Macros  		  	                     *
 ================================================================================*/
typedef enum
{
	SPI_POLLING_DISABLED, SPI_POLLING_ENABLED
}SPI_Polling_Mechanism;

typedef struct
{
	uint8_t	RXNE:1;
	uint8_t	TXE:1;
	uint8_t	RESERVED:3;
	uint8_t	MODF:1;
	uint8_t	OVR:1;

}S_SPI_IRQ_SRC;

/*===============================================================================
 *                            User Type Definitions                              *
 ================================================================================*/
typedef struct
{
	uint16_t SPI_Mode;					/* Specifies The SPI Mode Master Or Slave Regarding to @ref SPI Mode*/

	uint16_t SPI_Direction;				/* Specifies The SPI Communication Direction Mode Regarding to @ref Communication Direction Mode*/

	uint16_t SPI_DataOrder;				/* Specifies The SPI DataOrder data transfers start MSB Or LSB Regarding to @ref DataOrder */

	uint16_t SPI_DataSize;				/* Specifies The SPI DataSize 1Byte Or 2Byte Regarding to @ref DataSize */

	uint16_t SPI_ClockPolarity;			/* Specifies The SPI ClockPolarity Regarding to @ref ClockPolarity */

	uint16_t SPI_ClockPhase;			/* Specifies The SPI ClockPhase Regarding to @ref ClockPhase */

	uint16_t SPI_Prescaler;				/* Specifies The SPI Prescaler Regarding to @ref Prescaler */

	uint16_t SPI_NSS;					/* Specifies The SPI NSS Signal is Managed by HW or SW @ref SlaveSelectManagement */

	uint16_t SPI_IRQ;					/* Specifies The SPI Interrupt Mask Regarding to  @ref SPI Interrupt Enable/Disable */

	void(*p_SPI_ISR)(S_SPI_IRQ_SRC IRQ_src);				/* SET The Call Back Function That Will Be Called In ISR */

}SPI_Config_t;

/*===============================================================================
 *                       Macros Configuration References                         *
 ================================================================================*/
			/* @ref SPI Mode */
#define SPI_MODE_MASTER										(0x00000004UL)
#define SPI_MODE_SLAVE										(0x00000000UL)

			/* @ref Communication Direction Mode*/
#define SPI_DIRECTION_TWO_LINE								(0x00000000UL)	/* Full-Duplex 4-Wire */
#define SPI_DIRECTION_TWO_LINE_RXONLY						(0x00000400UL)	/* Half-Duplex 4-Wire */

#define SPI_DIRECTION_ONE_LINE_TXONLY						(0x0000C000UL)	/* Half-Duplex 3-Wire */
#define SPI_DIRECTION_ONE_LINE_RXONLY						(0x00008000UL)	/* Half-Duplex 3-Wire */

			/* @ref DataOrder */
#define SPI_DATA_ORDER_LSB									(0x00000080UL)	/* LSB Transmitted First */
#define SPI_DATA_ORDER_MSB									(0x00000000UL)	/* MSB Transmitted First */

			/* @ref DataSize */
#define SPI_DATA_SIZE_8_BIT									(0x00000000UL)	/* 8-Bit Data Frame Format */
#define SPI_DATA_SIZE_16_BIT								(0x00000800UL)	/* 16-Bit Data Frame Format */

			/* @ref ClockPolarity */
#define SPI_CLK_POLARITY_HIGH								(0x00000002UL)	/* CLK Idle Is High */
#define SPI_CLK_POLARITY_LOW								(0x00000000UL)	/* CLK Idle Is Low */

			/* @ref ClockPhase */
#define SPI_CLK_PHASE_LEAD									(0x00000000UL)	/* The First Clock Transition is the First Data Capture Edge */
#define SPI_CLK_PHASE_LAST									(0x00000002UL)	/* The Second Clock Transition is the First Data Capture Edge */

			/* @ref Prescaler */
#define SPI_CLK_PRESCALER_2									(0x00000000UL)
#define SPI_CLK_PRESCALER_4									(0x00000008UL)
#define SPI_CLK_PRESCALER_8									(0x00000010UL)
#define SPI_CLK_PRESCALER_16								(0x00000018UL)
#define SPI_CLK_PRESCALER_32								(0x00000020UL)
#define SPI_CLK_PRESCALER_64								(0x00000028UL)
#define SPI_CLK_PRESCALER_128								(0x00000030UL)
#define SPI_CLK_PRESCALER_256								(0x00000038UL)

			/* @ref SlaveSelectManagement */
/* SW Mode The SS signal is driven by the firmware and any free GPIO can be used to drive The Slave */
#define SPI_NSS_SW_SSI_SET									(0x00000300UL)
#define SPI_NSS_SW_SSI_RESET								(0x00000200UL)
/* HW Mode a specific MCU I/O is used to drive the SS signal, and it is internally managed by the SPI peripheral */
#define SPI_NSS_HW_SLAVE									(0x00000000UL)
#define SPI_NSS_HW_MASTER_OUTPUT							(0x00000004UL)	/* used only when the device operates in master mode, does not allow multi-master mode*/
#define SPI_NSS_HW_MASTER_INPUT								~(0x00000004UL)	/* this configuration allows multi-master capability for devices operating in master mode */

			/* @ref SPI Interrupt Enable/Disable */
#define SPI_IRQ_DISABLE										(0x00000000UL)
#define SPI_IRQ_ENABLE_TXEIE								(0x00000080UL)
#define SPI_IRQ_ENABLE_RXNEIE								(0x00000040UL)
#define SPI_IRQ_ENABLE_ERRIE								(0x00000020UL)

/*===============================================================================
 *                                	   APIs 		   		                     *
 ================================================================================*/

/**===============================================================================
 * Function Name  : MCAL_SPI_Init.
 * Brief          : Function To Initiate a SPI.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Parameter (in) : Pointer to The Instant Configuration SPI.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_Init(SPI_TypeDef* SPIx,SPI_Config_t* p_SPI_Config);

/**===============================================================================
 * Function Name  : MCAL_SPI_Deinit.
 * Brief          : Function To Reset a SPI.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_Deinit(SPI_TypeDef* SPIx);

/**===============================================================================
 * Function Name  : MCAL_SPI_Transmit.
 * Brief          : Function To Transmit a Data.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_Transmit(SPI_TypeDef* SPIx, uint16_t* p_Buffer, SPI_Polling_Mechanism a_PollingEn);
/**===============================================================================
 * Function Name  : MCAL_SPI_Receive.
 * Brief          : Function To Receive a Data.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_Receive(SPI_TypeDef* SPIx, uint16_t* p_Buffer, SPI_Polling_Mechanism a_PollingEn);
/**===============================================================================
 * Function Name  : MCAL_SPI_TransmitReceive.
 * Brief          : Function To Transmit and Receive a Data in .
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_TransmitReceive(SPI_TypeDef* SPIx, uint16_t* p_Buffer, SPI_Polling_Mechanism a_PollingEn);

#endif /* INC_STM32F103C8_SPI_DRIVER_H_ */
