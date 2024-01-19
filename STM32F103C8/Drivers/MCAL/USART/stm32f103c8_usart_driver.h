/*============================================================================================
 * Module : USART
 *
 * File Name : stm32f103c8_usart_driver.h
 *
 * Author: Abdullah Maher
 *
 * Description : Header File Of STM32F103C8 USART Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

#ifndef INC_STM32F103C8_USART_DRIVER_H_
#define INC_STM32F103C8_USART_DRIVER_H_

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8.h"
#include "stm32f103c8_gpio_driver.h"
#include "stm32f103c8_rcc_driver.h"


/*===============================================================================
 *                            User Type Definitions                              *
 ================================================================================*/
typedef struct{

	uint16_t USART_Mode; 			/* Specifies The TX/RX Enable/Disable Based @ref USART_Mode_define */
	
	uint32_t USART_BaudRate;		/* Specifies The USART BaudRate Regarding to @ref BaudRate_Value_define*/

	uint16_t USART_WordLength;		/* Specifies The USART WordLength Regarding to @ref WordLength_define*/

	uint16_t USART_StopBits;			/* Specifies The USART StopBits Regarding to @ref StopBits_define */

	uint16_t USART_ParityBit;		/* Specifies The USART ParityBit Regarding to @ref ParityBit_define */

	uint16_t USART_FlowControl;		/* Specifies The USART FlowControl Regarding to @ref FlowControl_define*/

	uint16_t USART_IRQ;				/* Specifies The USART Interrupt Mask Regarding to @ref USART_Interrupt_Enable/Disable_define */

	void(*p_USART_ISR)(void);		/* SET The Call Back Function That Will Be Called In ISR */

}USART_Config_t;

/*===============================================================================
 *                       Macros Configuration References                         *
 ================================================================================*/

/* @ref USART_Mode_define */
#define USART_MODE_TX						((uint32_t)0x00000008)
#define USART_MODE_RX						((uint32_t)0x00000004)
#define USART_MODE_TXRX						((uint32_t)0x0000000C)

/* @ref BaudRate_Value_define */
#define USART_BAUD_RATE_2400				2400
#define USART_BAUD_RATE_9600				9600
#define USART_BAUD_RATE_19200				19200
#define USART_BAUD_RATE_57600				57600
#define USART_BAUD_RATE_115200				115200
#define USART_BAUD_RATE_230400				230400
#define USART_BAUD_RATE_460800				460800
#define USART_BAUD_RATE_921600				921600
#define USART_BAUD_RATE_2250000				2250000
#define USART_BAUD_RATE_4500000				4500000

/*  @ref WordLength_define */
#define USART_Word_Length_8BITS				((uint32_t)0x00000000) 	/* 7 Data Bits and 1 Parity Bit */
#define USART_Word_Length_9BITS				((uint32_t)0x00001000)	/* 8 Data Bits and 1 Parity Bit */

/* @ref StopBits_define */
#define USART_STOP_BIT_1					((uint32_t)0x00000000)
#define USART_STOP_BIT_2					((uint32_t)0x00002000)

/* @ref ParityBit_define */
#define USART_PARITY_BIT_NONE				((uint32_t)0x00000000)	/* Disable Parity Bit */
#define USART_PARITY_BIT_EVEN				((uint32_t)0x00000400)	/* Even Parity Bit */
#define USART_PARITY_BIT_ODD				((uint32_t)0x00000600)	/* Odd Parity Bit */

/*  @ref FlowControl_define */
#define USART_FLOW_CONTROL_NONE				((uint32_t)0x00000000)	/* Disable Flow Control */
#define USART_FLOW_CONTROL_CTS				((uint32_t)0x00000200)
#define USART_FLOW_CONTROL_RTS				((uint32_t)0x00000100)
#define USART_FLOW_CONTROL_CTS_RTS			((uint32_t)0x00000300)

/* @ref USART_Interrupt_Enable/Disable_define */
#define USART_IRQ_DISABLE					((uint32_t)0x00000000)	/* Polling Mechanism Enabled */
#define USART_IRQ_ENABLE_TXE				((uint32_t)0x00000080)	/* Transmit Data Register Empty */
#define USART_IRQ_ENABLE_TC					((uint32_t)0x00000040)	/* Transmission Complete */
#define USART_IRQ_ENABLE_RXNE				((uint32_t)0x00000020)	/* Received Data Ready to Be Read, Overrun Error detected */
#define USART_IRQ_ENABLE_PE					((uint32_t)0x00000100)	/* Parity Error */

typedef enum
{
	POLLING_DISABLED, POLLING_ENABLED
}Polling_Mechanism;

/*===============================================================================
 *           		    	   	   Generic Macros  		  	                     *
 ================================================================================*/

/*===============================================================================
 *                                	   APIs 		   		                     *
 ================================================================================*/

/**===============================================================================
 * Function Name  : MCAL_GPIO_Init.
 * Brief          : Function To Initiate a USART.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Parameter (in) : Pointer to The Instant Configuration USART.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_Init(USART_TypeDef* USARTx,USART_Config_t* p_USART_Config);

/**===============================================================================
 * Function Name  : MCAL_USART_Deinit.
 * Brief          : Function To Reset a USART.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_Deinit(USART_TypeDef* USARTx);

/**===============================================================================
 * Function Name  : MCAL_USART_TransmitData.
 * Brief          : Function To Transmit a Data by Using USART.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_TransmitData(USART_TypeDef* USARTx, uint16_t* p_Buff, Polling_Mechanism a_PollingEn);

/**===============================================================================
 * Function Name  : MCAL_USART_ReceiveData.
 * Brief          : Function To Receive a Data by Using USART.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_ReceiveData(USART_TypeDef* USARTx, uint16_t* p_Buff, Polling_Mechanism a_PollingEn);

/**===============================================================================
 * Function Name  : MCAL_USART_ReceiveData.
 * Brief          : Function Waiting Transmission of the Data by USART is Completed.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_WaitTC(USART_TypeDef* USARTx);

#endif /* INC_STM32F103C8_USART_DRIVER_H_ */
