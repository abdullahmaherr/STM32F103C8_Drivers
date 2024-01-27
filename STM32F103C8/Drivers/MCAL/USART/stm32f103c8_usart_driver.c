/*============================================================================================
 * Module : USART
 *
 * File Name : stm32f103c8_usart_driver.c
 *
 * Author: Abdullah Maher
 *
 * Description : Source File Of STM32F103C8 USART Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8_usart_driver.h"
#include "stm32f103c8_gpio_driver.h"
#include "stm32f103c8_rcc_driver.h"

/*===============================================================================
 *                            		  MACROS	                                 *
 ================================================================================*/

/* =============================== BaudRate Calculation ======================================= */
#define DIV_Fraction(__FRACTION__)						((uint32_t)(((__FRACTION__ % 100) * 16) / 100))
#define DIV_Mantissa(__MANTISSA__)						((uint32_t) (__MANTISSA__ / 100))
#define BAUDRATEDIV(__PCLK__, __BAUDRATE__)				((uint32_t) ( (( 25 * __PCLK__ ) / (4 * __BAUDRATE__ ))) )
#define BRR_REG(__PCLK__,__BAUDRATE__)					( (DIV_Mantissa(BAUDRATEDIV(__PCLK__, __BAUDRATE__)) << 4) | (DIV_Fraction(BAUDRATEDIV(__PCLK__, __BAUDRATE__)) & 0xF) )


/*===============================================================================
 *                              Global Variables                                 *
 ================================================================================*/
/* Array of 3 Global Configuration Structure */
USART_Config_t g_USART_Config[3];

/*===============================================================================
 *                          Private Function Prototypes	   		                 *
 ================================================================================*/

/**===============================================================================
 * Function Name  : MCAL_USART_GPIO_PinConfig.
 * Brief          : Function To Initiate a USARTx GPIO Pins.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Return         : None.
 * Note           : None.																					*/
static void MCAL_USART_GPIO_PinConfig(USART_TypeDef* USARTx);


/*===============================================================================
 *                              API Definitions                                  *
 ================================================================================*/

/**===============================================================================
 * Function Name  : MCAL_GPIO_Init.
 * Brief          : Function To Initiate a USART.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Parameter (in) : Pointer to The Instant Configuration USART.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_Init(USART_TypeDef* USARTx,USART_Config_t* p_USART_Config)
{
	uint32_t PCLK = 0, Temp_BRR = 0, Temp_CR1 = 0, Temp_CR2 = 0, Temp_CR3 = 0;

	if(USART1 == USARTx)
	{
		/* Assign Global Configuration Structure */
		g_USART_Config[0] = *p_USART_Config;

		/* Enable Clock For The Peripheral */
		MCAL_RCC_enableCLK(RCC_APB2_BUS, RCC_USART1_ID);

		/* Get APB2CLK */
		PCLK = MCAL_RCC_GetPCLK2();
	}
	else if(USART2 == USARTx)
	{
		/* Assign Global Configuration Structure */
		g_USART_Config[1] = *p_USART_Config;

		/* Enable Clock For The Peripheral */
		MCAL_RCC_enableCLK(RCC_APB1_BUS, RCC_USART2_ID);

		/* Get APB1CLK */
		PCLK = MCAL_RCC_GetPCLK1();
	}
	else if(USART3 == USARTx)
	{
		/* Assign Global Configuration Structure */
		g_USART_Config[2] = *p_USART_Config;

		/* Enable Clock For The Peripheral */
		MCAL_RCC_enableCLK(RCC_APB1_BUS, RCC_USART3_ID);

		/* Get APB1CLK */
		PCLK = MCAL_RCC_GetPCLK1();
	}

	/* Enable The USART */
	SET_BIT(Temp_CR1,13);

	/* Enable USART TX/RX */
	Temp_CR1 |= (p_USART_Config->USART_Mode);

	/* Configure The USART Word Length */
	Temp_CR1 |= (p_USART_Config->USART_WordLength);

	/* Configure The USART Parity Bit */
	Temp_CR1 |= (p_USART_Config->USART_ParityBit);

	/* Configure The USART Stop Bits */
	Temp_CR2 |= (p_USART_Config->USART_StopBits);

	/* Configure The USART Flow Control */
	Temp_CR3 |= (p_USART_Config->USART_FlowControl);

	/* Configure The USART Baud Rate */
	Temp_BRR = BRR_REG(PCLK,p_USART_Config->USART_BaudRate);
	(USARTx->BRR) = Temp_BRR;

	if(USART_IRQ_DISABLE != p_USART_Config->USART_IRQ)
	{
		if(USART1 == USARTx)
		{
			/* NVIC IRQ Enable  */
			NVIC_USART1_EN();
		}
		else if(USART2 == USARTx)
		{
			/* NVIC IRQ Enable  */
			NVIC_USART2_EN();
		}
		else if(USART3 == USARTx)
		{
			/* NVIC IRQ Enable  */
			NVIC_USART3_EN();
		}
		/* IRQ MASK Enable */
		Temp_CR1 |= (p_USART_Config->USART_IRQ);
	}

	/* Configure The GPIO Pins */
	MCAL_USART_GPIO_PinConfig(USARTx);

	/* Assign Control Registers */
	(USARTx->CR3) = Temp_CR3;
	(USARTx->CR2) = Temp_CR2;
	(USARTx->CR1) = Temp_CR1;
}

/**===============================================================================
 * Function Name  : MCAL_USART_Deinit.
 * Brief          : Function To Reset a USART.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_Deinit(USART_TypeDef* USARTx)
{
	if(USART1 == USARTx)
	{
		/* Disable Clock For The Peripheral */
		MCAL_RCC_disableCLK(RCC_APB2_BUS, RCC_USART1_ID);

		/* NVIC IRQ Disable  */
		NVIC_USART1_DI();
	}
	else if(USART2 == USARTx)
	{
		/* Disable Clock For The Peripheral */
		MCAL_RCC_disableCLK(RCC_APB1_BUS, RCC_USART2_ID);

		/* NVIC IRQ Disable  */
		NVIC_USART2_DI();
	}
	else if(USART3 == USARTx)
	{
		/* Disable Clock For The Peripheral */
		MCAL_RCC_disableCLK(RCC_APB1_BUS, RCC_USART3_ID);

		/* NVIC IRQ Disable  */
		NVIC_USART3_DI();
	}
}

/**===============================================================================
 * Function Name  : MCAL_USART_TransmitData.
 * Brief          : Function To Transmit a Data by Using USART.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_TransmitData(USART_TypeDef* USARTx, uint16_t* p_Buff, USART_Polling_Mechanism a_PollingEn)
{
	if(USART_POLLING_ENABLED == a_PollingEn)
		while(BIT_IS_CLEAR(USARTx->SR,7));

	if(USART1 == USARTx)
	{
		/* Assign Data Register */
		if(USART_Word_Length_8BITS == g_USART_Config[0].USART_WordLength)
			USARTx->DR = ( *p_Buff & (uint8_t)0xFF );
		else if(USART_Word_Length_9BITS == g_USART_Config[0].USART_WordLength)
			USARTx->DR = ( *p_Buff & (uint16_t)0x1FF );
	}
	else if(USART2 == USARTx)
	{
		/* Assign Data Register */
		if(USART_Word_Length_8BITS == g_USART_Config[1].USART_WordLength)
			USARTx->DR = ( *p_Buff & (uint8_t)0xFF );
		else if(USART_Word_Length_9BITS == g_USART_Config[1].USART_WordLength)
			USARTx->DR = ( *p_Buff & (uint16_t)0x1FF );
	}
	else if(USART3 == USARTx)
	{
		/* Assign Data Register */
		if(USART_Word_Length_8BITS == g_USART_Config[2].USART_WordLength)
			USARTx->DR = ( *p_Buff & (uint8_t)0xFF );
		else if(USART_Word_Length_9BITS == g_USART_Config[2].USART_WordLength)
			USARTx->DR = ( *p_Buff & (uint16_t)0x1FF );
	}
}

/**===============================================================================
 * Function Name  : MCAL_USART_ReceiveData.
 * Brief          : Function To Receive a Data by Using USART.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_ReceiveData(USART_TypeDef* USARTx, uint16_t* p_Buff, USART_Polling_Mechanism a_PollingEn)
{
	if(USART_POLLING_ENABLED == a_PollingEn)
		while(BIT_IS_CLEAR(USARTx->SR,5));

	if(USART1 == USARTx)
	{
		/* Read Data Register */
		if(USART_Word_Length_8BITS == g_USART_Config[0].USART_WordLength)
		{
			if(USART_PARITY_BIT_NONE == g_USART_Config[0].USART_ParityBit)
				*p_Buff = ( USARTx->DR & (uint8_t)0xFF );
			else
				*p_Buff = ( USARTx->DR & (uint8_t)0x7F );
		}
		else if(USART_Word_Length_9BITS == g_USART_Config[0].USART_WordLength)
		{
			if(USART_PARITY_BIT_NONE == g_USART_Config[0].USART_ParityBit)
				*p_Buff = USARTx->DR;
			else
				*p_Buff = ( USARTx->DR & (uint8_t)0xFF );
		}
	}
	else if(USART2 == USARTx)
	{
		/* Read Data Register */
		if(USART_Word_Length_8BITS == g_USART_Config[1].USART_WordLength)
		{
			if(USART_PARITY_BIT_NONE == g_USART_Config[1].USART_ParityBit)
				*p_Buff = ( USARTx->DR & (uint8_t)0xFF );
			else
				*p_Buff = ( USARTx->DR & (uint8_t)0x7F );
		}
		else if(USART_Word_Length_9BITS == g_USART_Config[1].USART_WordLength)
		{
			if(USART_PARITY_BIT_NONE == g_USART_Config[1].USART_ParityBit)
				*p_Buff = USARTx->DR;
			else
				*p_Buff = ( USARTx->DR & (uint8_t)0xFF );
		}
	}
	else if(USART3 == USARTx)
	{
		/* Read Data Register */
		if(USART_Word_Length_8BITS == g_USART_Config[2].USART_WordLength)
		{
			if(USART_PARITY_BIT_NONE == g_USART_Config[2].USART_ParityBit)
				*p_Buff = ( USARTx->DR & (uint8_t)0xFF );	/* All 8 Bits are Data */
			else
				*p_Buff = ( USARTx->DR & (uint8_t)0x7F ); 	/* 7 Bits are Data */
		}
		else if(USART_Word_Length_9BITS == g_USART_Config[2].USART_WordLength)
		{
			if(USART_PARITY_BIT_NONE == g_USART_Config[2].USART_ParityBit)
				*p_Buff = USARTx->DR;	/* All Bits are Data */
			else
				*p_Buff = ( USARTx->DR & (uint8_t)0xFF );	/* 8 Bits are Data */
		}
	}
}

/**===============================================================================
 * Function Name  : MCAL_USART_WaitTC.
 * Brief          : Function Waiting Transmission of the Data by USART is Completed.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_WaitTC(USART_TypeDef* USARTx)
{
	/* This bit is set by hardware if the transmission of a frame containing data is complete and if TXE is set
	 * It is cleared by a software sequence (a read from the USART_SR register followed by a write to the USART_DR register).*/
	while(BIT_IS_CLEAR(USARTx->SR,6));
}


/*===============================================================================
 *                        Private Function Definitions                           *
 ================================================================================*/

/**===============================================================================
 * Function Name  : MCAL_USART_GPIO_PinConfig.
 * Brief          : Function To Initiate a USARTx GPIO Pins.
 * Parameter (in) : Instant USARTx Could be 1, 2, and 3.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_USART_GPIO_PinConfig(USART_TypeDef* USARTx)
{
	GPIO_PinConfig_t config;

	if(USART1 == USARTx)
	{
		/* GPIOA PIN9 TX */
		config.GPIO_PinNumber = GPIO_PIN9;
		config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
		config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
		MCAL_GPIO_Init(GPIOA, &config);

		/* GPIOA PIN10 RX */
		config.GPIO_PinNumber = GPIO_PIN10;
		config.GPIO_Mode = GPIO_MODE_INPUT_AF_FLOATING;
		MCAL_GPIO_Init(GPIOA, &config);

		/* Configure Control Flow Pins */
		if((USART_FLOW_CONTROL_CTS == g_USART_Config[0].USART_FlowControl) || (USART_FLOW_CONTROL_CTS_RTS == g_USART_Config[0].USART_FlowControl) )
		{
			/* GPIOA PIN11 CTS */
			config.GPIO_PinNumber = GPIO_PIN11;
			config.GPIO_Mode = GPIO_MODE_INPUT_AF_FLOATING;
			MCAL_GPIO_Init(GPIOA, &config);

		}else if((USART_FLOW_CONTROL_RTS == g_USART_Config[0].USART_FlowControl) || (USART_FLOW_CONTROL_CTS_RTS == g_USART_Config[0].USART_FlowControl))
		{
			/* GPIOA PIN12 RTS */
			config.GPIO_PinNumber = GPIO_PIN12;
			config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
			config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
			MCAL_GPIO_Init(GPIOA, &config);
		}

	}
	else if(USART2 == USARTx)
	{
		/* GPIOA PIN2 TX */
		config.GPIO_PinNumber = GPIO_PIN2;
		config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
		config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
		MCAL_GPIO_Init(GPIOA, &config);

		/* GPIOA PIN3 RX */
		config.GPIO_PinNumber = GPIO_PIN3;
		config.GPIO_Mode = GPIO_MODE_INPUT_AF_FLOATING;
		MCAL_GPIO_Init(GPIOA, &config);

		/* Configure Control Flow Pins */
		if((USART_FLOW_CONTROL_CTS == g_USART_Config[1].USART_FlowControl) || (USART_FLOW_CONTROL_CTS_RTS == g_USART_Config[1].USART_FlowControl) )
		{
			/* GPIOA PIN0 CTS */
			config.GPIO_PinNumber = GPIO_PIN0;
			config.GPIO_Mode = GPIO_MODE_INPUT_AF_FLOATING;
			MCAL_GPIO_Init(GPIOA, &config);

		}else if((USART_FLOW_CONTROL_RTS == g_USART_Config[1].USART_FlowControl) || (USART_FLOW_CONTROL_CTS_RTS == g_USART_Config[1].USART_FlowControl))
		{
			/* GPIOA PIN1 RTS */
			config.GPIO_PinNumber = GPIO_PIN1;
			config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
			config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
			MCAL_GPIO_Init(GPIOA, &config);
		}

	}
	else if(USART3 == USARTx)
	{
		/* GPIOB PIN10 TX */
		config.GPIO_PinNumber = GPIO_PIN10;
		config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
		config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
		MCAL_GPIO_Init(GPIOB, &config);

		/* GPIOB PIN11 RX */
		config.GPIO_PinNumber = GPIO_PIN11;
		config.GPIO_Mode = GPIO_MODE_INPUT_AF_FLOATING;
		MCAL_GPIO_Init(GPIOB, &config);

		/* Configure Control Flow Pins */
		if((USART_FLOW_CONTROL_CTS == g_USART_Config[2].USART_FlowControl) || (USART_FLOW_CONTROL_CTS_RTS == g_USART_Config[2].USART_FlowControl) )
		{
			/* GPIOB PIN13 CTS */
			config.GPIO_PinNumber = GPIO_PIN13;
			config.GPIO_Mode = GPIO_MODE_INPUT_AF_FLOATING;
			MCAL_GPIO_Init(GPIOB, &config);

		}else if((USART_FLOW_CONTROL_RTS == g_USART_Config[2].USART_FlowControl) || (USART_FLOW_CONTROL_CTS_RTS == g_USART_Config[2].USART_FlowControl))
		{
			/* GPIOB PIN14 RTS */
			config.GPIO_PinNumber = GPIO_PIN14;
			config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
			config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
			MCAL_GPIO_Init(GPIOB, &config);
		}

	}
}

/*===============================================================================
 *                       		 ISR Functions  		                         *
 ================================================================================*/

void USART1_IRQHandler(void)
{
	/* Call The ISR CallBack Function */
	if(g_USART_Config[0].p_USART_ISR != NULL)
		(*g_USART_Config[0].p_USART_ISR)();
}

void USART2_IRQHandler(void)
{
	/* Call The ISR CallBack Function */
	if(g_USART_Config[1].p_USART_ISR != NULL)
		(*g_USART_Config[1].p_USART_ISR)();
}


void USART3_IRQHandler(void)
{
	/* Call The ISR CallBack Function */
	if(g_USART_Config[2].p_USART_ISR != NULL)
		(*g_USART_Config[2].p_USART_ISR)();
}

