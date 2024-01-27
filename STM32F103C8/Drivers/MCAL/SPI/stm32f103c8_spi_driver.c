/*============================================================================================
 * Module : SPI
 *
 * File Name : stm32f103c8_spi_driver.c
 *
 * Author: Abdullah Maher
 *
 * Description : Source File Of STM32F103C8 SPI Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/
/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8_spi_driver.h"
#include "stm32f103c8_gpio_driver.h"
#include "stm32f103c8_rcc_driver.h"


/*===============================================================================
 *                            		  MACROS	                                 *
 ================================================================================*/


/*===============================================================================
 *                              Global Variables                                 *
 ================================================================================*/
SPI_Config_t g_SPI_Config[2];

/*===============================================================================
 *                          Private Function Prototypes	   		                 *
 ================================================================================*/

/**===============================================================================
 * Function Name  : MCAL_SPI_GPIO_PinConfig.
 * Brief          : Function To Initiate a SPIx GPIO Pins.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Return         : None.
 * Note           : None.																					*/
static void MCAL_SPI_GPIO_PinConfig(SPI_TypeDef* SPIx);


/*===============================================================================
 *                              API Definitions                                  *
 ================================================================================*/


/**===============================================================================
 * Function Name  : MCAL_SPI_Init.
 * Brief          : Function To Initiate a SPI.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Parameter (in) : Pointer to The Instant Configuration SPI.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_Init(SPI_TypeDef* SPIx,SPI_Config_t* p_SPI_Config)
{
	uint16_t Temp_CR1 = 0, Temp_CR2 = 0;

	if(SPI1 == SPIx)
	{
		/* Assign Global Configuration Structure */
		g_SPI_Config[0] = *p_SPI_Config;

		/* Enable Clock For The Peripheral */
		MCAL_RCC_enableCLK(RCC_APB2_BUS, RCC_SPI1_ID);


	}else if(SPI2 == SPIx)
	{
		/* Assign Global Configuration Structure */
		g_SPI_Config[1] = *p_SPI_Config;

		/* Enable Clock For The Peripheral */
		MCAL_RCC_enableCLK(RCC_APB1_BUS, RCC_SPI2_ID);

	}

	/* Enable The SPI */
	SET_BIT(Temp_CR1,6);

	/* Sets The SPI in Master or Slave Mode */
	Temp_CR1 |= (p_SPI_Config->SPI_Mode);

	/* Sets The Communication Mode */
	Temp_CR1 |= (p_SPI_Config->SPI_Direction);

	/* Sets The Data Size Frame Format */
	Temp_CR1 |= (p_SPI_Config->SPI_DataSize);

	/* Sets The Data Order MSB Or LSB */
	Temp_CR1 |= (p_SPI_Config->SPI_DataOrder);

	/* Sets The Clock Polarity */
	Temp_CR1 |= (p_SPI_Config->SPI_ClockPolarity);

	/* Sets The Clock Phase */
	Temp_CR1 |= (p_SPI_Config->SPI_ClockPhase);

	/* Sets The BaudRate */
	Temp_CR1 |= (p_SPI_Config->SPI_Prescaler);

	/* Set The NSS Pin Configurations */
	if(SPI_NSS_HW_MASTER_INPUT == p_SPI_Config->SPI_NSS)
	{
		/* HW NSS INPUT */
		Temp_CR2 &= (p_SPI_Config->SPI_NSS);
	}
	else if(SPI_NSS_HW_MASTER_OUTPUT == p_SPI_Config->SPI_NSS)
	{
		/* HW NSS OUTPUT */
		Temp_CR2 |= (p_SPI_Config->SPI_NSS);
	}
	else
	{
		Temp_CR1 |= (p_SPI_Config->SPI_NSS);
	}

	/* Set The IRQ Configurations */
	if(SPI_IRQ_DISABLE != p_SPI_Config->SPI_IRQ)
	{
		/* IRQ MASK Enable */
		Temp_CR2 |= (p_SPI_Config->SPI_IRQ);

		if(SPI1 == SPIx)
		{
			NVIC_SPI1_EN();
		}else if(SPI2 == SPIx)
		{
			NVIC_SPI2_EN();
		}
	}

	/* Configure The GPIO Pins */
	MCAL_SPI_GPIO_PinConfig(SPIx);

	/* Assign Control Registers */
	SPIx->CR2 = Temp_CR2;
	SPIx->CR1 = Temp_CR1;
}

/**===============================================================================
 * Function Name  : MCAL_SPI_Deinit.
 * Brief          : Function To Reset a SPI.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_Deinit(SPI_TypeDef* SPIx)
{
	if(SPI1 == SPIx)
	{
		/* NVIC IRQ Disable  */
		NVIC_SPI1_DI();

		/* Disable Clock For The Peripheral */
		MCAL_RCC_disableCLK(RCC_APB2_BUS, RCC_SPI1_ID);

	}else if(SPI2 == SPIx)
	{
		/* NVIC IRQ Disable  */
		NVIC_SPI2_DI();

		/* Disable Clock For The Peripheral */
		MCAL_RCC_disableCLK(RCC_APB1_BUS, RCC_SPI2_ID);
	}
}

/**===============================================================================
 * Function Name  : MCAL_SPI_Transmit.
 * Brief          : Function To Transmit a Data.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_Transmit(SPI_TypeDef* SPIx, uint16_t* p_Buffer, SPI_Polling_Mechanism a_PollingEn)
{
	/* Wait For Transmission Complete */
	if(SPI_POLLING_ENABLED == a_PollingEn)
		while(BIT_IS_CLEAR(SPIx->SR,1));

	/* Assign Data Register With Data To Be Transmitted */
	SPIx->DR = (*p_Buffer);
}
/**===============================================================================
 * Function Name  : MCAL_SPI_Receive.
 * Brief          : Function To Receive a Data.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_Receive(SPI_TypeDef* SPIx, uint16_t* p_Buffer, SPI_Polling_Mechanism a_PollingEn)
{
	/* Wait For Reception Complete */
	if(SPI_POLLING_ENABLED == a_PollingEn)
		while(BIT_IS_CLEAR(SPIx->SR,0));

	/* Assign The Buffer With Received Data */
	(*p_Buffer) = SPIx->DR;
}
/**===============================================================================
 * Function Name  : MCAL_SPI_TransmitReceive.
 * Brief          : Function To Transmit and Receive a Data in .
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Parameter (in) : Pointer to The Data Buffer.
 * Parameter (in) : Polling Mechanism State Enable/Disable.
 * Return         : None.
 * Note           : None.																					*/
void MCAL_SPI_TransmitReceive(SPI_TypeDef* SPIx, uint16_t* p_Buffer, SPI_Polling_Mechanism a_PollingEn)
{
	/* Wait For Transmission Complete */
	if(SPI_POLLING_ENABLED == a_PollingEn)
		while(BIT_IS_CLEAR(SPIx->SR,1));

	/* Assign Data Register With Data To Be Transmitted */
	SPIx->DR = (*p_Buffer);

	/* Wait For Reception Complete */
	if(SPI_POLLING_ENABLED == a_PollingEn)
		while(BIT_IS_CLEAR(SPIx->SR,0));

	/* Assign The Buffer With Received Data */
	(*p_Buffer) = SPIx->DR;
}


/*===============================================================================
 *                        Private Function Definitions                           *
 ================================================================================*/

/**===============================================================================
 * Function Name  : MCAL_SPI_GPIO_PinConfig.
 * Brief          : Function To Initiate a SPIx GPIO Pins.
 * Parameter (in) : Instant SPIx Could be 1 or 2.
 * Return         : None.
 * Note           : None.																					*/
static void MCAL_SPI_GPIO_PinConfig(SPI_TypeDef* SPIx)
{
	GPIO_PinConfig_t spi_gpio_config;
	if(SPI1 == SPIx)
	{
		if(SPI_MODE_MASTER == g_SPI_Config[0].SPI_Mode)	/* Master */
		{
			/*MOSI1*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN7;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
			spi_gpio_config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
			MCAL_GPIO_Init(GPIOA, &spi_gpio_config);

			/*MISO1*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN6;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
			MCAL_GPIO_Init(GPIOA, &spi_gpio_config);

			/*SCK1*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN5;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
			spi_gpio_config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
			MCAL_GPIO_Init(GPIOA, &spi_gpio_config);

			/*NSS1*/
			if(SPI_NSS_HW_MASTER_INPUT == g_SPI_Config[0].SPI_NSS)
			{
				spi_gpio_config.GPIO_PinNumber = GPIO_PIN4;
				spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
				MCAL_GPIO_Init(GPIOA, &spi_gpio_config);

			}else if(SPI_NSS_HW_MASTER_OUTPUT == g_SPI_Config[0].SPI_Mode)
			{
				spi_gpio_config.GPIO_PinNumber = GPIO_PIN4;
				spi_gpio_config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
				spi_gpio_config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
				MCAL_GPIO_Init(GPIOA, &spi_gpio_config);
			}

		}else	/* Slave */
		{
			/*MOSI1*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN7;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
			MCAL_GPIO_Init(GPIOA, &spi_gpio_config);

			/*MISO1*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN6;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
			spi_gpio_config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
			MCAL_GPIO_Init(GPIOA, &spi_gpio_config);

			/*SCK1*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN5;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
			MCAL_GPIO_Init(GPIOA, &spi_gpio_config);

			/*NSS1*/
			if(SPI_NSS_HW_SLAVE == g_SPI_Config[0].SPI_NSS)
			{
				spi_gpio_config.GPIO_PinNumber = GPIO_PIN4;
				spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
				MCAL_GPIO_Init(GPIOA, &spi_gpio_config);

			}
		}


	}else if(SPI2 == SPIx)
	{
		if(SPI_MODE_MASTER == g_SPI_Config[1].SPI_Mode)	/* Master */
		{
			/*MOSI2*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN15;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
			spi_gpio_config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
			MCAL_GPIO_Init(GPIOB, &spi_gpio_config);

			/*MISO2*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN14;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
			MCAL_GPIO_Init(GPIOB, &spi_gpio_config);

			/*SCK2*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN13;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
			spi_gpio_config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
			MCAL_GPIO_Init(GPIOB, &spi_gpio_config);

			/*NSS2*/
			if(SPI_NSS_HW_MASTER_INPUT == g_SPI_Config[1].SPI_NSS)
			{
				spi_gpio_config.GPIO_PinNumber = GPIO_PIN12;
				spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
				MCAL_GPIO_Init(GPIOB, &spi_gpio_config);

			}else if(SPI_NSS_HW_MASTER_OUTPUT == g_SPI_Config[1].SPI_Mode)
			{
				spi_gpio_config.GPIO_PinNumber = GPIO_PIN12;
				spi_gpio_config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
				spi_gpio_config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
				MCAL_GPIO_Init(GPIOB, &spi_gpio_config);
			}
		}else	/* Slave */
		{
			/*MOSI2*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN15;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
			MCAL_GPIO_Init(GPIOB, &spi_gpio_config);

			/*MISO2*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN14;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PUSHPULL;
			spi_gpio_config.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
			MCAL_GPIO_Init(GPIOB, &spi_gpio_config);

			/*SCK2*/
			spi_gpio_config.GPIO_PinNumber = GPIO_PIN13;
			spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
			MCAL_GPIO_Init(GPIOB, &spi_gpio_config);

			/*NSS2*/
			if(SPI_NSS_HW_SLAVE == g_SPI_Config[1].SPI_NSS)
			{
				spi_gpio_config.GPIO_PinNumber = GPIO_PIN12;
				spi_gpio_config.GPIO_Mode = GPIO_MODE_INPUT_FLOATING;
				MCAL_GPIO_Init(GPIOA, &spi_gpio_config);

			}

		}

	}
}


/*===============================================================================
 *                       		 ISR Functions  		                         *
 ================================================================================*/

void SPI1_IRQHandler(void)
{
	S_SPI_IRQ_SRC IRQ_src;

	/* Read Error Source Flag */
	IRQ_src.RXNE = GET_BIT(SPI1->SR,0);
	IRQ_src.TXE = GET_BIT(SPI1->SR,1);
	IRQ_src.MODF = GET_BIT(SPI1->SR,5);
	IRQ_src.OVR = GET_BIT(SPI1->SR,6);

	/* Call The ISR CallBack Function */
	if(g_SPI_Config[1].p_SPI_ISR != NULL)
		(*g_SPI_Config[0].p_SPI_ISR)(IRQ_src);
}

void SPI2_IRQHandler(void)
{
	S_SPI_IRQ_SRC IRQ_src;

	/* Read Error Source Flag */
	IRQ_src.RXNE = GET_BIT(SPI1->SR,0);
	IRQ_src.TXE = GET_BIT(SPI1->SR,1);
	IRQ_src.MODF = GET_BIT(SPI1->SR,5);
	IRQ_src.OVR = GET_BIT(SPI1->SR,6);

	/* Call The ISR CallBack Function */
	if(g_SPI_Config[1].p_SPI_ISR != NULL)
		(*g_SPI_Config[1].p_SPI_ISR)(IRQ_src);
}
