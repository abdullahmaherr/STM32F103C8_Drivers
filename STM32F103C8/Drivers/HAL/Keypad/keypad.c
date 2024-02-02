/*============================================================================================
 * Module : KEYPAD
 *
 * File Name : keypad.c
 *
 * Author: Abdullah Maher
 *
 * Description : Source File Of KEYPAD Driver
 *
 * Created on: May 1, 2023
 =============================================================================================*/
/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "keypad.h"
#include "stm32f103c8_gpio_driver.h"

/*===============================================================================
 *                            		  MACROS	                                 *
 ================================================================================*/

/*===============================================================================
 *                              Global Variables                                 *
 ================================================================================*/

static uint32_t Keypad_COL[4] = {KEYPAD_COL_PIN0,KEYPAD_COL_PIN1,KEYPAD_COL_PIN2,KEYPAD_COL_PIN3};
static uint32_t Keypad_ROW[4] = {KEYPAD_ROW_PIN0,KEYPAD_ROW_PIN1,KEYPAD_ROW_PIN2,KEYPAD_ROW_PIN3};
/*===============================================================================
 *                          Private Function Prototypes	   		                 *
 ================================================================================*/


/*===============================================================================
 *                              API Definitions                                  *
 ================================================================================*/

void HAL_KEYPAD_GPIO_Init(void)
{
	GPIO_PinConfig_t GPIO_tempConfig;

	/* Setup First Pin Of RAWS as Output*/
	GPIO_tempConfig.GPIO_PinNumber = KEYPAD_ROW_PIN0;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_INPUT_PULLDOWN;
	MCAL_GPIO_Init(KEYPAD_ROW_PORT, &GPIO_tempConfig);

	/* Setup 2nd Pin Of RAWS as Output*/
	GPIO_tempConfig.GPIO_PinNumber = KEYPAD_ROW_PIN1;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_INPUT_PULLDOWN;
	MCAL_GPIO_Init(KEYPAD_ROW_PORT, &GPIO_tempConfig);

	/* Setup 3rd Pin Of RAWS as Output*/
	GPIO_tempConfig.GPIO_PinNumber = KEYPAD_ROW_PIN2;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_INPUT_PULLDOWN;
	MCAL_GPIO_Init(KEYPAD_ROW_PORT, &GPIO_tempConfig);

	/* Setup 4th Pin Of RAWS as Output*/
	GPIO_tempConfig.GPIO_PinNumber = KEYPAD_ROW_PIN3;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_INPUT_PULLDOWN;
	MCAL_GPIO_Init(KEYPAD_ROW_PORT, &GPIO_tempConfig);

	/* Setup First Pin Of COLS as Input*/
	GPIO_tempConfig.GPIO_PinNumber = KEYPAD_COL_PIN0;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(KEYPAD_COL_PORT, &GPIO_tempConfig);

	/* Setup 2nd Pin Of COLS as Input*/
	GPIO_tempConfig.GPIO_PinNumber = KEYPAD_COL_PIN1;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(KEYPAD_COL_PORT, &GPIO_tempConfig);

	/* Setup 3rd Pin Of COLS as Input*/
	GPIO_tempConfig.GPIO_PinNumber = KEYPAD_COL_PIN2;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(KEYPAD_COL_PORT, &GPIO_tempConfig);

#ifndef KEYPAD_3x4_CONFIG
	/* Setup 4th Pin Of COLS as Input*/
	GPIO_tempConfig.GPIO_PinNumber = KEYPAD_COL_PIN3;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(KEYPAD_COL_PORT, &GPIO_tempConfig);
#endif
}


uint8_t HAL_KEYPAD_PressedKey(void)
{
	uint8_t row,col;

	for(col = 0; col < 4; col++)
	{
		MCAL_GPIO_WritePin(KEYPAD_COL_PORT, KEYPAD_COL_PIN0, LOGIC_HIGH);
		MCAL_GPIO_WritePin(KEYPAD_COL_PORT, KEYPAD_COL_PIN1, LOGIC_HIGH);
		MCAL_GPIO_WritePin(KEYPAD_COL_PORT, KEYPAD_COL_PIN2, LOGIC_HIGH);
		MCAL_GPIO_WritePin(KEYPAD_COL_PORT, KEYPAD_COL_PIN3, LOGIC_HIGH);

		MCAL_GPIO_WritePin(KEYPAD_COL_PORT,Keypad_COL[col], LOGIC_LOW);

		for(row = 0; row < 4; row++)
		{
			if(MCAL_GPIO_ReadPin(KEYPAD_ROW_PORT, Keypad_ROW[row]) == KEYPAD_BUTTON_PRESSED)
			{
				while(MCAL_GPIO_ReadPin(KEYPAD_ROW_PORT, Keypad_ROW[row]) == KEYPAD_BUTTON_PRESSED);

				switch (col)
				{
				case 0:
					if (row == 0) return '7';
					else if(row == 1) return '4';
					else if(row == 2) return '1';
					else if(row == 3) return '?';
					break;

				case 1:
					if (row == 0) return '8';
					else if(row == 1) return '5';
					else if(row == 2) return '2';
					else if(row == 3) return '0';
					break;

				case 2:
					if (row == 0) return '9';
					else if(row == 1) return '6';
					else if(row == 2) return '3';
					else if(row == 3) return '=';
					break;

				case 3:
					if (row == 0) return '/';
					else if(row == 1) return '*';
					else if(row == 2) return '-';
					else if(row == 3) return '+';
					break;
				}
			}
		}
	}
	return 0;
}
