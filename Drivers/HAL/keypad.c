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




/*===============================================================================
 *                              API Definitions                                  *
 ================================================================================*/

void HAL_KEYPAD_GPIO_Init(keypad_Config_t * p_KeypadConfig)
{
	GPIO_PinConfig_t GPIO_tempConfig;

	/* Setup First Pin Of RAWS as Output*/
	GPIO_tempConfig.GPIO_PinNumber = p_KeypadConfig->ROW_First_Pin;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(p_KeypadConfig->ROW_Port, &GPIO_tempConfig);

	/* Setup 2nd Pin Of RAWS as Output*/
	GPIO_tempConfig.GPIO_PinNumber = (p_KeypadConfig->ROW_First_Pin)+1;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(p_KeypadConfig->ROW_Port, &GPIO_tempConfig);

	/* Setup 3rd Pin Of RAWS as Output*/
	GPIO_tempConfig.GPIO_PinNumber = (p_KeypadConfig->ROW_First_Pin)+2;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(p_KeypadConfig->ROW_Port, &GPIO_tempConfig);

	/* Setup 4th Pin Of RAWS as Output*/
	GPIO_tempConfig.GPIO_PinNumber = (p_KeypadConfig->ROW_First_Pin)+3;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(p_KeypadConfig->ROW_Port, &GPIO_tempConfig);

	/* Setup First Pin Of COLS as Input*/
	GPIO_tempConfig.GPIO_PinNumber = p_KeypadConfig->COL_First_Pin;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(p_KeypadConfig->COL_Port, &GPIO_tempConfig);

	/* Setup 2nd Pin Of COLS as Input*/
	GPIO_tempConfig.GPIO_PinNumber = (p_KeypadConfig->COL_First_Pin)+1;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(p_KeypadConfig->COL_Port, &GPIO_tempConfig);

	/* Setup 3rd Pin Of COLS as Input*/
	GPIO_tempConfig.GPIO_PinNumber = (p_KeypadConfig->COL_First_Pin)+2;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(p_KeypadConfig->COL_Port, &GPIO_tempConfig);

#ifndef KEYPAD_3x4_CONFIG
	/* Setup 4th Pin Of COLS as Input*/
	GPIO_tempConfig.GPIO_PinNumber = (p_KeypadConfig->COL_First_Pin)+3;
	GPIO_tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	GPIO_tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(p_KeypadConfig->COL_Port, &GPIO_tempConfig);
#endif
}


uint8_t HAL_KEYPAD_PressedKey(keypad_Config_t * p_KeypadConfig)
{
	uint8_t row,col;

	for(col = 0; col < 4; col++)
	{
		MCAL_GPIO_WritePin(p_KeypadConfig->COL_Port, p_KeypadConfig->COL_First_Pin, LOGIC_HIGH);
		MCAL_GPIO_WritePin(p_KeypadConfig->COL_Port, (p_KeypadConfig->COL_First_Pin)+1, LOGIC_HIGH);
		MCAL_GPIO_WritePin(p_KeypadConfig->COL_Port, (p_KeypadConfig->COL_First_Pin)+2, LOGIC_HIGH);
		MCAL_GPIO_WritePin(p_KeypadConfig->COL_Port, (p_KeypadConfig->COL_First_Pin)+3, LOGIC_HIGH);

		MCAL_GPIO_WritePin(p_KeypadConfig->COL_Port,(p_KeypadConfig->COL_First_Pin)+col, LOGIC_LOW);

		for(row = 0; row < 4; row++)
		{
			if(MCAL_GPIO_ReadPin(p_KeypadConfig->ROW_Port, (p_KeypadConfig->ROW_First_Pin)+row) == 0)
			{
				while(MCAL_GPIO_ReadPin(p_KeypadConfig->ROW_Port, (p_KeypadConfig->ROW_First_Pin)+row) == 0);
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
