/*============================================================================================
 * Module : LCD
 *
 * File Name : lcd.c
 *
 * Author: Abdullah Maher
 *
 * Description : Source File Of LCD Driver
 *
 * Created on: May 1, 2023
 =============================================================================================*/

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "lcd.h"
#include "stm32f103c8_gpio_driver.h"
#include "util.h"

/*===============================================================================
 *                      Prototype of Private Function                            *
 ================================================================================*/
/**===============================================================================
 * Function Name  : delay_ms.
 * Brief          : Function To Make Delay.
 * Parameter (in) : delay time.
 * Return         : None.
 * Note           : None																				*/
static void delay_ms(uint32_t delay);

/*===============================================================================
 *                              API Definitions                                  *
 ================================================================================*/

void HAL_LCD_Init(void)
{
	GPIO_PinConfig_t tempConfig;

	/* Configure RS Pin */
	tempConfig.GPIO_PinNumber = LCD_RS_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_RS_PORT, &tempConfig);

	/* Configure E Pin */
	tempConfig.GPIO_PinNumber = LCD_E_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_E_PORT, &tempConfig);

	delay_ms(20);/*Delay for Processing*/

#if (LCD_DATA_BITS_MODE == 4)
	/* Configure 4 Pins in The Data Port as Output Pins */
	tempConfig.GPIO_PinNumber = LCD_DB4_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB5_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB6_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB7_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	HAL_LCD_SendCommand(0x02);
	HAL_LCD_SendCommand(LCD_CMD_FUNCTION_4BIT_2LINES);

#elif (LCD_DATA_BITS_MODE == 8)
	/* Configure 8 Pins in The Data Port as Output Pins */
	tempConfig.GPIO_PinNumber = LCD_DB0_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB1_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB2_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB3_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB4_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB5_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB6_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	tempConfig.GPIO_PinNumber = LCD_DB7_PIN;
	tempConfig.GPIO_Mode = GPIO_MODE_OUTPUT_PUSHPULL;
	tempConfig.GPIO_Output_Speed = GPIO_OUTPUT_SPEED_10M;
	MCAL_GPIO_Init(LCD_DATA_PORT, &tempConfig);

	HAL_LCD_SendCommand(LCD_CMD_FUNCTION_8BIT_2LINES);
#endif
	delay_ms(1);/*Delay for Processing*/
	HAL_LCD_SendCommand(LCD_CMD_ENTRY_MODE);
	delay_ms(1);/*Delay for Processing*/
	HAL_LCD_SendCommand(LCD_CMD_BEGIN_AT_FIRST_ROW);
	delay_ms(1);/*Delay for Processing*/
	HAL_LCD_SendCommand(LCD_CMD_DISP_ON_CURSOR_BLINK);
}


void HAL_LCD_SendCommand(uint8_t a_command)
{
	delay_ms(5);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, LOGIC_LOW);/* Instruction Mode RS=0 */
	delay_ms(1);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_HIGH);/* Enable LCD E=1 */
	delay_ms(1);/*Delay for Processing*/

#if (LCD_DATA_BITS_MODE == 4)
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB4_PIN, GET_BIT(a_command,4));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB5_PIN, GET_BIT(a_command,5));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB6_PIN, GET_BIT(a_command,6));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB7_PIN, GET_BIT(a_command,7));
	delay_ms(1);/*Delay for Processing*/

	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_LOW);/* Disable LCD E=0 */
	delay_ms(1);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_HIGH);/* Enable LCD E=1 */

	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB4_PIN, GET_BIT(a_command,0));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB5_PIN, GET_BIT(a_command,1));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB6_PIN, GET_BIT(a_command,2));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB7_PIN, GET_BIT(a_command,3));

	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_LOW);/* Disable LCD E=0 */
	delay_ms(1);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_HIGH);/* Enable LCD E=1 */

#elif (LCD_DATA_BITS_MODE == 8)
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB0_PIN, GET_BIT(a_command,0));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB1_PIN, GET_BIT(a_command,1));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB2_PIN, GET_BIT(a_command,2));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB3_PIN, GET_BIT(a_command,3));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB4_PIN, GET_BIT(a_command,4));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB5_PIN, GET_BIT(a_command,5));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB6_PIN, GET_BIT(a_command,6));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB7_PIN, GET_BIT(a_command,7));
	delay_ms(1);/*Delay for Processing*/

	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_LOW);/* Disable LCD E=0 */
	delay_ms(1);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_HIGH);/* Enable LCD E=1 */
#endif

}

void HAL_LCD_DisplayCharacter(sint8_t a_data)
{
	delay_ms(5);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, LOGIC_HIGH);/* Data Mode RS=1 */
	delay_ms(1);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_HIGH);/* Enable LCD E=1 */
	delay_ms(1);/*Delay for Processing*/

#if (LCD_DATA_BITS_MODE == 4)
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB4_PIN, GET_BIT(a_data,4));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB5_PIN, GET_BIT(a_data,5));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB6_PIN, GET_BIT(a_data,6));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB7_PIN, GET_BIT(a_data,7));
	delay_ms(1);/*Delay for Processing*/

	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_LOW);/* Disable LCD E=0 */
	delay_ms(1);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_HIGH);/* Enable LCD E=1 */

	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB4_PIN, GET_BIT(a_data,0));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB5_PIN, GET_BIT(a_data,1));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB6_PIN, GET_BIT(a_data,2));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB7_PIN, GET_BIT(a_data,3));

	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_LOW);/* Disable LCD E=0 */
	delay_ms(1);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_HIGH);/* Enable LCD E=1 */

#elif (LCD_DATA_BITS_MODE == 8)
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB0_PIN, GET_BIT(a_data,0));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB1_PIN, GET_BIT(a_data,1));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB2_PIN, GET_BIT(a_data,2));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB3_PIN, GET_BIT(a_data,3));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB4_PIN, GET_BIT(a_data,4));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB5_PIN, GET_BIT(a_data,5));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB6_PIN, GET_BIT(a_data,6));
	MCAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DB7_PIN, GET_BIT(a_data,7));
	delay_ms(1);/*Delay for Processing*/

	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_LOW);/* Disable LCD E=0 */
	delay_ms(1);/*Delay for Processing*/
	MCAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, LOGIC_HIGH);/* Enable LCD E=1 */
#endif

}

void HAL_LCD_DisplayString(const sint8_t * p_str)
{
	uint8_t count;

	while(*p_str != '\0')
	{
		HAL_LCD_DisplayCharacter(*p_str);
		p_str++;
		count++;

		if(count == 16)
		{
			HAL_LCD_MoveCursor(1,0);
		}
		else if(count == 32)
		{
			HAL_LCD_ClearScreen();
			HAL_LCD_MoveCursor(0,0);
			count = 0;
		}
	}
}

void HAL_LCD_MoveCursor(uint8_t a_row,uint8_t a_col)
{
	uint8_t lcd_memory_address;

	/* Calculate the required address in the LCD DDRAM */
	switch(a_row)
	{
	case 0:
		lcd_memory_address=a_col;
		break;
	case 1:
		lcd_memory_address=a_col+0x40;
		break;
	case 2:
		lcd_memory_address=a_col+0x10;
		break;
	case 3:
		lcd_memory_address=a_col+0x50;
		break;
	}

	/* Move the LCD cursor to this specific address */
	HAL_LCD_SendCommand((lcd_memory_address | LCD_CMD_BEGIN_AT_FIRST_ROW));
}

void HAL_LCD_DisplayStringRowCol(uint8_t a_row,uint8_t a_col,const sint8_t * p_str)
{
	HAL_LCD_MoveCursor(a_row, a_col);
	HAL_LCD_DisplayString(p_str);
}

void HAL_LCD_ClearScreen(void)
{
	/* Send clear display command */
	HAL_LCD_SendCommand(LCD_CMD_CLEAR_SCREEN);
}


void HAL_LCD_IntgerToString(sint32_t a_data)
{
	sint8_t buff[16];
	itoa(a_data,buff,10);
	HAL_LCD_DisplayString(buff);
}


static void delay_ms(uint32_t delay)
{
	uint32_t i, j;
	for (i = 0; i < delay; i++)
	{
		for (j = 0; j < 255; j++);
	}
}
