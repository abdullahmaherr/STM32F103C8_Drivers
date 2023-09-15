/*============================================================================================
 * Module : LCD
 *
 * File Name : lcd.h
 *
 * Author: Abdullah Maher
 *
 * Description : Header File Of LCD Driver
 *
 * Created on: May 1, 2023
 =============================================================================================*/

#ifndef HAL_LCD_LCD_H_
#define HAL_LCD_LCD_H_

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103x6.h"
#include "STM32F103x6_GPIO_driver.h"

/*===============================================================================
 *                       User Type Definitions Macros                            *
 ================================================================================*/

/* LCD Data bits mode configuration, its value should be 4 or 8*/
#define LCD_DATA_BITS_MODE 8 /*Write 4 or 8 Bits Mode*/

#if((LCD_DATA_BITS_MODE != 4) && (LCD_DATA_BITS_MODE != 8))

#error "Number of Data bits should be equal to 4 or 8"

#endif


/* LCD HW Ports and Pins IDS */

/* @ref Port Number ID (gpio.h)*/
/* @ref Pin Number ID (gpio.h)*/

#define LCD_RS_PORT         	        GPIOA
#define LCD_RS_PIN      	            GPIO_PIN8

#define LCD_E_PORT  	                GPIOA
#define LCD_E_PIN	                    GPIO_PIN10

#define LCD_DATA_PORT              	 	GPIOA

#if (LCD_DATA_BITS_MODE == 4)

#define LCD_DB4_PIN             	    GPIO_PIN3
#define LCD_DB5_PIN         	        GPIO_PIN4
#define LCD_DB6_PIN     	            GPIO_PIN5
#define LCD_DB7_PIN		                GPIO_PIN6

#elif (LCD_DATA_BITS_MODE == 8)

#define LCD_DB0_PIN             	    GPIO_PIN0
#define LCD_DB1_PIN         	        GPIO_PIN1
#define LCD_DB2_PIN     	            GPIO_PIN2
#define LCD_DB3_PIN		                GPIO_PIN3
#define LCD_DB4_PIN             	    GPIO_PIN4
#define LCD_DB5_PIN         	        GPIO_PIN5
#define LCD_DB6_PIN     	            GPIO_PIN6
#define LCD_DB7_PIN		                GPIO_PIN7

#endif

/* LCD Commands */
#define LCD_CMD_FUNCTION_8BIT_2LINES   					(0x38)
#define LCD_CMD_FUNCTION_4BIT_2LINES   					(0x28)
#define LCD_CMD_MOVE_DISP_RIGHT       					(0x1C)
#define LCD_CMD_MOVE_DISP_LEFT   						(0x18)
#define LCD_CMD_MOVE_CURSOR_RIGHT   					(0x14)
#define LCD_CMD_MOVE_CURSOR_LEFT 	  					(0x10)
#define LCD_CMD_DISP_OFF   								(0x08)
#define LCD_CMD_DISP_ON_CURSOR   						(0x0E)
#define LCD_CMD_DISP_ON_CURSOR_BLINK   					(0x0F)
#define LCD_CMD_DISP_ON_BLINK   						(0x0D)
#define LCD_CMD_DISP_ON   								(0x0C)
#define LCD_CMD_ENTRY_DEC   							(0x04)
#define LCD_CMD_ENTRY_DEC_SHIFT   						(0x05)
#define LCD_CMD_ENTRY_INC_   							(0x06)
#define LCD_CMD_ENTRY_INC_SHIFT   						(0x07)
#define LCD_CMD_BEGIN_AT_FIRST_ROW						(0x80)
#define LCD_CMD_BEGIN_AT_SECOND_ROW						(0xC0)
#define LCD_CMD_CLEAR_SCREEN							(0x01)
#define LCD_CMD_ENTRY_MODE								(0x06)

#define LCD_TWO_LINES_FOUR_BITS_MODE_INIT1   0x33
#define LCD_TWO_LINES_FOUR_BITS_MODE_INIT2   0x32
/*===============================================================================
 *                                	   APIs 		   		                     *
 ================================================================================*/

/**===============================================================================
 * Function Name  : HAL_LCD_Init.
 * Brief          : Function To Initialization LCD.
 * Parameter (in) : None.
 * Return         : None.
 * Note           : None*/
void HAL_LCD_Init(void);

/**===============================================================================
 * Function Name  : HAL_LCD_SendCommand.
 * Brief          : Function To Send Command.
 * Parameter (in) : Command.
 * Return         : None.
 * Note           : None*/
void HAL_LCD_SendCommand(uint8_t a_command);

/**===============================================================================
 * Function Name  : HAL_LCD_DisplayCharacter.
 * Brief          : Function To Display Character on LCD.
 * Parameter (in) : character to display.
 * Return         : None.
 * Note           : None*/
void HAL_LCD_DisplayCharacter(sint8_t a_data);

/**===============================================================================
 * Function Name  : HAL_LCD_DisplayString.
 * Brief          : Function To Display String on LCD.
 * Parameter (in) : Pointer to String to Display.
 * Return         : None.
 * Note           : None*/
void HAL_LCD_DisplayString(const sint8_t * p_str);

/**===============================================================================
 * Function Name  : HAL_LCD_MoveCursor.
 * Brief          : Function To Move The Cursor.
 * Parameter (in) : Row Number.
 * Parameter (in) : Col Number.
 * Return         : None.
 * Note           : None*/
void HAL_LCD_MoveCursor(uint8_t a_row,uint8_t a_col);

/**===============================================================================
 * Function Name  : HAL_LCD_DisplayStringRowCol.
 * Brief          : Function To Display String on LCD in Certain Digit.
 * Parameter (in) : Row Number.
 * Parameter (in) : Col Number.
 * Parameter (in) : Pointer to String to Display.
 * Return         : None.
 * Note           : None*/
void HAL_LCD_DisplayStringRowCol(uint8_t a_row,uint8_t a_col,const sint8_t * p_str);

/**===============================================================================
 * Function Name  : HAL_LCD_ClearScreen.
 * Brief          : Function To Clear Screen.
 * Parameter (in) : None.
 * Return         : None.
 * Note           : None*/
void HAL_LCD_ClearScreen(void);

/**===============================================================================
 * Function Name  : HAL_LCD_IntgerToString.
 * Brief          : Function To Convert Integer To String.
 * Parameter (in) : The Integer Number.
 * Return         : None.
 * Note           : None*/
void HAL_LCD_IntgerToString(sint32_t a_data);

#endif /* HAL_LCD_LCD_H_ */
