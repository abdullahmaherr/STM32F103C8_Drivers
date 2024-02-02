/*============================================================================================
 * Module : KEYPAD
 *
 * File Name : keypad.h
 *
 * Author: Abdullah Maher
 *
 * Description : Headder File Of KEYPAD Driver
 *
 * Created on: May 1, 2023
 =============================================================================================*/

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8.h"


/*===============================================================================
 *                            User Type Definitions                              *
 ================================================================================*/

/* Keypad configurations for number of rows and columns */
#define KEYPAD_NUM_COLS                   4
#define KEYPAD_NUM_ROWS                   4 /*COLS Will Be 3 in 3x4*/

/* Keypad Port Configurations */
#define KEYPAD_ROW_PORT            GPIOB
#define KEYPAD_ROW_PIN0       	   GPIO_PIN9
#define KEYPAD_ROW_PIN1       	   GPIO_PIN9
#define KEYPAD_ROW_PIN2       	   GPIO_PIN9
#define KEYPAD_ROW_PIN3       	   GPIO_PIN9

#define KEYPAD_COL_PORT	          GPIOB
#define KEYPAD_COL_PIN0           GPIO_PIN5
#define KEYPAD_COL_PIN1           GPIO_PIN5
#define KEYPAD_COL_PIN2           GPIO_PIN5
#define KEYPAD_COL_PIN3           GPIO_PIN5

/* Keypad button logic configurations */
#define KEYPAD_BUTTON_PRESSED            LOGIC_LOW
#define KEYPAD_BUTTON_RELEASED           LOGIC_HIGH

/*===============================================================================
 *           		    	   	   Generic Macros  		  	                     *
 ================================================================================*/


/*===============================================================================
 *                                	   APIs 		   		                     *
 ================================================================================*/

/**===============================================================================
 * Function Name  : HAL_KEYPAD_GPIO_Init.
 * Brief          : Function To Initial the GPTOx Pins.
 * Parameter (in) : GPIOx.
 * Parameter (in) : .
 * Return         : None.
 * Note           : None																		*/
void HAL_KEYPAD_GPIO_Init(void);

/**===============================================================================
 * Function Name  : HAL_KEYPAD_PressedKey.
 * Brief          : Function To Get The Pressed Key.
 * Parameter (in) : None.
 * Return         : The Pressed Key Location.
 * Note           : None																		*/
uint8_t HAL_KEYPAD_PressedKey(void);


#endif /* INC_KEYPAD_H_ */
