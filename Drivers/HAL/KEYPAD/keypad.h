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

#ifndef HAL_KEYPAD_KEYPAD_H_
#define HAL_KEYPAD_KEYPAD_H_

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103x6.h"
#include "STM32F103x6_GPIO_driver.h"

/*===============================================================================
 *                            User Type Definitions                              *
 ================================================================================*/

/* Keypad configurations for number of rows and columns */
#define KEYPAD_NUM_COLS                   4
#define KEYPAD_NUM_ROWS                   4

/* Keypad Port Configurations */
#define KEYPAD_ROW_PORT           	   GPIOB
#define KEYPAD_FIRST_ROW_PIN       	   GPIO_PIN9

#define KEYPAD_COL_PORT	               GPIOB
#define KEYPAD_FIRST_COL_PIN           GPIO_PIN5

/* Keypad button logic configurations */
#define KEYPAD_BUTTON_PRESSED            LOGIC_LOW
#define KEYPAD_BUTTON_RELEASED           LOGIC_HIGH


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


#endif /* HAL_KEYPAD_KEYPAD_H_ */
