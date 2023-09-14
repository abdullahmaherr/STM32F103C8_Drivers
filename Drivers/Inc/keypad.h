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
#include "stm32f103x6.h"
#include "STM32F103x6_GPIO_driver.h"

/*===============================================================================
 *                            User Type Definitions                              *
 ================================================================================*/

typedef struct{
	GPIO_TypeDef * ROW_Port;
	uint8_t ROW_First_Pin;  /* Specifies The GPIOx Pin Number According to @ref GPIO_PIN_NUMBER_DEFINE */

	GPIO_TypeDef * COL_Port;
	uint8_t COL_First_Pin;  /* Specifies The GPIOx Pin Number According to @ref GPIO_PIN_NUMBER_DEFINE */
}keypad_Config_t;


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
void HAL_KEYPAD_GPIO_Init(keypad_Config_t * p_KeypadConfig);

/**===============================================================================
 * Function Name  : HAL_KEYPAD_PressedKey.
 * Brief          : Function To Get The Pressed Key.
 * Parameter (in) : None.
 * Return         : The Pressed Key Location.
 * Note           : None																		*/
uint8_t HAL_KEYPAD_PressedKey(keypad_Config_t * p_KeypadConfig);


#endif /* INC_KEYPAD_H_ */
