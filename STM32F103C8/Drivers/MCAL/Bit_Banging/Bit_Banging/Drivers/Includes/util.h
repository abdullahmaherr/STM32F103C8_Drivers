/*============================================================================================
 * Module : Utility
 *
 * File Name : util.h
 *
 * Author: Abdullah Maher
 *
 * Description : Utility Macros
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

/* Set a Specific Bit in any Register*/
#define SET_BIT(REG,BIT) ((REG) |= (1<<BIT))

/* Clear a Specific Bit in any Register*/
#define CLEAR_BIT(REG,BIT) ((REG) &= (~(1<<BIT)))

/* Toggle a Specific Bit in any Register*/
#define TOGGLE_BIT(REG,BIT) ((REG) ^= (1<<BIT))

/* Check if a Specific Bit in any Register is Set*/
#define BIT_IS_SET(REG,BIT) ((REG) & (1<<BIT))

/* Check if a Specific Bit in any Register is Cleared*/
#define BIT_IS_CLEAR(REG,BIT) (!((REG) & (1<<BIT)))

/*Get a Specific BIT Value*/
#define GET_BIT(REG,BIT) (((REG) & (1<<BIT)) >> BIT)

/*Write a Specific Value on BIT*/
#define WRI_BIT(REG,BIT,DATA) ((REG) = ((REG) & (~(1<<BIT))) | (DATA<<BIT))

/* Rotate The Register Right By a specific Numbers Of Rotates*/
#define ROR(REG,NUM) ((REG >> NUM) | (REG << (32-NUM)))

/* Rotate The Register Right By a specific Numbers Of Rotates*/
#define ROL(REG,NUM) ((REG << NUM) | (REG >> (32-NUM)))

#endif /* INC_UTIL_H_ */
