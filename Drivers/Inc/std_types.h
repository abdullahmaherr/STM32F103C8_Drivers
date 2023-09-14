/*============================================================================================
 * Module : Standard Types
 *
 * File Name : std_types.h
 *
 * Author: Abdullah Maher
 *
 * Description : Standard Types
 *
 * Created on: Apr 30, 2023
 =============================================================================================*/

#ifndef INC_STD_TYPES_H_
#define INC_STD_TYPES_H_

/* Unsigned Integer Data Types */
typedef unsigned char								uint8_t;
typedef unsigned short int							uint16_t;
typedef unsigned long int				 			uint32_t;
typedef unsigned long long int						uint64_t;


/* Signed Integer Data Types */
typedef char										sint8_t;
typedef short int									sint16_t;
typedef long int		 							sint32_t;
typedef long long int								sint64_t;


/* Volatile Unsigned Integer Data Types */
typedef volatile unsigned char						vuint8_t;
typedef volatile unsigned short int					vuint16_t;
typedef volatile unsigned long int		 			vuint32_t;
typedef volatile unsigned long long int				vuint64_t;


/* Volatile Signed Integer Data Types */
typedef volatile char								vsint8_t;
typedef volatile short int							vsint16_t;
typedef volatile long int		 					vsint32_t;
typedef volatile long long int						vsint64_t;


/* Float Data Types */
typedef float										float32;
typedef double										float64;


/* Boolean Data Types */
typedef enum
{
	FALSE,
	TRUE
}bool;


/* Error Data Types for indication */
typedef enum
{
	INVALIDARGUMENT,
	OVERFLOW
}Error;

#define NULL_PTR ((void*)0)


#endif /* INC_STD_TYPES_H_ */
