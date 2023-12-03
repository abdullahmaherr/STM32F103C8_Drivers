/*============================================================================================
 * Module : EXTI
 *
 * File Name : STM32F103c8_exti_driver.h
 *
 * Author: Abdullah Maher
 *
 * Description : Header File Of STM32F103C8 EXTI Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

#ifndef INC_STM32F103C8_EXTI_DRIVER_H_
#define INC_STM32F103C8_EXTI_DRIVER_H_

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8.h"



/*===============================================================================
 *                            User Type Definitions                              *
 ================================================================================*/
typedef struct /* Static Configuration in EXTI_PinConfig_t To Limit Configuration Error */
{
	uint8_t EXTI_LineNumber; /* The EXTI Line Number */

	GPIO_TypeDef * GPIOx; 	/* The GPIO Port Used In EXTI */

	uint8_t GPIO_PinNumber; /* The GPIO PIN Number Used In EXTI */

	uint8_t IVT_IRQ_Number; /* The Interrupt Handler According to IVT */

}EXTI_GPIO_MAPPING_t;


typedef struct
{
	EXTI_GPIO_MAPPING_t EXTIx_Pin; 	  /* Specifies The Required EXTI According to @ref EXTI_DEFINE */

	uint8_t EXTI_TriggerCase;		  /* Specifies The Trigger Case According to @ref EXTI_TRIGGER_SELECT_DEFINE */

	uint8_t EXTI_IRQ; 				  /* Specifies ENABLE/DISABLE interrupt According to @ref EXTI_IRQ_DEFINE
										 ENABLE/DISABLE EXTI Interrupt Mask Register (EXTI_IMR) also The NVIC */

	void(*p_EXTI_ISR_CallBack)(void); /* SET The C Function Which will Be Called If IRQ Enabled*/

}EXTI_PinConfig_t;


/*===============================================================================
 *                       Macros Configuration References                         *
 ================================================================================*/




/* @ref EXTI_DEFINE */

/* EXTI0 */
#define EXTI0PA0			(EXTI_GPIO_MAPPING_t){EXTI0,GPIOA,GPIO_PIN0,EXTI0_IRQ}
#define EXTI0PB0			(EXTI_GPIO_MAPPING_t){EXTI0,GPIOB,GPIO_PIN0,EXTI0_IRQ}
#define EXTI0PC0			(EXTI_GPIO_MAPPING_t){EXTI0,GPIOC,GPIO_PIN0,EXTI0_IRQ}
#define EXTI0PD0			(EXTI_GPIO_MAPPING_t){EXTI0,GPIOD,GPIO_PIN0,EXTI0_IRQ}


/* EXTI1 */
#define EXTI1PA1			(EXTI_GPIO_MAPPING_t){EXTI1,GPIOA,GPIO_PIN1,EXTI1_IRQ}
#define EXTI1PB1			(EXTI_GPIO_MAPPING_t){EXTI1,GPIOB,GPIO_PIN1,EXTI1_IRQ}
#define EXTI1PC1			(EXTI_GPIO_MAPPING_t){EXTI1,GPIOC,GPIO_PIN1,EXTI1_IRQ}
#define EXTI1PD1			(EXTI_GPIO_MAPPING_t){EXTI1,GPIOD,GPIO_PIN1,EXTI1_IRQ}


/* EXTI2 */
#define EXTI2PA2			(EXTI_GPIO_MAPPING_t){EXTI2,GPIOA,GPIO_PIN2,EXTI2_IRQ}
#define EXTI2PB2			(EXTI_GPIO_MAPPING_t){EXTI2,GPIOB,GPIO_PIN2,EXTI2_IRQ}
#define EXTI2PC2			(EXTI_GPIO_MAPPING_t){EXTI2,GPIOC,GPIO_PIN2,EXTI2_IRQ}
#define EXTI2PD2			(EXTI_GPIO_MAPPING_t){EXTI2,GPIOD,GPIO_PIN2,EXTI2_IRQ}


/* EXTI3 */
#define EXTI3PA3			(EXTI_GPIO_MAPPING_t){EXTI3,GPIOA,GPIO_PIN3,EXTI3_IRQ}
#define EXTI3PB3			(EXTI_GPIO_MAPPING_t){EXTI3,GPIOB,GPIO_PIN3,EXTI3_IRQ}
#define EXTI3PC3			(EXTI_GPIO_MAPPING_t){EXTI3,GPIOC,GPIO_PIN3,EXTI3_IRQ}
#define EXTI3PD3			(EXTI_GPIO_MAPPING_t){EXTI3,GPIOD,GPIO_PIN3,EXTI3_IRQ}


/* EXTI4 */
#define EXTI4PA4			(EXTI_GPIO_MAPPING_t){EXTI4,GPIOA,GPIO_PIN4,EXTI4_IRQ}
#define EXTI4PB4			(EXTI_GPIO_MAPPING_t){EXTI4,GPIOB,GPIO_PIN4,EXTI4_IRQ}
#define EXTI4PC4			(EXTI_GPIO_MAPPING_t){EXTI4,GPIOC,GPIO_PIN4,EXTI4_IRQ}
#define EXTI4PD4			(EXTI_GPIO_MAPPING_t){EXTI4,GPIOD,GPIO_PIN4,EXTI4_IRQ}


/* EXTI5 */
#define EXTI5PA5			(EXTI_GPIO_MAPPING_t){EXTI5,GPIOA,GPIO_PIN5,EXTI5_IRQ}
#define EXTI5PB5			(EXTI_GPIO_MAPPING_t){EXTI5,GPIOB,GPIO_PIN5,EXTI5_IRQ}
#define EXTI5PC5			(EXTI_GPIO_MAPPING_t){EXTI5,GPIOC,GPIO_PIN5,EXTI5_IRQ}
#define EXTI5PD5			(EXTI_GPIO_MAPPING_t){EXTI5,GPIOD,GPIO_PIN5,EXTI5_IRQ}


/* EXTI6 */
#define EXTI6PA6			(EXTI_GPIO_MAPPING_t){EXTI6,GPIOA,GPIO_PIN6,EXTI6_IRQ}
#define EXTI6PB6			(EXTI_GPIO_MAPPING_t){EXTI6,GPIOB,GPIO_PIN6,EXTI6_IRQ}
#define EXTI6PC6			(EXTI_GPIO_MAPPING_t){EXTI6,GPIOC,GPIO_PIN6,EXTI6_IRQ}
#define EXTI6PD6			(EXTI_GPIO_MAPPING_t){EXTI6,GPIOD,GPIO_PIN6,EXTI6_IRQ}


/* EXTI7 */
#define EXTI7PA7			(EXTI_GPIO_MAPPING_t){EXTI7,GPIOA,GPIO_PIN7,EXTI7_IRQ}
#define EXTI7PB7			(EXTI_GPIO_MAPPING_t){EXTI7,GPIOB,GPIO_PIN7,EXTI7_IRQ}
#define EXTI7PC7			(EXTI_GPIO_MAPPING_t){EXTI7,GPIOC,GPIO_PIN7,EXTI7_IRQ}
#define EXTI7PD7			(EXTI_GPIO_MAPPING_t){EXTI7,GPIOD,GPIO_PIN7,EXTI7_IRQ}


/* EXTI8 */
#define EXTI8PA8			(EXTI_GPIO_MAPPING_t){EXTI8,GPIOA,GPIO_PIN8,EXTI8_IRQ}
#define EXTI8PB8			(EXTI_GPIO_MAPPING_t){EXTI8,GPIOB,GPIO_PIN8,EXTI8_IRQ}
#define EXTI8PC8			(EXTI_GPIO_MAPPING_t){EXTI8,GPIOC,GPIO_PIN8,EXTI8_IRQ}
#define EXTI8PD8			(EXTI_GPIO_MAPPING_t){EXTI8,GPIOD,GPIO_PIN8,EXTI8_IRQ}


/* EXTI9 */
#define EXTI9PA9			(EXTI_GPIO_MAPPING_t){EXTI9,GPIOA,GPIO_PIN9,EXTI9_IRQ}
#define EXTI9PB9			(EXTI_GPIO_MAPPING_t){EXTI9,GPIOB,GPIO_PIN9,EXTI9_IRQ}
#define EXTI9PC9			(EXTI_GPIO_MAPPING_t){EXTI9,GPIOC,GPIO_PIN9,EXTI9_IRQ}
#define EXTI9PD9			(EXTI_GPIO_MAPPING_t){EXTI9,GPIOD,GPIO_PIN9,EXTI9_IRQ}


/* EXTI10 */
#define EXTI10PA10			(EXTI_GPIO_MAPPING_t){EXTI10,GPIOA,GPIO_PIN10,EXTI10_IRQ}
#define EXTI10PB10			(EXTI_GPIO_MAPPING_t){EXTI10,GPIOB,GPIO_PIN10,EXTI10_IRQ}
#define EXTI10PC10			(EXTI_GPIO_MAPPING_t){EXTI10,GPIOC,GPIO_PIN10,EXTI10_IRQ}
#define EXTI10PD10			(EXTI_GPIO_MAPPING_t){EXTI10,GPIOD,GPIO_PIN10,EXTI10_IRQ}


/* EXTI11 */
#define EXTI11PA11			(EXTI_GPIO_MAPPING_t){EXTI11,GPIOA,GPIO_PIN11,EXTI11_IRQ}
#define EXTI11PB11			(EXTI_GPIO_MAPPING_t){EXTI11,GPIOB,GPIO_PIN11,EXTI11_IRQ}
#define EXTI11PC11			(EXTI_GPIO_MAPPING_t){EXTI11,GPIOC,GPIO_PIN11,EXTI11_IRQ}
#define EXTI11PD11			(EXTI_GPIO_MAPPING_t){EXTI11,GPIOD,GPIO_PIN11,EXTI11_IRQ}


/* EXTI12 */
#define EXTI12PA12			(EXTI_GPIO_MAPPING_t){EXTI12,GPIOA,GPIO_PIN12,EXTI12_IRQ}
#define EXTI12PB12			(EXTI_GPIO_MAPPING_t){EXTI12,GPIOB,GPIO_PIN12,EXTI12_IRQ}
#define EXTI12PC12			(EXTI_GPIO_MAPPING_t){EXTI12,GPIOC,GPIO_PIN12,EXTI12_IRQ}
#define EXTI12PD12			(EXTI_GPIO_MAPPING_t){EXTI12,GPIOD,GPIO_PIN12,EXTI12_IRQ}


/* EXTI13 */
#define EXTI13PA13			(EXTI_GPIO_MAPPING_t){EXTI13,GPIOA,GPIO_PIN13,EXTI13_IRQ}
#define EXTI13PB13			(EXTI_GPIO_MAPPING_t){EXTI13,GPIOB,GPIO_PIN13,EXTI13_IRQ}
#define EXTI13PC13			(EXTI_GPIO_MAPPING_t){EXTI13,GPIOC,GPIO_PIN13,EXTI13_IRQ}
#define EXTI13PD13			(EXTI_GPIO_MAPPING_t){EXTI13,GPIOD,GPIO_PIN13,EXTI13_IRQ}


/* EXTI14 */
#define EXTI14PA14			(EXTI_GPIO_MAPPING_t){EXTI14,GPIOA,GPIO_PIN14,EXTI14_IRQ}
#define EXTI14PB14			(EXTI_GPIO_MAPPING_t){EXTI14,GPIOB,GPIO_PIN14,EXTI14_IRQ}
#define EXTI14PC14			(EXTI_GPIO_MAPPING_t){EXTI14,GPIOC,GPIO_PIN14,EXTI14_IRQ}
#define EXTI14PD14			(EXTI_GPIO_MAPPING_t){EXTI14,GPIOD,GPIO_PIN14,EXTI14_IRQ}


/* EXTI15 */
#define EXTI15PA15			(EXTI_GPIO_MAPPING_t){EXTI15,GPIOA,GPIO_PIN15,EXTI15_IRQ}
#define EXTI15PB15			(EXTI_GPIO_MAPPING_t){EXTI15,GPIOB,GPIO_PIN15,EXTI15_IRQ}
#define EXTI15PC15			(EXTI_GPIO_MAPPING_t){EXTI15,GPIOC,GPIO_PIN15,EXTI15_IRQ}
#define EXTI15PD15			(EXTI_GPIO_MAPPING_t){EXTI15,GPIOD,GPIO_PIN15,EXTI15_IRQ}


/* @ref EXTI_TRIGGER_SELECT_DEFINE */
#define  EXTI_RISING_TRIG 					0
#define  EXTI_FALLING_TRIG 					1
#define  EXTI_RISING_FALLING_TRIG 			2


/* @ref EXTI_IRQ_DEFINE */
#define EXTI_IRQ_DISABLE					0
#define EXTI_IRQ_ENABLE						1


/*===============================================================================
 *           		    	   	   Generic Macros  		  	                     *
 ================================================================================*/

/*Configuration Reference*/
#define EXTI0		0
#define EXTI1		1
#define EXTI2		2
#define EXTI3		3
#define EXTI4		4
#define EXTI5		5
#define EXTI6		6
#define EXTI7		7
#define EXTI8		8
#define EXTI9		9
#define EXTI10		10
#define EXTI11		11
#define EXTI12		12
#define EXTI13		13
#define EXTI14		14
#define EXTI15		15



/*===============================================================================
 *                                	   APIs 		   		                     *
 ================================================================================*/
/**===============================================================================
 * Function Name  : MCAL_EXTI_GPIO_Init.
 * Brief          : Function To Initiate The EXTI Configuration.
 * Parameter (in) : Pointer to EXTI Structure Configuration.
 * Return         : None.
 * Note           : None.                                                                           */
void MCAL_EXTI_GPIO_Init(EXTI_PinConfig_t *p_EXTI_Config);

/**===============================================================================
 * Function Name  : MCAL_EXTI_GPIO_DeInit.
 * Brief          : Function To Reset The EXTI Configuration.
 * Parameter (in) : None.
 * Return         : None.
 * Note           : None.                                                                           */
void MCAL_EXTI_GPIO_DeInit();

/**===============================================================================
 * Function Name  : MCAL_EXTI_GPIO_Update.
 * Brief          : Function To Update The EXTI Configuration.
 * Parameter (in) : Pointer to EXTI Structure Configuration.
 * Return         : None.
 * Note           : None.                                                                           */
void MCAL_EXTI_GPIO_Update(EXTI_PinConfig_t *p_EXTI_Config);



#endif /* INC_STM32F103C8_EXTI_DRIVER_H_ */
