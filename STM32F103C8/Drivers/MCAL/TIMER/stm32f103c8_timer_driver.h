/*============================================================================================
 * Module : TIMER
 *
 * File Name : stm32f103c8_timer_driver.h
 *
 * Author: Abdullah Maher
 *
 * Description : Header File Of STM32F103C8 TIMER Driver
 *
 * Created on: Sep 1, 2023
 =============================================================================================*/

#ifndef INC_STM32F103C8_TIMER_DRIVER_H_
#define INC_STM32F103C8_TIMER_DRIVER_H_

/*===============================================================================
 *                                Includes                                       *
 ================================================================================*/
#include "stm32f103c8.h"

/*===============================================================================
 *                            User Type Definitions                              *
 ================================================================================*/
typedef struct
{
	uint16_t OnePulseMode;				/* Specifies Auto-reload One Pulse Mode enable/disable Regarding to @ref OnePulseMode_define */

	uint16_t Preload;					/* Specifies Auto-reload preload enable/disable Regarding to @ref Preload_define */

	uint16_t CounterMode; 				/* Specifies the counter mode Regarding to @ref CounterMode_define */

	uint16_t Prescaler; 				/* Specifies the prescaler value used to divide the TIM clock. */

	uint16_t Period; 					/* Specifies the period value to be loaded into the active Auto-Reload Register at the next update event. */

	uint8_t RepetitionCounter; 			/* Specifies the repetition counter value (Advanced Timers Only)  */

	uint16_t ActiveChannels;			/* Specifies the Active Channels Regarding to @ref ActiveChannels_define */

	uint16_t ChannelCompareValue[4];	/* Specifies the capture/compare register channel to be active when the counter matches */

	uint8_t ChannelMode[4];				/* Specifies the channel mode Regarding to @ref ChannelMode_define */

	uint16_t TIM_IRQ;					/* Specifies the The Timer Interrupt Mask Regarding to @ref TIM_Interrupt_Enable/Disable_define */

	void(*p_TIM_ISR)(void);				/* SET The Call Back Function That Will Be Called In ISR */

}TIM_Config_t;

/*===============================================================================
 *                       Macros Configuration References                         *
 ================================================================================*/
/* @ref OnePulseMode_define */
#define TIM_ONE_PULSE_DISABLE					(0x00000000U)
#define TIM_ONE_PULSE_ENABLE					(0x00000008U)

/* @ref Preload_define */
#define	TIM_PRELOAD_NOT_BUFFERED				(0x00000000U)	/* The content of the TIMx_ARR transferred to the shadow register permanently (register internal to the timer that effectively contains the counter value to match) */
#define	TIM_PRELOAD_BUFFERED					(0x00000080U)	/* The content of the TIMx_ARR transferred to the shadow register at each UEV  */

/*  @ref CounterMode_define */
#define TIM_COUNTERMODE_UP						(0x00000000U)
#define TIM_COUNTERMODE_DOWN					(0x00000010U)
#define TIM_COUNTERMODE_CENTERALIGNED1			(0x00000020U)	/* The counter counts up and down alternatively. Output compare interrupt flags of channels are set only when the counter is counting down */
#define TIM_COUNTERMODE_CENTERALIGNED2			(0x00000040U)	/* The counter counts up and down alternatively. Output compare interrupt flags of channels are set only when the counter is counting up */
#define TIM_COUNTERMODE_CENTERALIGNED3			(0x00000060U)	/* The counter counts up and down alternatively. Output compare interrupt flags of channels are set both when the counter is counting up or down. */

/* @ref ActiveChannels_define */
#define TIM_ACTIVE_CHANNEL_1					(0x00000001U)	/* Enable channel 1 as output with active high */
#define TIM_ACTIVE_CHANNEL_2					(0x00000010U)	/* Enable channel 2 as output with active high */
#define TIM_ACTIVE_CHANNEL_3					(0x00000100U)	/* Enable channel 3 as output with active high */
#define TIM_ACTIVE_CHANNEL_4					(0x00001000U)	/* Enable channel 4 as output with active high */
#define TIM_ACTIVE_ALL_CHANNEL					(0x00001111U)	/* Enable all channels as output with active high */

/* @ref ChannelMode_define */
#define TIM_CH_MODE_FROZEN						(0x00000000U) 	/* The comparison between the output compare register (CCRx) and the counter (CNT) has no effect on the output (Time base mode) */
#define TIM_CH_MODE_OC_ACTIVE					(0x00000010U)	/* Set the channel output to active level on match */
#define TIM_CH_MODE_OC_INACTIVE					(0x00000020U)	/* Set channel to inactive level on match */
#define TIM_CH_MODE_OC_TOGGLE					(0x00000030U)	/* The channel output toggles when the counter (CNT) matches the capture/compare */
#define TIM_CH_MODE_PWM1						(0x00000068U)	/* PWM Mode 1 - In upcounting, channel is active as long as the counter (CNT) smaller than  compare register (CCRx) else inactive. In downcounting channel is inactive as long as the counter (CNT) greater than compare register else inactive */
#define TIM_CH_MODE_PWM2						(0x00000078U)	/* PWM Mode 2 - In upcounting, channel is inactive as long as the counter (CNT) smaller than  compare register else active. In downcounting, channel is active as long as the counter (CNT) greater than compare register else inactive. */
#define TIM_CH_MODE_OC_FORCED_ACTIVE			(0x00000050U)	/* The channel output is forced high independently from the counter value */
#define TIM_CH_MODE_OC_FORCED_INACTIVE			(0x00000040U)	/* The channel output is forced low independently from the counter value */

/* @ref TIM_Interrupt_Enable/Disable_define */
#define TIM_IRQ_DISABLE							(0x00000000U)
#define TIM_IRQ_UIE								(0x00000001U)	/* Update Interrupt Enabled */
#define TIM_IRQ_CC1								(0x00000002U)	/* Capture/Compare 1 Interrupt Enabled */
#define TIM_IRQ_CC2								(0x00000004U)	/* Capture/Compare 2 Interrupt Enabled */
#define TIM_IRQ_CC3								(0x00000008U)	/* Capture/Compare 3 Interrupt Enabled */
#define TIM_IRQ_CC4								(0x00000010U)	/* Capture/Compare 4 Interrupt Enabled */


/*===============================================================================
 *           		    	    Generic Constants  	  		                     *
 ================================================================================*/
enum PWMChannel
{
	CHANNEL1,CHANNEL2,CHANNEL3,CHANNEL4
};


/*===============================================================================
 *                                	   APIs 		   		                     *
 ================================================================================*/

/*
 * Update Event = TIM_CLK / (Prescaler) * (Period) * (RepetitionCounter)
 *
 *	note: Repetition Counter is available in TIM1 (Advanced Contorl Timer) only.
 *
 */


/**===============================================================================
 * Function Name  : MCAL_Tim_Init.
 * Brief          : Function To Initialize Timer.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Parameter (in) : Pointer to TIM_Config_t Containing Configuration of The Timer.
 * Return         : None.
 * Note           : In case of using any channel you have to configure the pin as AF, Enable AFIO and GPIO.																				*/
void MCAL_Tim_Init(TIM_TypeDef* TIMx, TIM_Config_t* p_TIM_Config);

/**===============================================================================
 * Function Name  : MCAL_Tim_Init.
 * Brief          : Function To Reset Timer.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Return         : None.
 * Note           : None.																				*/
void MCAL_Tim_Deinit(TIM_TypeDef* TIMx);

/**===============================================================================
 * Function Name  : MCAL_Tim_Start.
 * Brief          : Function To Start Timer Counting.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Return         : None.
 * Note           : None.																				*/
void MCAL_Tim_Start(TIM_TypeDef* TIMx);

/**===============================================================================
 * Function Name  : MCAL_Tim_Stop.
 * Brief          : Function To Stop Timer Counting.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Return         : None.
 * Note           : None.																				*/
void MCAL_Tim_Stop(TIM_TypeDef* TIMx);

/**===============================================================================
 * Function Name  : MCAL_Tim_PWM.
 * Brief          : Function To Generate PWM.
 * Parameter (in) : Instant TIMx where x Could Be 1(Advanced) or 2,3,4(General Purpose).
 * Parameter (in) : The Desired Channel.
 * Parameter (in) : The Channel Compare Value.
 * Parameter (in) : The Period Value.
 * Parameter (in) : The Prescaler Value.
 * Return         : None.
 * Note           : None.																				*/
void MCAL_Tim_PWM(TIM_TypeDef* TIMx, uint32_t Channel, uint16_t DutyCycle, uint16_t Freq, uint16_t Presc);

#endif /* INC_STM32F103C8_TIMER_DRIVER_H_ */
