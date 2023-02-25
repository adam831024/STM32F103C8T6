/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *		osWatchDog.c
 * DESCRIPTION:
 *
*******************************************************************************/
/******************************************************************************
 * Includes
 *******************************************************************************/
/*Standard include*/
#include <stdint.h>
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"
#include "stm32f10x_wwdg.h"

/*Free-RTOS include*/

/*Application include*/


/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/

/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/

/******************************************************************************
 * Module Typedefs
 *******************************************************************************/

/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
/******************************************************************************
 * @brief     Watchdog interrupt Handle
 * @return                              void
 *******************************************************************************/
void WWDG_IRQHandler(void)
{
	WWDG_ClearFlag(); // This function reset flag WWDG->SR and cancel the resetting
	WWDG_SetCounter(100);

	/* Toggle LED which connected to PC13*/
	// GPIOC->ODR ^= GPIO_Pin_13;
}

/******************************************************************************
 * @brief     Initialize Watchdog
 * @return                              void
 *******************************************************************************/
void osWatchDogInit(void)
{
	/* Enable Watchdog*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	WWDG_DeInit();
	WWDG_SetPrescaler(WWDG_Prescaler_8); // 1, 2, 4, 8
	WWDG_SetWindowValue(127);						 // 64...127
	WWDG_Enable(100);
	WWDG_EnableIT();

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn; /*WWDG interrupt*/
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); /*    NVIC initialization*/
}

/*************** END OF FUNCTIONS *********************************************/
