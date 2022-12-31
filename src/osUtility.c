/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *		osUtility.c
 * DESCRIPTION:
 *
*******************************************************************************/
/******************************************************************************
 * Includes
 *******************************************************************************/
/*Standard include*/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"                  // Device header

/*Free-RTOS include*/

/*Application include*/
#include "osUtility.h"

/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/
#define CONSTANT 5

/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/

/******************************************************************************
 * Module Typedefs
 *******************************************************************************/

/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/
static uint8_t	fac_us=0;
static uint16_t	fac_ms=0;
/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
/******************************************************************************
 * @brief     syste tick delay init
 * @return                              void
 *******************************************************************************/
void SysTick_Init(uint8_t SYSCLK)
{
	SysTick->CTRL = 0xfffffffb;
	fac_us=SYSCLK/8;
	fac_ms=(uint16_t)fac_us*1000;

}

/******************************************************************************
 * @brief     system delay us
 * @param[in] us                		microseconds
 * @return                              void
 *******************************************************************************/
void delay_us(uint16_t nus)
{
	uint32_t temp;
	SysTick->LOAD = (uint32_t)nus*fac_us;
	SysTick->VAL =0x00; 	
	SysTick->CTRL =0x01;
	do
	{
	 temp = SysTick->CTRL;
	}
	while((temp&0x01)&&(!(temp&(1<<16))));
	SysTick->CTRL = 0x00;
	SysTick->VAL =0x00;
}

/******************************************************************************
 * @brief     system delay ms
 * @param[in] ms                		milliseconds
 * @return                              void
 *******************************************************************************/
void delay_ms(uint16_t nms)
{
	uint32_t temp;
	SysTick->LOAD = (uint32_t)nms*fac_ms;
	SysTick->VAL =0x00; 	
	SysTick->CTRL =0x01;
	do
	{
	 temp = SysTick->CTRL;
	}
	while((temp&0x01)&&(!(temp&(1<<16))));
	SysTick->CTRL = 0x00;
	SysTick->VAL =0x00;
}

#if 0
/******************************************************************************
 * @DESCRIPTION: malloc
 * @param[in] size
 *      malloc size
 * @RETURNS: pointer 
*******************************************************************************/
void *osMalloc(uint16_t size)
{
    void *memory = (void *)pvPortMalloc(size);
    memset(memory, 0, size);
    return memory;
}
/******************************************************************************
 * @DESCRIPTION: 
 *      Free malloc
 * @param[in] ptr
 *      The pointer needed to be free
 * @RETURNS: void 
*******************************************************************************/
void osFree(void *ptr)
{
    if (ptr)
    {
        vPortFree(ptr);
    }
}
#endif
/*************** END OF FUNCTIONS *********************************************/
