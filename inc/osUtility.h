/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *    osUtility.h
 * DESCRIPTION:
 *      header file of osUtility.c
*******************************************************************************/
#ifndef _OS_UTILITY_H_
#define _OS_UTILITY_H_

/******************************************************************************
 * Includes
 *******************************************************************************/
/*Standard include*/
#include <stdint.h>

/*Application include*/

/******************************************************************************
 * Preprocessor Constants
 *******************************************************************************/
/**
 * This constant is
 */
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
#define BUILD_UINT16(hiByte, loByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define PACKED __attribute__((packed))
/******************************************************************************
 * Configuration Constants
 *******************************************************************************/

/******************************************************************************
 * Macros
 *******************************************************************************/

/******************************************************************************
 * Typedefs
 *******************************************************************************/

/******************************************************************************
 * Variables
 *******************************************************************************/

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void delay_ms(uint16_t nms);
void delay_us(uint16_t nus);
void SysTick_Init(uint8_t SYSCLK);
void *osMalloc(uint16_t size);
void osFree(void *ptr);

/*************** END OF FUNCTIONS *********************************************/
#endif /*_OS_UTILITY_H_*/
