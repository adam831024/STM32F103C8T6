/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *		main.c
 * DESCRIPTION:
 *
 *******************************************************************************/
/******************************************************************************
 * Includes
 *******************************************************************************/
/*Standard include*/
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rtc.h"              // Keil::Device:StdPeriph Drivers:RTC
#include "stm32f10x_wwdg.h"             // Keil::Device:StdPeriph Drivers:WWDG

#include <stdint.h>
#include <stdio.h>

/*Free-RTOS include*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "timers.h"
#include "task.h"
#include "queue.h"

/*Application include*/
#include "osUtility.h"
#include "osWatchDog.h"
#include "osUart.h"
#include "osTask.h"
#include "message.h"

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
/*timer*/
TimerHandle_t stateCheckTimerHandle = NULL;

/*task*/
TaskHandle_t mainTaskHandle = NULL;

/*task arg*/
uint8_t mainTaskArg = 0x01;
/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
/******************************************************************************
 * @brief     delay init, no use
 * @return                              void
 *******************************************************************************/
void sysTickDalayInit(void) 
{
  SysTick_Config(SystemCoreClock/1000);
}

/******************************************************************************
 * @brief     Init HSE 72M
 * @return                              void
 *******************************************************************************/
void SetSysClockTo72(void)
{
	ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
    	//FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

        /* Flash 2 wait state */
        //FLASH_SetLatency( FLASH_Latency_2);

        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);

        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2);

        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd( ENABLE);

        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */

        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

/******************************************************************************
 * @brief     GPIO Pin Initialize
 * @param[in] gpioGroup               		
 * @param[in] gpioPin               		
 * @param[in] mode                		
 * @param[in] speed                		
 * @return                              void
 *******************************************************************************/
void initGpioPin(uint32_t gpioGroup, uint16_t gpioPin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_TypeDef* gpioBase;
  RCC_APB2PeriphClockCmd(gpioGroup, ENABLE);
  GPIO_InitStructure.GPIO_Pin = gpioPin;
  GPIO_InitStructure.GPIO_Mode = mode;
  if(speed)
  {
    GPIO_InitStructure.GPIO_Speed = speed;
  }
  else
  {
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  }
  switch (gpioGroup)
  {
    case RCC_APB2Periph_GPIOA:
    {
      gpioBase = GPIOA;
    }
    break;
    case RCC_APB2Periph_GPIOB:
    {
      gpioBase = GPIOB;
    }
    break;
    case RCC_APB2Periph_GPIOC:
    {
      gpioBase = GPIOC;
    }
    break;
    case RCC_APB2Periph_GPIOD:
    {
      gpioBase = GPIOD;
    }
    break;
    case RCC_APB2Periph_GPIOE:
    {
      gpioBase = GPIOE;
    }
    break;
    case RCC_APB2Periph_GPIOF:
    {
      gpioBase = GPIOF;
    }
    break;
    
    default:
      break;
  }
  GPIO_Init(gpioBase, &GPIO_InitStructure);
}
/******************************************************************************
 * @brief     state check timer callack every second
 * @param[out] xTimer             		the pointer to the TimerHandle_t.
 * @return                              void
 *******************************************************************************/
static void stateCheckTimerCb(TimerHandle_t xTimer)
{
  /*run every second*/
  static uint8_t count = 0;
  uartSend(&count, 1);
  count++;
}

/******************************************************************************
 * @brief     OS main task
 * @param[out] pvParameters             event arg
 * @return                              void
 *******************************************************************************/
static void mainTask(void *pvParameters)
{
  while (1)
  {
    /* toggle C13*/
    GPIOC->ODR ^= GPIO_Pin_13;
    uartSend((uint8_t*)pvParameters, 1);
    vTaskDelay(1000);
    taskYIELD();
  }
}
/******************************************************************************
 * @brief     main
 * @return    0
 *******************************************************************************/
int main(void)
{
  /*systick timer*/
  SetSysClockTo72();
  SysTick_Init(72);

  /* Initialize USART*/
  usart_init();

  /*LED init*/
  initGpioPin(RCC_APB2Periph_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);

  /*WDT init*/
  osWatchDogInit();
 
  /* system delay */
  delay_ms(1000);
  
  stateCheckTimerHandle = xTimerCreate("stateCheck" /* The timer name. */,
                                        1000 / portTICK_PERIOD_MS /*const TickType_t xTimerPeriodInTicks*/,
                                        pdTRUE /*const UBaseType_t uxAutoReload, pdFALSE for on shot, pdTRUE for period*/,
                                        NULL /*void * const pvTimerID*/,
                                        stateCheckTimerCb /*TimerCallbackFunction_t pxCallbackFunction*/);
  xTaskCreate(mainTask, "mainTask", 256, (void *)&mainTaskArg, 2, &mainTaskHandle);
  xTimerStart(stateCheckTimerHandle, 0);
  MessageInit();
  funcHdlTaskInit();
  vTaskStartScheduler();
  return 0;
}
