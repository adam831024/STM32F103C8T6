#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rtc.h"              // Keil::Device:StdPeriph Drivers:RTC
#include <stdint.h>
#include <stdio.h>

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
 * @brief     main
 * @return    0
 *******************************************************************************/
int main(void)
{
  /*systick timer*/
  SetSysClockTo72();
  SysTick_Init(72);
  
  /*LED init*/
  initGpioPin(RCC_APB2Periph_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
  while (1)
  {
      /* delay */
      delay_ms(1000);
      /* toggle C13*/
      GPIOC->ODR ^= GPIO_Pin_13; 
      /* hi C13*/
      // GPIO_SetBits(GPIOC, GPIO_Pin_13);
      /* lo C13*/
      // GPIO_ResetBits(GPIOC, GPIO_Pin_13);
  }

  return 0;
}
