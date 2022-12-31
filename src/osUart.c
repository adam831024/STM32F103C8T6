/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *		osUart.c
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

/*Free-RTOS include*/

/*Application include*/
#include "osUart.h"

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
volatile uint8_t rxData;
/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
/******************************************************************************
 * @brief     usart1 init
 * @return                              void
 *******************************************************************************/
void usart_init(void)
{
  /* Enable USART1 and GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  /* NVIC Configuration */
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure the GPIOs */
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure the USART1 */
  USART_InitTypeDef USART_InitStructure;

  /* USART1 configuration ------------------------------------------------------*/
  /* USART1 configured as follow:
    - BaudRate = 115200 baud
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
    - USART Clock disabled
    - USART CPOL: Clock is active low
    - USART CPHA: Data is captured on the middle
    - USART LastBit: The clock pulse of the last data bit is not output to
      the SCLK pin
   */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);

  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);

  /* Enable the USART1 Receive interrupt: this interrupt is generated when the
    USART1 receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}
/******************************************************************************
 * @brief     USART1 Rx interrupt Handle
 * @return    void
 *******************************************************************************/
void USART1_IRQHandler(void)
{
  if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
  {
    rxData = USART_ReceiveData(USART1);

    // loop back
    USART_SendData(USART1, rxData);
  }
}

/******************************************************************************
 * @brief     USART1 Tx Send
 * @param[in] data
 * @param[in] dataLen
 *  length of data
 * @return    void
 *******************************************************************************/
void uartSend(uint8_t* data, uint16_t dataLen)
{
  for (uint16_t i = 0; i < dataLen; i++)
  {
    USART_SendData(USART1, data[i]);
  }
}


/*************** END OF FUNCTIONS *********************************************/
