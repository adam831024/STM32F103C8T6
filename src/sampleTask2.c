/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *		sampleTask2.c
 * DESCRIPTION:
 *
 *******************************************************************************/
/******************************************************************************
 * Includes
 *******************************************************************************/
/*Standard include*/
#include <stdint.h>

/*Free-RTOS include*/

/*Application include*/
#include "sampleTask2.h"
#include "osUart.h"

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
void sample2TaskFunc(osMsg_t *pMsg)
{
  uint16_t dataLen =  BUILD_UINT16(pMsg->msg.dataLen[0], pMsg->msg.dataLen[1]);
#if 1 //debug purpose
  uartSend((uint8_t *)pMsg, 4 + BUILD_UINT16(pMsg->msg.dataLen[0], pMsg->msg.dataLen[1]));
#endif
  switch (pMsg->msgId)
  {
    case SAMPLE2_MSG1:
    {
      // uartSend(pMsg->msg.data, dataLen);
    }
    break;
    case SAMPLE2_MSG2:
    {
      // uartSend(pMsg->msg.data, dataLen);
    }
    break;
    case SAMPLE2_MSG3:
    {
    }
    break;
    default:
      break;
  }
}
/*************** END OF FUNCTIONS *********************************************/
