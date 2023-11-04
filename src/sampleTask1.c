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
#include "sampleTask1.h"
#include "sampleTask2.h"
#include "osTask.h"
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
void sample1TaskFunc(osMsg_t *pMsg)
{
  uint16_t dataLen = BUILD_UINT16(pMsg->msg.dataLen[0], pMsg->msg.dataLen[1]);
#if 1 //debug purpose
  uartSend((uint8_t*)pMsg, 4 + BUILD_UINT16(pMsg->msg.dataLen[0], pMsg->msg.dataLen[1]));
#endif
  switch (pMsg->msgId)
  {
    case SAMPLE1_MSG1:
    {
      uint8_t test[4] = {0,2,0xaa, 0xbb};
      MessageSend(SAMPLE_TASK_2, SAMPLE2_MSG2, test);
    }  
    break;
    case SAMPLE1_MSG2:
    {
      uint8_t test[4] = {0, 2, 0xcc, 0xdd};
      MessageSendLater(SAMPLE_TASK_2, SAMPLE2_MSG2, test, 3000);
    }  
    break;
    case SAMPLE1_MSG3:
    {
      MessageBlock(SAMPLE_TASK_2, SAMPLE2_MSG2);
    }  
    break;
    case SAMPLE1_MSG4:
    {
      MessageUnblock(SAMPLE_TASK_2, SAMPLE2_MSG2);
    }  
    break;
    case SAMPLE1_MSG5:
    {
      MessageCancelAll(SAMPLE_TASK_2, SAMPLE2_MSG2);
    }  
    break;
    default:
      break;
  }
}
/*************** END OF FUNCTIONS *********************************************/
