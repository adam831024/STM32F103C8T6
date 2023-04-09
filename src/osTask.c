/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *		osTask.c
 * DESCRIPTION:
 *
 *******************************************************************************/
/******************************************************************************
 * Includes
 *******************************************************************************/
/*Standard include*/
#include <stdint.h>

/*Free-RTOS include*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*Application include*/
#include "osTask.h"
#include "osUart.h"
#include "osUtility.h"
#include "sampleTask1.h"
#include "sampleTask2.h"
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
const taskFuncStruct taskFuncTable[] =
{
  sample1TaskFunc,
  sample2TaskFunc,
};
/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
/******************************************************************************
 * @brief     OS main task
 * @param[out] pvParameters             event arg
 * @return                              void
 *******************************************************************************/
static void funcHdlTask(void *pvParameters)
{
  void *pData = NULL;
  uint16_t dataLen = 0;
  while (1)
  {
    xQueueReceive(taskMessageQueue, /* The handle of the queue. */
                  &pData,           /* Store the buffer’s address in pcReceivedString. */
                  portMAX_DELAY);
    if (pData)
    {
      osMsg_t *pBuf = (osMsg_t*) pData;
      dataLen = BUILD_UINT16(pBuf->msg.dataLen[0], pBuf->msg.dataLen[1]);
      uartSend(pBuf->msg.data, dataLen);
      if (END_OF_TASK_ID > pBuf->taskId)
      {
        taskFuncTable[pBuf->taskId](pBuf);
      }
      else
      {
        /*assert no taskID*/
      }
      
      osFree(pData);
    }
    taskYIELD();
  }
}

/******************************************************************************
 * @brief     Initialize whole task function handler
 * @return                              void
 *******************************************************************************/
void funcHdlTaskInit(void)
{
  xTaskCreate(funcHdlTask, "funcHdlTask", 512, NULL /*arg*/, 2, NULL /*pxCreatedTask*/);
}
/*************** END OF FUNCTIONS *********************************************/
