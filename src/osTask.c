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
  while (1)
  {
    xQueueReceive(taskMsgQueue, &pData, portMAX_DELAY);
    if (pData)
    {
      osMsg_t *pBuf = (osMsg_t*) pData;
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
  }
}

/******************************************************************************
 * @brief     Initialize whole task function handler
 * @return                              void
 *******************************************************************************/
void msgHdlTaskInit(void)
{
  xTaskCreate(funcHdlTask, "funcHdlTask", 512, NULL /*arg*/, 2, NULL /*pxCreatedTask*/);
}
/*************** END OF FUNCTIONS *********************************************/
