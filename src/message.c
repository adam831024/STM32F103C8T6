/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *    message.c
 * DESCRIPTION:
 *
*******************************************************************************/
/******************************************************************************
 * Includes
 *******************************************************************************/
/*Standard include*/
#include <stdio.h>
#include <string.h>

/*Free-RTOS include*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "timers.h"
#include "queue.h"

/*Application include*/
#include "message.h"
#include "osUtility.h"

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
/*Queue taskMessage*/
QueueHandle_t taskMsgQueue;
msgBlockState_t taskMsgBlockList[MSG_QUEUE_SIZE];
msgSuspend_t taskMsgSuspBuf[MSG_QUEUE_SIZE];
/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
/******************************************************************************
 * Function Definitions
 *******************************************************************************/
/******************************************************************************
 * @brief         brief Init message buffer     
 * @return                              void
 *******************************************************************************/
void MessageInit( void )
{
  /* Create a queue that can hold a maximum of MSG_QUEUE_SIZE pointers, in this case character pointers. */
  taskMsgQueue = xQueueCreate(MSG_QUEUE_SIZE, sizeof(char *));
  memset(taskMsgBlockList, INVALID_ID, MSG_QUEUE_SIZE * sizeof(msgBlockState_t));
  memset(taskMsgSuspBuf, NULL, sizeof(msgSuspend_t) * MSG_QUEUE_SIZE);
}

/******************************************************************************
 * @brief     Use TimerHandler change Suspend Buffer index
 * @param[in] xTimer             		the pointer to the TimerHandle_t.
 * @return                          Success: suspend buffer index
 *                                  Fail: MSG_QUEUE_SIZE
 *******************************************************************************/
static uint8_t msgTimerHdlChangeBufIdx(TimerHandle_t xTimer)
{
  for (uint8_t i = 0; i < MSG_QUEUE_SIZE;i++)
  {
    if (taskMsgSuspBuf[i].timerHandle == xTimer && taskMsgSuspBuf[i].used == 1)
    {
      return i;
    }
  }
  return MSG_QUEUE_SIZE;
}

/******************************************************************************
 * @brief     Get Empty Space index in suspend buffer
 * @return                          Success: suspend buffer index
 *                                  Fail: MSG_QUEUE_SIZE
 *******************************************************************************/
static uint8_t msgGetEmptySuspBufIdx(void)
{
  for (uint8_t i = 0; i < MSG_QUEUE_SIZE;i++)
  {
    if (taskMsgSuspBuf[i].used == 0)
    {
      return i;
    }
  }
  return MSG_QUEUE_SIZE;
}

/******************************************************************************
 * @brief     After Take the message in Suspend buffer, Clean up
 * @param[in] index                 Suspend buffer index
 * @return                          void
 *******************************************************************************/
static void msgClearSuspBufIdx(uint8_t index)
{
  if (index < MSG_QUEUE_SIZE)
  {
    taskMsgSuspBuf[index].used = 0;
    taskMsgSuspBuf[index].timerHandle = NULL;
    taskMsgSuspBuf[index].payload = NULL;
  }
}
/******************************************************************************
 * @brief     check message is block or not
 * @param[in] task
 *  The task to deliver the message to.
 * @param[in] id
 *  The message type identifier.
 * @return                         1 for block, 0 for unblock
 *******************************************************************************/
static uint8_t msgCheckBlock(Task task, MessageId id)
{
  for (uint8_t i = 0; i < MSG_QUEUE_SIZE; i++)
  {
    if ((taskMsgBlockList[i].msgId == id && taskMsgBlockList[i].taskId == task) ||
        (taskMsgBlockList[i].msgId == VALID_ID && taskMsgBlockList[i].taskId == task))
    {
      return 1;
    }
  }
  return 0;
}
/******************************************************************************
 * @brief     MessageSendLater timeout callback
 * @param[out] xTimer             		
 *  the pointer to the TimerHandle_t.
 * @return                              void
 *******************************************************************************/
static void MessageTimerCb(TimerHandle_t xTimer)
{
  uint8_t index = msgTimerHdlChangeBufIdx(xTimer);
  if (index < MSG_QUEUE_SIZE)
  {
    if(taskMsgSuspBuf[index].payload)
    {
      if (!msgCheckBlock(((osMsg_t *)taskMsgSuspBuf[index].payload)->taskId, ((osMsg_t *)taskMsgSuspBuf[index].payload)->msgId))
      {
        xQueueSend(taskMsgQueue, &taskMsgSuspBuf[index].payload, portMAX_DELAY);
        msgClearSuspBufIdx(index);
      }
    }
    taskMsgSuspBuf[index].timerHandle = NULL;
  }
  xTimerDelete(xTimer, 0);
}
/******************************************************************************
 * @brief
 *  Send a message to the corresponding task immediately.
 * The message will be passed to free after delivery.
 * @param[in] task
 *  The task to deliver the message to.
 * @param[in] id
 *  The message type identifier.
 * @param[in] message
 *  message_t, at least 2 bytes for length
 * @return                              void
 *******************************************************************************/
void MessageSend( Task task, MessageId id, void* message )
{
  osMsg_t *pMsg = NULL;
  uint16_t messageLen = 0;
  uint8_t index = MSG_QUEUE_SIZE;
  if(message)
  {
    messageLen = BUILD_UINT16(((uint8_t*)message)[0], ((uint8_t*)message)[1]);
  }
  pMsg = osMalloc(sizeof(osMsg_t)+messageLen-1);
  if (pMsg)
  {
    pMsg->msgId = id;
    pMsg->taskId = task;
    pMsg->msg.dataLen[0] = ((uint8_t *)message)[0];
    pMsg->msg.dataLen[1] = ((uint8_t *)message)[1];
    if (messageLen)
    {
      memcpy(pMsg->msg.data, &((uint8_t *)message)[2], messageLen);
    }

    if(!msgCheckBlock(task, id))
    {
      xQueueSend(taskMsgQueue, &pMsg, portMAX_DELAY);
    }
    else
    {
      index = msgGetEmptySuspBufIdx();
      if (index < MSG_QUEUE_SIZE)
      {
        taskMsgSuspBuf[index].used = 1;
        taskMsgSuspBuf[index].payload = pMsg;
        taskMsgSuspBuf[index].timerHandle = NULL;
      }
      else
      {
        osFree(pMsg);
      }
    }
  }
}

/******************************************************************************
 * @brief
 *  Send a message to the corresponding task after the given delay in ms.
 * The message will be passed to free after delivery.
 * @param[in] task
 *  The taskId to deliver the message to.
 * @param[in] id
 *  The message type identifier.
 * @param[in] message
 *  message_t, at least 2 bytes for length
 * @param[in] delay
 *  The delay in ms before the message will be sent.
 * @return                              void
 *******************************************************************************/
void MessageSendLater(Task task, MessageId id, void *message, uint32_t delay)
{
  osMsg_t *pMsg = NULL;
  uint16_t messageLen = 0;
  TimerHandle_t handle = NULL;
  uint8_t index = MSG_QUEUE_SIZE;
  if (message)
  {
    messageLen = BUILD_UINT16(((uint8_t *)message)[0], ((uint8_t *)message)[1]);
  }
  index = msgGetEmptySuspBufIdx();
  if (index < MSG_QUEUE_SIZE)
  {
    pMsg = osMalloc(sizeof(osMsg_t) + messageLen - 1);
    if (pMsg)
    {
      pMsg->msgId = id;
      pMsg->taskId = task;
      pMsg->msg.dataLen[0] = ((uint8_t *)message)[0];
      pMsg->msg.dataLen[1] = ((uint8_t *)message)[1];
      if (messageLen)
      {
        memcpy(pMsg->msg.data, &((uint8_t *)message)[2], messageLen);
      }

      handle = xTimerCreate("msgTimer", delay / portTICK_PERIOD_MS, pdFALSE, (void *)NULL, MessageTimerCb);
      if (handle)
      {
        taskMsgSuspBuf[index].payload = pMsg;
        taskMsgSuspBuf[index].timerHandle = handle;
        taskMsgSuspBuf[index].used = 1;
        xTimerStart(handle, 0);
      }
      else
      {
        osFree(pMsg);
      }
    }
  }
}

/******************************************************************************
 * @brief
 *  Cancel all queued messages (independent of id) for the given taskId.
 * @param[in] task
 *  The taskId to flush all message for.
 * @return
 *  The number of messages removed from the queue.
 ********************************************************************************/
uint16_t MessageFlushTask(Task task)
{
  osMsg_t *pMsg = NULL;
  UBaseType_t queueCount = MSG_QUEUE_SIZE;
  uint16_t ret = 0;
  taskENTER_CRITICAL();
  queueCount -= uxQueueSpacesAvailable(taskMsgQueue);
  //check runnung queue
  for (uint8_t i = 0; i < queueCount; i++)
  {
    xQueueReceive(taskMsgQueue, &pMsg, portMAX_DELAY);
    if (pMsg)
    {
      if (pMsg->taskId != task)
      {
        xQueueSend(taskMsgQueue, &pMsg, portMAX_DELAY);
      }
      else
      {
        ret++;
        osFree(pMsg);
      }
    }
  }
  // check suspend buffer
  for (uint8_t i = 0; i < MSG_QUEUE_SIZE; i++)
  { 
    if (taskMsgSuspBuf[i].used == 1)
    {
      if (taskMsgSuspBuf[i].payload)
      {
        if (((osMsg_t *)taskMsgSuspBuf[i].payload)->taskId == task)
        {
          if(taskMsgSuspBuf[i].timerHandle)
          {
            xTimerDelete(taskMsgSuspBuf[i].timerHandle, 0);
          }
          osFree(taskMsgSuspBuf[i].payload);
          msgClearSuspBufIdx(i);
          ret++;
        }
      }
      else
      {
        //error
      }
    }
  }
  taskEXIT_CRITICAL();
  return ret;
}

/******************************************************************************
 * @brief
 *  flush all data in queue by taskId amd messageId
 * @param[in] task
 *  The taskId to flush all message for.
 * @param[in] id
 *  The messageId to flush all message for.
 * @return
 *  The number of messages removed from the queue.
 *******************************************************************************/
uint16_t MessageCancelAll(Task task, MessageId id)
{
  osMsg_t *pMsg = NULL;
  UBaseType_t queueCount = MSG_QUEUE_SIZE;
  uint16_t ret = 0;

  taskENTER_CRITICAL();
  queueCount -= uxQueueSpacesAvailable(taskMsgQueue);

  for (uint32_t i = 0; i < queueCount; i++)
  {
    xQueueReceive(taskMsgQueue, &pMsg, portMAX_DELAY);
    if (pMsg->taskId != task || pMsg->msgId != id)
    {
      xQueueSend(taskMsgQueue, &pMsg, portMAX_DELAY);
    }
    else
    {
      if (pMsg)
      {
        ret++;
        osFree(pMsg);
      }
    }
  }

  // check suspend buffer
  for (uint8_t i = 0; i < MSG_QUEUE_SIZE; i++)
  {
    if (taskMsgSuspBuf[i].used == 1)
    {
      if (taskMsgSuspBuf[i].payload)
      {
        if (((osMsg_t *)taskMsgSuspBuf[i].payload)->taskId == task && ((osMsg_t *)taskMsgSuspBuf[i].payload)->msgId == id)
        {
          if (taskMsgSuspBuf[i].timerHandle)
          {
            xTimerDelete(taskMsgSuspBuf[i].timerHandle, 0);
          }
          osFree(taskMsgSuspBuf[i].payload);
          msgClearSuspBufIdx(i);
          ret++;
        }
      }
      else
      {
        //error
      }
    }
  }
  taskEXIT_CRITICAL();
  return ret;
}
/******************************************************************************
 * @brief
 *  block all data in queue by taskId amd messageId
 * @param[in] task
 *  The taskId to flush all message for.
 * @param[in] id
 *  The messageId to flush all message for.
 *******************************************************************************/
void MessageBlock(Task task, MessageId id)
{
  osMsg_t *pMsg = NULL;
  UBaseType_t queueCount = MSG_QUEUE_SIZE;
  uint8_t index = MSG_QUEUE_SIZE;

  taskENTER_CRITICAL();

  //add block list
  for (uint8_t i = 0; i < MSG_QUEUE_SIZE; i++)
  {
    if (taskMsgBlockList[i].taskId == INVALID_ID)
    {
      taskMsgBlockList[i].taskId = task;
      taskMsgBlockList[i].msgId = id;
      break;
    }
  }
  
  //move msg from runnung Queue to suspend buffer
  queueCount -= uxQueueSpacesAvailable(taskMsgQueue);
  for (uint8_t i = 0; i < queueCount; i++)
  {
    xQueueReceive(taskMsgQueue, &pMsg, portMAX_DELAY);
    if (pMsg->taskId != task || pMsg->msgId != id)
    {
      xQueueSend(taskMsgQueue, &pMsg, portMAX_DELAY);
    }
    else
    {
      index = msgGetEmptySuspBufIdx();
      if (index < MSG_QUEUE_SIZE)
      {
        taskMsgSuspBuf[index].used = 1;
        taskMsgSuspBuf[index].payload = pMsg;
      }
    }
  }
  taskEXIT_CRITICAL();
}
/******************************************************************************
 * @brief
 *  unblock all data in queue by taskId amd messageId
 * @param[in] task
 *  The taskId to flush all message for.
 * @param[in] id
 *  The messageId to flush all message for.
 *******************************************************************************/
void MessageUnblock(Task task, MessageId id)
{
  taskENTER_CRITICAL();
  //clean block list
  for (uint8_t i = 0; i < MSG_QUEUE_SIZE; i++)
  {
    if (taskMsgBlockList[i].taskId == task && taskMsgBlockList[i].msgId == id)
    {
      memset(&taskMsgBlockList[i], INVALID_ID, sizeof(msgBlockState_t));
    }
  }

  // move msg from suspend buffer to runnung Queue
  for (uint8_t i = 0; i < MSG_QUEUE_SIZE; i++)
  {
    if (taskMsgSuspBuf[i].used == 1)
    {
      if (taskMsgSuspBuf[i].timerHandle == NULL)
      {
        if (taskMsgSuspBuf[i].payload)
        {
          if (((osMsg_t *)taskMsgSuspBuf[i].payload)->taskId == task && ((osMsg_t *)taskMsgSuspBuf[i].payload)->msgId == id)
          {
            xQueueSend(taskMsgQueue, &taskMsgSuspBuf[i].payload, portMAX_DELAY);
            msgClearSuspBufIdx(i);
          }
        }
        else
        {
          //error
        }
      }
    }
  }
  taskEXIT_CRITICAL();
}
/*************** END OF FUNCTIONS *********************************************/
