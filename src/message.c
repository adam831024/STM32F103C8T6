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

/*Application include*/
#include "message.h"
#include "fifo.h"
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
QueueHandle_t taskMessageQueue;

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
/******************************************************************************
 * @brief
 *  brief Init message queue     
 * @return                              void
 *******************************************************************************/
void MessageInit( void )
{
  /* Create a queue that can hold a maximum of 10 pointers, in this case character pointers. */
  taskMessageQueue = xQueueCreate(10, sizeof(char *));
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
 *  The message data (if any).                 
 * @return                              void
 *******************************************************************************/
void MessageSend( Task task, MessageId id, void* message )
{
  osMsg_t *taskEvent = NULL;
  uint16_t messageLen = 0;
  if(message)
  {
    messageLen = BUILD_UINT16(((uint8_t*)message)[0], ((uint8_t*)message)[1]);
  }
  taskEvent = osMalloc(sizeof(osMsg_t)+messageLen-1);
  taskEvent->msgId = id;
  taskEvent->taskId = task;
  taskEvent->msg.dataLen[0] = ((uint8_t *)message)[0];
  taskEvent->msg.dataLen[1] = ((uint8_t *)message)[1];

  memcpy(taskEvent->msg.data, &((uint8_t*)message)[2], messageLen);
  // uartSend((uint8_t *)taskEvent, sizeof(osMsg_t) + messageLen - 1);
  xQueueSend(taskMessageQueue, /* The handle of the queue. */
             &taskEvent,        /* The address of the pointer that points to the buffer. */
             portMAX_DELAY);
}

/******************************************************************************
 * @brief     
 *  Send a message to the corresponding task after the given delay in ms.
 * The message will be passed to free after delivery.
 * @param[in] task     
 *  The task to deliver the message to.               
 * @param[in] id     
 *  The message type identifier.               
 * @param[in] message   
 *  The message data (if any).                    
 * @param[in] delay   
 *  The delay in ms before the message will be sent.                 
 * @return                              void
 *******************************************************************************/
void MessageSendLater(Task task, MessageId id, void *message, uint32_t delay)
{
  // create timer , timeout put queue
}

/******************************************************************************
 * @brief     
 *  Cancel all queued messages (independent of id) for the given task.
 * @param[in] task   
 *  The task to flush all message for.
 * @return                 
 *  The number of messages removed from the queue.  
********************************************************************************/
uint16_t MessageFlushTask(Task task)
{

}

/******************************************************************************
 * @brief     
 *  flush all data in queue by taskId amd messageId
 * @param[in] task   
 *  The task to flush all message for.
 * @param[in] id   
 *  The MessageId to flush all message for.
 * @return                              
 *  The number of messages removed from the queue.
 *******************************************************************************/
uint16_t MessageCancelAll(Task task, MessageId id)
{

}

/******************************************************************************
 * @brief     
 *  block the message in queue
 * @param[in] task                    
 * @param[in] id                                      
 * @return                              void
 *******************************************************************************/
void MessageBlock( Task task, MessageId id )
{

}

/******************************************************************************
 * @brief     
 *  block the message in queue
 * @param[in] task                    
 * @param[in] id                                      
 * @return                              void
 *******************************************************************************/
void MessageUnblock( Task task, MessageId id )
{

}

/******************************************************************************
 * @brief     
 *  block task in queue
 * @param[in] task                                     
 * @return                              void
 *******************************************************************************/
void MessageBlockTask ( Task task )
{

}

/******************************************************************************
 * @brief     
 *  unblock the task in queue
 * @param[in] task                                    
 * @return                              void
 *******************************************************************************/
void MessageUnblockTask ( Task task )
{

}
/*************** END OF FUNCTIONS *********************************************/
