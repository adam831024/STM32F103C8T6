/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *    message.h
 * DESCRIPTION:
 *      header file of message.c
*******************************************************************************/
#ifndef _MESSAGE_H_
#define _MESSAGE_H_

/******************************************************************************
 * Includes
 *******************************************************************************/
/*Standard include*/
#include <stdint.h>
#include <stdlib.h>

/*Free-RTOS include*/
#include "FreeRTOS.h"
#include "queue.h"

/*Application include*/
#include "osUtility.h"
/******************************************************************************
 * Preprocessor Constants
 *******************************************************************************/
/**
 * This constant is
 */
#define MSG_QUEUE_SIZE  20
#define INVALID_ID      0xFF
#define VALID_ID        0x00
/******************************************************************************
 * Configuration Constants
 *******************************************************************************/

/******************************************************************************
 * Macros
 *******************************************************************************/

/******************************************************************************
 * Typedefs
 *******************************************************************************/
/*!
Message identifier type.
*/
typedef uint8_t MessageId;

/*!
Task identifier type.
*/
typedef uint8_t Task;

typedef Task TaskId ;

typedef struct message_t
{
  // uint16_t dataLen;
  uint8_t dataLen[2];
  uint8_t data[1];
} PACKED message_t;

typedef struct 
{
  TaskId taskId;
  MessageId msgId;
  message_t msg;
}osMsg_t;

typedef struct 
{
  TaskId taskId;
  MessageId msgId;
}msgBlockState_t;

typedef struct
{
  void* timerHandle;
  void* payload;
  uint8_t used;
}msgSuspend_t;

/******************************************************************************
 * Variables
 *******************************************************************************/
extern QueueHandle_t taskMsgQueue;

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void MessageInit( void );

void MessageSend( Task task, MessageId id, void* message );

void MessageSendLater(Task task, MessageId id, void *message, uint32_t delay);

uint16_t MessageFlushTask(Task task);

uint16_t MessageCancelAll(Task task, MessageId id);

void MessageBlock(Task task, MessageId id);

void MessageUnblock(Task task, MessageId id);


#endif /*_MESSAGE_H_*/

/*************** END OF FUNCTIONS *********************************************/
