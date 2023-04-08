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

/******************************************************************************
 * Preprocessor Constants
 *******************************************************************************/
/**
 * This constant is
 */

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

typedef struct 
{
  TaskId taskId;
  MessageId msgId;
  uint8_t msgLen[2];
  uint8_t msg[1];
}osMsg_t;

/******************************************************************************
 * Variables
 *******************************************************************************/
extern QueueHandle_t taskMessageQueue;

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void MessageInit( void );

void MessageSend( Task task, MessageId id, void* message );

void MessageSendLater(Task task, MessageId id, void *message, uint32_t delay);

uint16_t MessageFlushTask(Task task);

uint16_t MessageCancelAll(Task task, MessageId id);

void MessageBlockTask ( Task task );

void MessageUnblockTask ( Task task );

void MessageBlock( Task task, MessageId id );

void MessageUnblock( Task task, MessageId id );


#endif /*_MESSAGE_H_*/

/*************** END OF FUNCTIONS *********************************************/
