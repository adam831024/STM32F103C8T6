/******************************************************************************
 * Copyright (C)
 *
 * NAME:
 *		osTask.h
 * DESCRIPTION:
 *      header file of osTask.c
 *******************************************************************************/
#ifndef _OS_TASK_H_
#define _OS_TASK_H_

/******************************************************************************
 * Includes
 *******************************************************************************/
/*Standard include*/
#include <stdint.h>

/*Application include*/
#include "message.h"

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
enum TaskId
{
  SAMPLE_TASK_1,
  SAMPLE_TASK_2,

  END_OF_TASK_ID,
};
/******************************************************************************
 * Typedefs
 *******************************************************************************/
typedef void (*taskFuncStruct)(osMsg_t *pMsg);
/******************************************************************************
 * Variables
 *******************************************************************************/

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void funcHdlTaskInit(void);

#endif /*_OS_TASK_H_*/
/*************** END OF FUNCTIONS *********************************************/
