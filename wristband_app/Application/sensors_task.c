/*
 * sensors_task.c
 *
 *  Created on: Feb 19, 2021
 *      Author: Aidan Clyens
 */

/*******************************************************************************
 * INCLUDES
 */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/knl/Task.h>

#include <uartlog/UartLog.h>

#include <icall.h>


/*********************************************************************
 * CONSTANTS
 */
#define SENSORS_THREAD_STACK_SIZE                      1024
#define SENSORS_TASK_PRIORITY                          1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
Task_Struct sensorsTask;
uint8_t sensorsTaskStack[SENSORS_THREAD_STACK_SIZE];

/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
// Task functions
static void Sensors_init(void);
static void Sensors_taskFxn(UArg a0, UArg a1);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Sensors_createTask
 *
 * @brief   Task creation function for Wristband sensors.
 */
void Sensors_createTask(void) {
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = sensorsTaskStack;
    taskParams.stackSize = SENSORS_THREAD_STACK_SIZE;
    taskParams.priority = SENSORS_TASK_PRIORITY;

    Task_construct(&sensorsTask, Sensors_taskFxn, &taskParams, Error_IGNORE);
}

/*********************************************************************
 * @fn      Sensors_init
 *
 * @brief   Initialization for Sensors task.
 */
static void Sensors_init(void) {
    // Register application with ICall
    ICall_registerApp(&selfEntity, &syncEvent);
}

/*********************************************************************
 * @fn      Sensors_taskFxn
 *
 * @brief   Application task entry point for the Wristband sensors.
 *
 * @param   a0, a1 - not used.
 */
static void Sensors_taskFxn(UArg a0, UArg a1) {
    Sensors_init();

    for (;;) {

    }
}

