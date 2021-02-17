/*
 * mis2dh_task.c
 *
 *  Created on: Feb 17, 2021
 *      Author: Aidan Clyens
 */

/*******************************************************************************
 * INCLUDES
 */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/knl/Task.h>

#include <uartlog/UartLog.h>

#include <Board.h>

#include <mis2dh_task.h>

/*********************************************************************
 * CONSTANTS
 */
// Task
#define MIS2DH_THREAD_STACK_SIZE                      1024
#define MIS2DH_TASK_PRIORITY                          1

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */
Task_Struct mis2dhTask;
uint8_t mis2dhTaskStack[MIS2DH_THREAD_STACK_SIZE];

/*********************************************************************
 * LOCAL VARIABLES
 */


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Mis2dh_init(void);
static void Mis2dh_taskFxn(UArg a0, UArg a1);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Mis2dh_createTask
 *
 * @brief   Task creation function for the MIS2DH Accelerometer.
 */
void Mis2dh_createTask(void) {
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = mis2dhTaskStack;
    taskParams.stackSize = MIS2DH_THREAD_STACK_SIZE;
    taskParams.priority = MIS2DH_TASK_PRIORITY;

    Task_construct(&mis2dhTask, Mis2dh_taskFxn, &taskParams, Error_IGNORE);
}

/*********************************************************************
 * @fn      Mis2dh_inits
 *
 * @brief   Initialization for MIS2DH task.
 */
static void Mis2dh_init(void) {

}

/*********************************************************************
 * @fn      Mis2dh_taskFxn
 *
 * @brief   Application task entry point for the MIS2DH Accelerometer.
 *
 * @param   a0, a1 - not used.
 */
static void Mis2dh_taskFxn(UArg a0, UArg a1) {
    Mis2dh_init();

    for (;;) {

    }
}
