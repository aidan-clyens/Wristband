/*
 * max32664_task.c
 *
 *  Created on: Jan 15, 2021
 *      Author: Aidan Clyens
 */

/*******************************************************************************
 * INCLUDES
 */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

#include <icall.h>
#include <project_zero.h>
#include <util.h>


/*********************************************************************
 * CONSTANTS
 */
#define MAX32664_THREAD_STACK_SIZE      1024
#define MAX32664_TASK_PRIORITY          1

#define MAX32664_HEARTRATE_CLOCK_PERIOD     30*1000


/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */
Task_Struct max32664Task;
uint8_t max32664TaskStack[MAX32664_THREAD_STACK_SIZE];

uint16_t heartRateValue;

/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clocks
static Clock_Struct heartrateClock;
static Clock_Handle heartrateClockHandle;

// Semaphores
static Semaphore_Handle heartRateSemaphore;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Max32664_init(void);
static void Max32664_taskFxn(UArg a0, UArg a1);

static void Max32664_heartRateSwiFxn(UArg a0);


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Max32664_createTask
 *
 * @brief   Task creation function for the Max32664 Biometric Sensor Hub.
 */
void Max32664_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = max32664TaskStack;
    taskParams.stackSize = MAX32664_THREAD_STACK_SIZE;
    taskParams.priority = MAX32664_TASK_PRIORITY;

    Task_construct(&max32664Task, Max32664_taskFxn, &taskParams, Error_IGNORE);
}

/*********************************************************************
 * @fn      Max32664_init
 *
 * @brief   Initialization for Max32664 task.
 */
static void Max32664_init(void)
{
    // Register application with ICall
    ICall_registerApp(&selfEntity, &syncEvent);

    // Initialize variables
    heartRateValue = 0;

    // Create semaphore for heart rate value
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    heartRateSemaphore = Semaphore_create(0, &semParams, Error_IGNORE);

    // Create clocks
    heartrateClockHandle = Util_constructClock(&heartrateClock,
                                               Max32664_heartRateSwiFxn,
                                               MAX32664_HEARTRATE_CLOCK_PERIOD,
                                               MAX32664_HEARTRATE_CLOCK_PERIOD,
                                               1,
                                               NULL
                                               );
}

/*********************************************************************
 * @fn      Max32664_taskFxn
 *
 * @brief   Application task entry point for the Max32664 Biometric Sensor Hub.
 *
 * @param   a0, a1 - not used.
 */
static void Max32664_taskFxn(UArg a0, UArg a1)
{
    // Application initialization
    Max32664_init();

    // Application main loop
    for(;;)
    {
        Semaphore_pend(heartRateSemaphore, BIOS_WAIT_FOREVER);
    }
}

/*********************************************************************
 * @fn      Max32664_heartRateSwiFxn
 *
 * @brief   Callback for Heart Rate value update.
 *
 * @param   a0 - not used.
 */
static void Max32664_heartRateSwiFxn(UArg a0) {
    Semaphore_post(heartRateSemaphore);
    heartRateValue++;

    uint8_t data[2];
    data[0] = heartRateValue & 0xFF;
    data[1] = heartRateValue >> 8;
    ProjectZero_valueChangeHandler(DATA_HEARTRATE, data);
}
