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

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/GPIO.h>

#include <uartlog/UartLog.h>
#include <Board.h>
#include <icall.h>
#include <util.h>

#include <sensors_task.h>
#include <i2c_util.h>

/*********************************************************************
 * CONSTANTS
 */
#define SENSORS_THREAD_STACK_SIZE                       1024
#define SENSORS_TASK_PRIORITY                           1

// Clocks
#define SENSORS_ACCELEROMETER_POLLING_PERIOD_MS         200

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

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clocks
static Clock_Struct accelerometerReadClock;
static Clock_Handle accelerometerReadClockHandle;
static bool readAccelerometerFlag;

// Semaphores
static Semaphore_Handle swiSemaphore;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
// Task functions
static void Sensors_init(void);
static void Sensors_taskFxn(UArg a0, UArg a1);
static void Sensors_processApplicationMessage(sensors_msg_t *pMsg);

static void Sensors_accelerometerReadSwiFxn(UArg a0);

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

    // Initialize GPIO pins
    GPIO_setConfig(Board_GPIO_MAX32664_MFIO, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_GPIO_MAX32664_RESET, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);

    // Initialize I2C
    if (!Util_i2cInit()) {
        Log_error0("I2C failed to initialize");
        Task_exit();
    }

    // Create message queue
    Queue_construct(&appMsgQueue, NULL);
    appMsgQueueHandle = Queue_handle(&appMsgQueue);

    // Create semaphores
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    swiSemaphore = Semaphore_create(0, &semParams, Error_IGNORE);

    // Create clocks
    accelerometerReadClockHandle = Util_constructClock(
            &accelerometerReadClock,
            Sensors_accelerometerReadSwiFxn,
            SENSORS_ACCELEROMETER_POLLING_PERIOD_MS,
            SENSORS_ACCELEROMETER_POLLING_PERIOD_MS,
            0,
            NULL
    );

    readAccelerometerFlag = false;

    // Start clocks
    Util_startClock(&accelerometerReadClock);

    Log_info0("Sensors initialized");
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
        Semaphore_pend(swiSemaphore, BIOS_WAIT_FOREVER);
        if (readAccelerometerFlag) {
            Log_info0("Read from Accelerometer");
            readAccelerometerFlag = false;
        }

        // Process messages sent from another task or another context.
        while(!Queue_empty(appMsgQueueHandle)) {
            sensors_msg_t *pMsg = (sensors_msg_t *)Util_dequeueMsg(appMsgQueueHandle);
            if(pMsg) {
                Sensors_processApplicationMessage(pMsg);
                // Free the received message.
                ICall_free(pMsg);
            }
        }
    }
}

/*********************************************************************
 * @fn      Sensors_processApplicationMessage
 *
 * @brief   Process application messages.
 *
 * @param   pMsg  Pointer to the message of type max32664_msg_t.
 */
static void Sensors_processApplicationMessage(sensors_msg_t *pMsg) {
    switch(pMsg->event) {
        case SENSORS_INIT_HEARTRATE_MODE:
            break;
        case SENSORS_INIT_ECG_MODE:
            break;
        case SENSORS_TRIGGER_ALERT:
            break;
        default:
            break;
    }

    if(pMsg->pData != NULL) {
        ICall_free(pMsg->pData);
    }
}

/*********************************************************************
 * @fn      Sensors_accelerometerReadSwiFxn
 *
 * @brief   Callback to read accelerometer samples.
 *
 * @param   a0 - not used.
 */
static void Sensors_accelerometerReadSwiFxn(UArg a0) {
    readAccelerometerFlag = true;
    Semaphore_post(swiSemaphore);
}

/*********************************************************************
 * @fn     Sensors_enqueueMsg
 *
 * @brief  Utility function that sends the event and data to the application.
 *         Handled in the task loop.
 *
 * @param  event    Event type
 * @param  pData    Pointer to message data
 */
bool Sensors_enqueueMsg(sensors_event_t event, void *pData) {
    uint8_t success;
    sensors_msg_t *pMsg = ICall_malloc(sizeof(sensors_msg_t));

    if(pMsg) {
        pMsg->event = event;
        pMsg->pData = pData;

        success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
        return success;
    }

    return false;
}
