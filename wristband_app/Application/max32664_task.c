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
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

#include <uartlog/UartLog.h>

#include <Board.h>

#include <icall.h>
#include <project_zero.h>
#include <util.h>


/*********************************************************************
 * CONSTANTS
 */
#define MAX32664_THREAD_STACK_SIZE      1024
#define MAX32664_TASK_PRIORITY          1

#define MAX32664_HEARTRATE_CLOCK_PERIOD     30*1000

#define MAX32664_I2C_ADDRESS        1


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

// I2C
static I2C_Handle i2cHandle;
static I2C_Transaction transaction;
static uint8_t txBuffer[1];
static uint8_t rxBuffer[2];

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

static void Max32664_i2cInit(void);


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

    // Initialize GPIO
    GPIO_init();

    // Initialize variables
    heartRateValue = 0;

    // Create semaphores
    Semaphore_Params semParamsHeartRate;
    Semaphore_Params_init(&semParamsHeartRate);
    heartRateSemaphore = Semaphore_create(0, &semParamsHeartRate, Error_IGNORE);

    // Initialize I2C connection
    Max32664_i2cInit();

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
        Log_info1("Read Heart Rate: %d", heartRateValue);
    }
}

/*********************************************************************
 * @fn      Max32664_i2cInit
 *
 * @brief   Initialization the master I2C connection with the Biometric Sensor Hub.
 */
static void Max32664_i2cInit(void)
{
    I2C_Params i2cParams;

    I2C_init();

    // Configure I2C
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cHandle = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2cHandle == NULL) {
        Log_error0("I2C initialization failed!");
        Task_exit();
    }
    Log_info0("I2C initialized");

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;
    transaction.readBuf = rxBuffer;
    transaction.readCount = 2;
    transaction.slaveAddress = MAX32664_I2C_ADDRESS;
    if (!I2C_transfer(i2cHandle, &transaction)) {
        Log_error0("MAX32664 sensor not found");
        Task_exit();
    }
    Log_info0("MAX32664 sensor found");
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
