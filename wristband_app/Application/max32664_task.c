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

#define MAX32664_I2C_ADDRESS        0xAA
#define MAX32664_BUFFER_SIZE        32
#define MAX32664_I2C_CMD_DELAY      6

// Family names
#define MAX32664_READ_OUTPUT_MODE    0x11



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
static uint8_t txBuffer[MAX32664_BUFFER_SIZE];
static uint8_t rxBuffer[MAX32664_BUFFER_SIZE];
static int rxIndex;

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

static uint8_t Max32664_readOutputMode(void);

static void Max32664_i2cInit(void);
static void Max32664_i2cBeginTransmission(uint8_t address);
static void Max32664_i2cEndTransmission(void);
static void Max32664_i2cReadRequest(int num_bytes);
static void Max32664_i2cWrite(uint8_t data);
static uint8_t Max32664_i2cRead(void);
static bool Max32664_i2cAvailable(void);


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

    uint8_t outputMode = Max32664_readOutputMode();
    Log_info1("MAX32664 Output Mode: %d", outputMode);
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
 * @fn      Max32664_readOutputMode
 *
 * @brief   Read current output mode of the Biometric Sensor Hub.
 */
static uint8_t Max32664_readOutputMode(void)
{
    uint8_t familyByte = MAX32664_READ_OUTPUT_MODE;
    uint8_t indexByte = 0x00;
    uint8_t ret = 0xFF;

    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cWrite(familyByte);
    Max32664_i2cWrite(indexByte);
    Max32664_i2cEndTransmission();

    Task_sleep(MAX32664_I2C_CMD_DELAY * (1000 / Clock_tickPeriod));

    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cReadRequest(1);
    Max32664_i2cEndTransmission();

    if (Max32664_i2cAvailable()) {
        ret = Max32664_i2cRead();
    }

    return ret;
}


/*********************************************************************
 * @fn      Max32664_i2cInit
 *
 * @brief   Initialization the master I2C connection with the Biometric Sensor Hub.
 */
static void Max32664_i2cInit(void)
{
    I2C_Params i2cParams;
    rxIndex = 0;

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

    // Read test
    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cReadRequest(1);
    Max32664_i2cEndTransmission();

    while (Max32664_i2cAvailable()) {
        uint8_t data = Max32664_i2cRead();
        Log_info1("Read: %d", data);
    }
}

/*********************************************************************
 * @fn      Max32664_i2cBeginTransmission
 *
 * @brief   Begin I2C data transmission.
 *
 * @param   address - Address of slave device.
 */
static void Max32664_i2cBeginTransmission(uint8_t address)
{
    transaction.writeBuf = txBuffer;
    transaction.readBuf = rxBuffer;
    transaction.readCount = 0;
    transaction.writeCount = 0;
    transaction.slaveAddress = address;
}

/*********************************************************************
 * @fn      Max32664_i2cEndTransmission
 *
 * @brief   End I2C data transmission.
 */
static void Max32664_i2cEndTransmission(void)
{
    if (!I2C_transfer(i2cHandle, &transaction))
    {
        Log_error0("I2C transaction failed");
    }
    else
    {
        Log_info0("I2C transaction successful");
    }

    transaction.writeCount = 0;
}

/*********************************************************************
 * @fn      Max32664_i2cWrite
 *
 * @brief   Write a byte to transmit over I2C.
 */
static void Max32664_i2cWrite(uint8_t data)
{
    txBuffer[transaction.writeCount] = data;
    transaction.writeCount++;
}

/*********************************************************************
 * @fn      Max32664_i2cWrite
 *
 * @brief   Write a byte to transmit over I2C.
 */
static void Max32664_i2cReadRequest(int num_bytes)
{
    transaction.readCount = num_bytes;
}

/*********************************************************************
 * @fn      Max32664_i2cRead
 *
 * @brief   Read a byte to over I2C.
 */
static uint8_t Max32664_i2cRead(void)
{
    uint8_t data = -1;
    if (transaction.readCount > 0) {
        data = rxBuffer[transaction.readCount-1];
        transaction.readCount--;
    }

    return data;
}

/*********************************************************************
 * @fn      Max32664_i2cAvailable
 *
 * @brief   Check if data has been received over I2C.
 */
static bool Max32664_i2cAvailable(void)
{
    return (transaction.readCount > 0);
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
