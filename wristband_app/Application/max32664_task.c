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
// Task
#define MAX32664_THREAD_STACK_SIZE      1024
#define MAX32664_TASK_PRIORITY          1

#define MAX32664_HEARTRATE_CLOCK_PERIOD     30*1000

// I2C
#define MAX32664_I2C_ADDRESS        0xAA
#define MAX32664_BUFFER_SIZE        32
#define MAX32664_I2C_CMD_DELAY      6
#define MAX32664_ENABLE_CMD_DELAY   22

// Family names
#define MAX32664_READ_SENSOR_HUB_STATUS     0x00
#define MAX32664_SET_OUTPUT_MODE            0x10
#define MAX32664_READ_OUTPUT_MODE           0x11
#define MAX32664_SENSOR_MODE_ENABLE         0x44
#define MAX32664_ALGORITHM_MODE_ENABLE      0x52

// Sensors
#define MAX32664_MAX86141_ENABLE            0x00


/*********************************************************************
 * TYPEDEFS
 */
// MAX32664 Output Mode
typedef enum {
  OUTPUT_MODE_PAUSE = 0x00,
  OUTPUT_MODE_SENSOR_DATA = 0x01,
  OUTPUT_MODE_ALGORITHM_DATA = 0x02,
  OUTPUT_MODE_SENSOR_AND_ALGORITHM_DATA = 0x03,
  OUTPUT_MODE_PAUSE2 = 0x04,
  OUTPUT_MODE_SAMPLE_COUNTER_BYTE_SENSOR_DATA = 0x05,
  OUTPUT_MODE_SAMPLE_COUNTER_BYTE_ALGORITHM_DATA = 0x06,
  OUTPUT_MODE_SAMPLE_COUNTER_BYTE_SENSOR_AND_ALGORITHM_DATA = 0x07
} output_mode_t;


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
// Task functions
static void Max32664_init(void);
static void Max32664_taskFxn(UArg a0, UArg a1);

static void Max32664_heartRateSwiFxn(UArg a0);

// MAX32664 commands
static void Max32664_initApplicationMode();
static void Max32664_initHeartRateAlgorithm();
static uint8_t Max32664_readSensorHubStatus(void);
static uint8_t Max32664_setOutputMode(uint8_t output_mode);
static uint8_t Max32664_readOutputMode(uint8_t *data);
static uint8_t Max32664_setFifoInterruptThreshold(uint8_t threshold);
static uint8_t Max32664_enableAutoGainControlAlgorithm(uint8_t enable);
static uint8_t Max32664_enableMax86141Sensor(uint8_t enable);

// I2C functions
static void Max32664_i2cInit(void);
static void Max32664_i2cBeginTransmission(uint8_t address);
static bool Max32664_i2cEndTransmission(void);
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
    GPIO_setConfig(Board_GPIO_MAX32664_MFIO, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_GPIO_MAX32664_RESET, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);

    // Initialize variables
    heartRateValue = 0;

    // Create semaphores
    Semaphore_Params semParamsHeartRate;
    Semaphore_Params_init(&semParamsHeartRate);
    heartRateSemaphore = Semaphore_create(0, &semParamsHeartRate, Error_IGNORE);

    // Initialize MAX32664 sensor
    Max32664_initApplicationMode();
    Max32664_i2cInit();
    Max32664_initHeartRateAlgorithm();

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
 * @fn      Max32664_initApplicationMode
 *
 * @brief   Initialize the Biometric Sensor Hub in Application Mode.
 */
static void Max32664_initApplicationMode()
{
    GPIO_write(Board_GPIO_MAX32664_RESET, GPIO_CFG_OUT_LOW);
    GPIO_write(Board_GPIO_MAX32664_MFIO, GPIO_CFG_OUT_HIGH);

    // Delay for 20 ms
    Task_sleep(20 * (1000 / Clock_tickPeriod));

    GPIO_write(Board_GPIO_MAX32664_RESET, GPIO_CFG_OUT_HIGH);
    GPIO_write(Board_GPIO_MAX32664_MFIO, GPIO_CFG_OUT_LOW);

    // Delay for 1 s
    Task_sleep(1100 * (1000 / Clock_tickPeriod));

    Log_info0("Initialized MAX32664 in Application Mode");
}

/*********************************************************************
 * @fn      Max32664_initHeartRateAlgorithm
 *
 * @brief   Initialize the Heart Rate algorithm on the Biometric Sensor Hub.
 */
static void Max32664_initHeartRateAlgorithm()
{
    uint8_t enable;
    uint8_t ret = 0xFF;

    // Set output mode to Algorithm Data
    ret = Max32664_setOutputMode(OUTPUT_MODE_ALGORITHM_DATA);
    if (ret != 0) {
        Log_error0("Error setting output mode");
        return;
    }
    Log_info0("Set output mode to Algorithm Data");

    // Set FIFO threshold for interrupt to be triggered
    uint8_t threshold = 0x01;
    ret = Max32664_setFifoInterruptThreshold(threshold);
    if (ret != 0) {
        Log_error0("Error setting FIFO interrupt threshold");
        return;
    }
    Log_info1("Set FIFO interrupt threshold to: %d", threshold);

    // Enable Automatic Gain Control algorithm
    enable = 0x01;
    ret = Max32664_enableAutoGainControlAlgorithm(enable);
    if (ret != 0) {
        Log_error0("Error enabling Automatic Gain Control algorithm");
        return;
    }
    Log_info0("Enable Automatic Gain Control algorithm");

    // Enable MAX86141 sensor
    enable = 0x01;
    ret = Max32664_enableMax86141Sensor(enable);
    if (ret != 0) {
        Log_error0("Error enabling MAX86141 sensor");
        return;
    }
    Log_info0("Enable MAX86141 sensor");

    // TODO: Fast algo control

    // TODO: Configure number of samples
}


/*********************************************************************
 * @fn      Max32664_readSensorHubStatus
 *
 * @brief   Read current status of the Biometric Sensor Hub.
 */
static uint8_t Max32664_readSensorHubStatus(void)
{
    uint8_t familyByte = MAX32664_READ_SENSOR_HUB_STATUS;
    uint8_t indexByte = 0x00;
    uint8_t ret = 0xFF;
    bool success = false;

    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cWrite(familyByte);
    Max32664_i2cWrite(indexByte);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

    Task_sleep(MAX32664_I2C_CMD_DELAY * (1000 / Clock_tickPeriod));

    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cReadRequest(1);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

    if (Max32664_i2cAvailable()) {
        ret = Max32664_i2cRead();
    }

    return ret;
}

/*********************************************************************
 * @fn      Max32664_setOutputMode
 *
 * @brief   Set output mode of the Biometric Sensor Hub.
 *
 * @param   Output mode
 */
static uint8_t Max32664_setOutputMode(uint8_t output_mode)
{
    uint8_t familyByte = MAX32664_SET_OUTPUT_MODE;
    uint8_t indexByte = 0x00;
    uint8_t ret = 0xFF;
    bool success = false;

    // Update output mode
    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cWrite(familyByte);
    Max32664_i2cWrite(indexByte);
    Max32664_i2cWrite(output_mode);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

    Task_sleep(MAX32664_I2C_CMD_DELAY * (1000 / Clock_tickPeriod));

    // Read status
    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cReadRequest(1);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

    if (Max32664_i2cAvailable()) {
        ret = Max32664_i2cRead();
    }


    return ret;
}


/*********************************************************************
 * @fn      Max32664_readOutputMode
 *
 * @brief   Read current output mode of the Biometric Sensor Hub.
 */
static uint8_t Max32664_readOutputMode(uint8_t *data)
{
    uint8_t familyByte = MAX32664_READ_OUTPUT_MODE;
    uint8_t indexByte = 0x00;
    bool success = false;

    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cWrite(familyByte);
    Max32664_i2cWrite(indexByte);
    success = Max32664_i2cEndTransmission();
    if (!success) return 0xFF;

    Task_sleep(MAX32664_I2C_CMD_DELAY * (1000 / Clock_tickPeriod));

    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cReadRequest(1);
    success = Max32664_i2cEndTransmission();
    if (!success) return 0xFF;

    if (Max32664_i2cAvailable()) {
        (*data) = Max32664_i2cRead();
    }

    return 0x0;
}

/*********************************************************************
 * @fn      Max32664_setFifoInterruptThreshold
 *
 * @brief   Set the number of samples required in the Biometric Sensor Hub FIFO to trigger an interrupt.
 *
 * @param   Interrupt threshold for FIFO (number of samples)
 */
static uint8_t Max32664_setFifoInterruptThreshold(uint8_t threshold)
{
    uint8_t familyByte = MAX32664_SET_OUTPUT_MODE;
    uint8_t indexByte = 0x01;
    uint8_t ret = 0xFF;
    bool success = false;

    // Update FIFO interrupt threshold
    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cWrite(familyByte);
    Max32664_i2cWrite(indexByte);
    Max32664_i2cWrite(threshold);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

    Task_sleep(MAX32664_I2C_CMD_DELAY * (1000 / Clock_tickPeriod));

    // Read status
    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cReadRequest(1);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

    if (Max32664_i2cAvailable()) {
        ret = Max32664_i2cRead();
    }

    return ret;
}

/*********************************************************************
 * @fn      Max32664_enableAutoGainControlAlgorithm
 *
 * @brief   Enable or disable the Automatic Gain Control algorithm on the Biometric Sensor Hub.
 *
 * @param   Enable or Disable
 */
static uint8_t Max32664_enableAutoGainControlAlgorithm(uint8_t enable)
{
    uint8_t familyByte = MAX32664_ALGORITHM_MODE_ENABLE;
    uint8_t indexByte = 0x00;
    uint8_t ret = 0xFF;
    bool success = false;

    // Enable AGC algorithm
    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cWrite(familyByte);
    Max32664_i2cWrite(indexByte);
    Max32664_i2cWrite(enable);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

    // Wait for enable value to update
    Task_sleep(MAX32664_ENABLE_CMD_DELAY * (1000 / Clock_tickPeriod));

    // Read status
    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cReadRequest(1);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

    if (Max32664_i2cAvailable()) {
        ret = Max32664_i2cRead();
    }

    return ret;
}

/*********************************************************************
 * @fn      Max32664_enableMax86141Sensor
 *
 * @brief   Enable or disable the MAX86141 on the Biometric Sensor Hub.
 *
 * @param   Enable or Disable
 */
static uint8_t Max32664_enableMax86141Sensor(uint8_t enable)
{
    uint8_t familyByte = MAX32664_SENSOR_MODE_ENABLE;
    uint8_t indexByte = MAX32664_MAX86141_ENABLE;
    uint8_t ret = 0xFF;
    bool success = false;

    // Enable MAX86141 sensor
    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cWrite(familyByte);
    Max32664_i2cWrite(indexByte);
    Max32664_i2cWrite(enable);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

    // Wait for enable value to update
    Task_sleep(MAX32664_ENABLE_CMD_DELAY * (1000 / Clock_tickPeriod));

    // Read status
    Max32664_i2cBeginTransmission(MAX32664_I2C_ADDRESS);
    Max32664_i2cReadRequest(1);
    success = Max32664_i2cEndTransmission();
    if (!success) return ret;

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
    uint8_t status = Max32664_readSensorHubStatus();
    if (status != 0) {
        Log_error0("MAX32644 not connected");
    }
    else
    {
        Log_info0("MAX32664 connected successfully");
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
static bool Max32664_i2cEndTransmission(void)
{
    bool ret;
    if (!I2C_transfer(i2cHandle, &transaction))
    {
        Log_error0("I2C transaction failed");
        ret = false;
    }
    else
    {
        Log_info0("I2C transaction successful");
        ret = true;
    }

    transaction.writeCount = 0;

    return ret;
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
