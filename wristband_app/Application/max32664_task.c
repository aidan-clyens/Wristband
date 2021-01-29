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
#include <i2c_util.h>


/*********************************************************************
 * CONSTANTS
 */
// Task
#define MAX32664_THREAD_STACK_SIZE                      1024
#define MAX32664_TASK_PRIORITY                          1

#define MAX32664_REPORT_PERIOD_MS           5*1000  // Change to 2*40 ms

#define MAX32664_NORMAL_REPORT_ALGORITHM_ONLY_SIZE      20
#define MAX32664_MAX_NUM_SAMPLES            4

// I2C
#define MAX32664_ADDRESS                0xAA
#define MAX32664_CMD_DELAY              6
#define MAX32664_ENABLE_CMD_DELAY           22

// Family names
#define MAX32664_READ_SENSOR_HUB_STATUS     0x00
#define MAX32664_SET_OUTPUT_MODE            0x10
#define MAX32664_READ_OUTPUT_MODE           0x11
#define MAX32664_READ_OUTPUT_FIFO           0x12
#define MAX32664_SENSOR_MODE_ENABLE         0x44
#define MAX32664_ALGORITHM_MODE_ENABLE      0x52

// Read Output
#define MAX32664_NUM_FIFO_SAMPLES           0x00
#define MAX32664_READ_FIFO_DATA             0x01

// Sensors
#define MAX32664_MAX86141_ENABLE            0x00

// Algorithms
#define MAX32664_AGC_ALGORITHM              0x00
#define MAX32664_WHRM_WSPO2_ALGORITHM       0x07

// Modes
#define MAX32664_NORMAL_REPORT_MODE         0x01
#define MAX32664_EXTENDED_REPORT_MODE       0x02


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

// Status Byte Value
typedef enum {
  STATUS_SUCCESS = 0x00,
  STATUS_ILLEGAL_FAMILY = 0x01,
  STATUS_NOT_IMPLEMENTED = 0x02,
  STATUS_INCORRECT_NUM_BYTES = 0x03,
  STATUS_ILLEGAL_CONFIG = 0x04,
  STATUS_INCORRECT_MODE = 0x05,
  STATUS_ERROR_FLASHING = 0x80,
  STATUS_CHECKSUM_ERROR = 0x81,
  STATUS_AUTH_ERROR = 0x82,
  STATUS_APPLICATION_INVALID = 0x83,
  STATUS_DEVICE_BUSY = 0xFE,
  STATUS_UNKNOWN_ERROR = 0xFF
} status_t;

// Heart Rate Data
typedef struct {
    uint16_t heartRate;
    uint16_t spO2;
    uint8_t confidence;
    uint8_t status;
} heartrate_data_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */
Task_Struct max32664Task;
uint8_t max32664TaskStack[MAX32664_THREAD_STACK_SIZE];

bool heartRateAlgorithmInitialized = false;


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

static uint8_t reportBuffer[MAX32664_MAX_NUM_SAMPLES * MAX32664_NORMAL_REPORT_ALGORITHM_ONLY_SIZE];


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
static void Max32664_readHeartRate(heartrate_data_t *data);

static status_t Max32664_readSensorHubStatus(uint8_t *status);
static status_t Max32664_setOutputMode(uint8_t output_mode);
static status_t Max32664_readOutputMode(uint8_t *data);
static status_t Max32664_setFifoInterruptThreshold(uint8_t threshold);
static status_t Max32664_enableAutoGainControlAlgorithm(uint8_t enable);
static status_t Max32664_enableWhrmWspo2Algorithm(uint8_t enableMode);
static status_t Max32664_enableMax86141Sensor(uint8_t enable);
static status_t Max32664_readFifoNumSamples(uint8_t *num_samples);
static status_t Max32664_readFifoData(int num_bytes);

// MAX32664 helper functions
static status_t Max32664_readByte(uint8_t family, uint8_t index, uint8_t *data);
static status_t Max32664_writeByte(uint8_t family, uint8_t index, uint8_t data, int delay);


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

    // Initialize GPIO pins
    GPIO_setConfig(Board_GPIO_MAX32664_MFIO, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_GPIO_MAX32664_RESET, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);

    // Create semaphores
    Semaphore_Params semParamsHeartRate;
    Semaphore_Params_init(&semParamsHeartRate);
    heartRateSemaphore = Semaphore_create(0, &semParamsHeartRate, Error_IGNORE);

    // Initialize MAX32664 sensor
    Max32664_initApplicationMode();

    // Read test
    uint8_t sensor_status;
    status_t ret = Max32664_readSensorHubStatus(&sensor_status);
    if (ret != STATUS_SUCCESS || sensor_status == 1) {
        Log_error0("MAX32644 not connected");
    }
    else
    {
        Log_info0("MAX32664 connected successfully");
    }

    Max32664_initHeartRateAlgorithm();

    // Create clocks
    heartrateClockHandle = Util_constructClock(&heartrateClock,
                                               Max32664_heartRateSwiFxn,
                                               MAX32664_REPORT_PERIOD_MS,
                                               MAX32664_REPORT_PERIOD_MS,
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

    heartrate_data_t heartRateData;
    heartRateData.heartRate = 0;
    heartRateData.spO2 = 0;
    heartRateData.confidence = 0;
    heartRateData.status = 0;

    // Application main loop
    for(;;)
    {
        Semaphore_pend(heartRateSemaphore, BIOS_WAIT_FOREVER);
        if (heartRateAlgorithmInitialized) {
            // Read heart rate from MAX32664
            Max32664_readHeartRate(&heartRateData);

            // Pass message containing heart rate value to the BLE task
            uint8_t data[2];
            data[0] = heartRateData.heartRate & 0xFF;
            data[1] = heartRateData.heartRate >> 8;
            ProjectZero_valueChangeHandler(DATA_HEARTRATE, data);

            Log_info1("Read Heart Rate: %d", heartRateData.heartRate);
        }
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
    uint8_t mode;
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

    // TODO: Enable host accelerometer, if used

    // TODO: Double check algorithm config

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

    // Enable WHRM + WSpO2 algorithm (version C)
    // Mode 1: Normal Report
    // Mode 2: Extended Report
    mode = MAX32664_NORMAL_REPORT_MODE;
    ret = Max32664_enableWhrmWspo2Algorithm(mode);
    if (ret != 0) {
        Log_error0("Error enabling WHRM + WSpo2 (version C) algorithm");
        return;
    }
    Log_info1("Enable WHRM + WSpo2 (version C) algorithm in mode: %d", mode);

    heartRateAlgorithmInitialized = true;
    Log_info0("MAX32664 Heart Rate Algorithm configured");
}

/*********************************************************************
 * @fn      Max32664_readHeartRate
 *
 * @brief   Read heart rate value from Biometric Sensor Hub.
 *
 * @param   Heart rate, SpO2, confidence and status values
 */
static void Max32664_readHeartRate(heartrate_data_t *data)
{
    status_t ret;

    // Initialize values
    data->heartRate = 0;
    data->spO2 = 0;
    data->confidence = 0;
    data->status = 0;

    // Check sensor status before reading FIFO data
    uint8_t sensor_status;
    ret = Max32664_readSensorHubStatus(&sensor_status);
    if (ret != STATUS_SUCCESS || sensor_status == 1) {
        Log_error0("Error communicating with Sensor Hub, cannot read Heart Rate data");
        return;
    }
    Log_info0("Reading Heart Rate data from Sensor Hub");
    // TODO: Check if bit 3 (FIFO filled to threshold) is set

    // Get the number of samples in the FIFO
    uint8_t num_samples = 0;
    ret = Max32664_readFifoNumSamples(&num_samples);
    if (ret != STATUS_SUCCESS) {
        Log_error0("Error reading number of FIFO samples");
        return;
    }
    Log_info1("Reading %d samples from FIFO", num_samples);

    uint8_t num_bytes = num_samples * MAX32664_NORMAL_REPORT_ALGORITHM_ONLY_SIZE;

    // TODO: Read the data stored in the FIFO
    ret = Max32664_readFifoData(num_bytes);
    if (ret != STATUS_SUCCESS) {
        Log_error0("Error reading report from FIFO");
        return;
    }
    Log_info1("Read %d samples from FIFO", num_samples);

//    for (int i = 0; i < num_bytes; i++) {
//        if (reportBuffer[i] != 0) Log_info1("%d", reportBuffer[i]);
//    }

    data->heartRate++;
}


/*********************************************************************
 * @fn      Max32664_readSensorHubStatus
 *
 * @brief   Read current status of the Biometric Sensor Hub.
 */
static status_t Max32664_readSensorHubStatus(uint8_t *status)
{
    return Max32664_readByte(MAX32664_READ_SENSOR_HUB_STATUS, 0x00, status);
}

/*********************************************************************
 * @fn      Max32664_setOutputMode
 *
 * @brief   Set output mode of the Biometric Sensor Hub.
 *
 * @param   Output mode
 */
static status_t Max32664_setOutputMode(uint8_t output_mode)
{
    return Max32664_writeByte(MAX32664_SET_OUTPUT_MODE, 0x00, output_mode, MAX32664_CMD_DELAY);
}


/*********************************************************************
 * @fn      Max32664_readOutputMode
 *
 * @brief   Read current output mode of the Biometric Sensor Hub.
 */
static status_t Max32664_readOutputMode(uint8_t *data)
{
    return Max32664_readByte(MAX32664_READ_OUTPUT_MODE, 0x00, data);
}

/*********************************************************************
 * @fn      Max32664_setFifoInterruptThreshold
 *
 * @brief   Set the number of samples required in the Biometric Sensor Hub FIFO to trigger an interrupt.
 *
 * @param   Interrupt threshold for FIFO (number of samples)
 */
static status_t Max32664_setFifoInterruptThreshold(uint8_t threshold)
{
    return Max32664_writeByte(MAX32664_SET_OUTPUT_MODE, 0x01, threshold, MAX32664_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_enableAutoGainControlAlgorithm
 *
 * @brief   Enable or disable the Automatic Gain Control algorithm on the Biometric Sensor Hub.
 *
 * @param   Enable or Disable
 */
static status_t Max32664_enableAutoGainControlAlgorithm(uint8_t enable)
{
    return Max32664_writeByte(MAX32664_ALGORITHM_MODE_ENABLE, MAX32664_AGC_ALGORITHM, enable, MAX32664_ENABLE_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_enableWhrmWspo2Algorithm
 *
 * @brief   Enable or disable the WHRM + WSpO2 algorithm on the Biometric Sensor Hub.
 *
 * @param   Enable Mode or Disable
 */
static status_t Max32664_enableWhrmWspo2Algorithm(uint8_t enableMode)
{
    return Max32664_writeByte(MAX32664_ALGORITHM_MODE_ENABLE, MAX32664_WHRM_WSPO2_ALGORITHM, enableMode, 130);
}

/*********************************************************************
 * @fn      Max32664_enableMax86141Sensor
 *
 * @brief   Enable or disable the MAX86141 on the Biometric Sensor Hub.
 *
 * @param   Enable or Disable
 */
static status_t Max32664_enableMax86141Sensor(uint8_t enable)
{
    return Max32664_writeByte(MAX32664_SENSOR_MODE_ENABLE, MAX32664_MAX86141_ENABLE, enable, MAX32664_ENABLE_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_readFifoNumSamples
 *
 * @brief   Get the number of samples in the Biometric Sensor Hub FIFO.
 *
 * @param   Number of samples in FIFO
 */
static status_t Max32664_readFifoNumSamples(uint8_t *num_samples)
{
    return Max32664_readByte(MAX32664_READ_OUTPUT_FIFO, MAX32664_NUM_FIFO_SAMPLES, num_samples);
}

/*********************************************************************
 * @fn      Max32664_readFifoData
 *
 * @brief   Read the samples from the FIFO in the Biometric Sensor Hub FIFO.
 *
 * @param   data - Buffer to store FIFO data
 * @param   num_bytes - Number of bytes to read from the FIFO
 */
static status_t Max32664_readFifoData(int num_bytes)
{
    uint8_t familyByte = MAX32664_READ_OUTPUT_FIFO;
    uint8_t indexByte = MAX32664_READ_FIFO_DATA;
    status_t ret = STATUS_UNKNOWN_ERROR;
    bool success = false;

    Util_i2cBeginTransmission(MAX32664_ADDRESS);
    Util_i2cWrite(familyByte);
    Util_i2cWrite(indexByte);
    success = Util_i2cEndTransmission();
    if (!success) return ret;

    Task_sleep(MAX32664_CMD_DELAY * (1000 / Clock_tickPeriod));

    Util_i2cBeginTransmission(MAX32664_ADDRESS);
    Util_i2cReadRequest(num_bytes + 1);
    success = Util_i2cEndTransmission();
    if (!success) return ret;

    // Read value bytes
    int i = 0;
    Log_info1("Bytes: %d", Util_i2cAvailable());
    while (Util_i2cAvailable() > 1) {
        reportBuffer[i] = Util_i2cRead();
        i++;
    }

    // Read status byte
    if (Util_i2cAvailable()) {
        ret = (status_t)Util_i2cRead();
    }

    return ret;
}


/*********************************************************************
 * @fn      Max32664_writeByte
 *
 * @brief   Write a byte to the Biometric Sensor Hub.
 *
 * @param   family  -   Family register byte
 * @param   index   -   Index byte of family
 * @param   data    -   Data to write
 * @param   delay   -   Delay in ms to wait in between write and read
 */
static status_t Max32664_writeByte(uint8_t family, uint8_t index, uint8_t data, int delay)
{
    status_t ret = STATUS_UNKNOWN_ERROR;
    bool success = false;

    // Write value
    Util_i2cBeginTransmission(MAX32664_ADDRESS);
    Util_i2cWrite(family);
    Util_i2cWrite(index);
    Util_i2cWrite(data);
    success = Util_i2cEndTransmission();
    if (!success) return ret;

    // Wait to value to update on slave
    Task_sleep(delay * (1000 / Clock_tickPeriod));

    // Read status
    Util_i2cBeginTransmission(MAX32664_ADDRESS);
    Util_i2cReadRequest(1);
    success = Util_i2cEndTransmission();
    if (!success) return ret;

    // Read status byte
    if (Util_i2cAvailable()) {
        ret = (status_t)Util_i2cRead();
    }

    return ret;
}

/*********************************************************************
 * @fn      Max32664_readByte
 *
 * @brief   Read a byte from the Biometric Sensor Hub.
 *
 * @param   family  -   Family register byte
 * @param   index   -   Index byte of family
 * @param   data    -   Data to read
 */
static status_t Max32664_readByte(uint8_t family, uint8_t index, uint8_t *data)
{
    status_t ret = STATUS_UNKNOWN_ERROR;
    bool success = false;
    (*data) = 0;

    Util_i2cBeginTransmission(MAX32664_ADDRESS);
    Util_i2cWrite(family);
    Util_i2cWrite(index);
    success = Util_i2cEndTransmission();
    if (!success) return ret;

    Task_sleep(MAX32664_CMD_DELAY * (1000 / Clock_tickPeriod));

    Util_i2cBeginTransmission(MAX32664_ADDRESS);
    Util_i2cReadRequest(2);
    success = Util_i2cEndTransmission();
    if (!success) return ret;

    // Read value byte
    if (Util_i2cAvailable()) {
        (*data) = Util_i2cRead();
    }

    // Read status byte
    if (Util_i2cAvailable()) {
        ret = (status_t)Util_i2cRead();
    }

    return ret;
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
}
