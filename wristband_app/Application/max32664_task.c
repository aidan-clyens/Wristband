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
#include <max32664_task.h>
#include <project_zero.h>
#include <util.h>

#include <services/emergency_alert_service.h>


/*********************************************************************
 * CONSTANTS
 */
// Task
#define MAX32664_THREAD_STACK_SIZE                      1024
#define MAX32664_TASK_PRIORITY                          1

#define MAX32664_REPORT_PERIOD_MS           5*1000  // Change to 2*40 ms

#define MAX32664_NORMAL_REPORT_ALGORITHM_ONLY_SIZE      20
#define MAX32664_MAX_NUM_SAMPLES            4

// Heart rate constants
#define MAX32664_LOW_HEARTRATE              70
#define MAX32664_HIGH_HEARTRATE             150

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
// MAX32664 Operating mode
typedef enum {
    RESET_MODE,
    HEARTRATE_MODE,
    ECG_MODE
} max32664_operating_mode_t;

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
    uint8_t heartRateConfidence;
    uint16_t spO2;
    uint8_t spO2Confidence;
    uint8_t scdState;
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

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Bio Sensor Hub data buffer
static uint8_t reportBuffer[MAX32664_MAX_NUM_SAMPLES * MAX32664_NORMAL_REPORT_ALGORITHM_ONLY_SIZE];

static max32664_operating_mode_t operatingMode;

// I2C
static I2C_Handle i2cHandle;
static I2C_Transaction transaction;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
// Task functions
static void Max32664_init(void);
static void Max32664_taskFxn(UArg a0, UArg a1);

static void Max32664_heartRateSwiFxn(UArg a0);
static void Max32664_processApplicationMessage(max32664_msg_t *pMsg);

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
static status_t Max32664_readFifoData(uint8_t *data, int num_bytes);

// MAX32664 helper functions
static status_t Max32664_readByte(uint8_t family, uint8_t index, uint8_t *data);
static status_t Max32664_writeByte(uint8_t family, uint8_t index, uint8_t data, int delay);

// I2C
static bool Max32664_i2cInit(void);


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

    // Initialize I2C
    if (!Max32664_i2cInit()) {
        Log_error0("Startup failed. Exiting");
        Task_exit();
    }

    // Create semaphores
    Semaphore_Params semParamsHeartRate;
    Semaphore_Params_init(&semParamsHeartRate);
    heartRateSemaphore = Semaphore_create(0, &semParamsHeartRate, Error_IGNORE);

    // Create message queue
    Queue_construct(&appMsgQueue, NULL);
    appMsgQueueHandle = Queue_handle(&appMsgQueue);

    // Create clocks
    heartrateClockHandle = Util_constructClock(&heartrateClock,
                                               Max32664_heartRateSwiFxn,
                                               MAX32664_REPORT_PERIOD_MS,
                                               MAX32664_REPORT_PERIOD_MS,
                                               0,
                                               NULL
                                               );

    operatingMode = RESET_MODE;

    // Initialize MAX32664 sensor
    Max32664_enqueueMsg(MAX32664_INIT_HEARTRATE_MODE, NULL);
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
    heartRateData.heartRateConfidence = 0;
    heartRateData.spO2 = 0;
    heartRateData.spO2Confidence = 0;
    heartRateData.scdState = 0;

    uint8_t byte_buffer[1];
    uint8_t word_buffer[2];

    // Application main loop
    for(;;)
    {
        switch (operatingMode) {
            case HEARTRATE_MODE: {
                Semaphore_pend(heartRateSemaphore, BIOS_WAIT_FOREVER);
                if (heartRateAlgorithmInitialized) {
                    // Read heart rate from MAX32664
                    Max32664_readHeartRate(&heartRateData);

                    // Pass messages containing heart rate value to the BLE task
                    word_buffer[0] = heartRateData.heartRate & 0xFF;
                    word_buffer[1] = heartRateData.heartRate >> 8;
                    ProjectZero_valueChangeHandler(DATA_HEARTRATE, word_buffer);

                    byte_buffer[0] = heartRateData.heartRateConfidence;
                    ProjectZero_valueChangeHandler(DATA_HEARTRATE_CONFIDENCE, byte_buffer);

                    word_buffer[0] = heartRateData.spO2 & 0xFF;
                    word_buffer[1] = heartRateData.spO2 >> 8;
                    ProjectZero_valueChangeHandler(DATA_SPO2, word_buffer);

                    byte_buffer[0] = heartRateData.spO2Confidence;
                    ProjectZero_valueChangeHandler(DATA_SPO2_CONFIDENCE, byte_buffer);

                    byte_buffer[0] = heartRateData.scdState;
                    ProjectZero_valueChangeHandler(DATA_SCD_STATE, byte_buffer);

                    Log_info5("Read: Heart Rate: %d - Heart Rate Confidence: %d - SpO2: %d - SpO2 Confidence: %d - SCD state: %d",
                              heartRateData.heartRate,
                              heartRateData.heartRateConfidence,
                              heartRateData.spO2,
                              heartRateData.spO2Confidence,
                              heartRateData.scdState
                    );

                    // Check for low or high heartrate
                    if (heartRateData.heartRate > MAX32664_HIGH_HEARTRATE) {
                        uint8_t alert_type[1];
                        alert_type[0] = ALERT_HIGH_HEARTRATE;
                        Max32664_enqueueMsg(MAX32664_TRIGGER_ALERT, alert_type);
                    }
                    else if (heartRateData.heartRate < MAX32664_LOW_HEARTRATE) {
                        uint8_t alert_type[1];
                        alert_type[0] = ALERT_LOW_HEARTRATE;
                        Max32664_enqueueMsg(MAX32664_TRIGGER_ALERT, alert_type);
                    }
                }

                break;
            }
            default:
                break;
        }

        // Process messages sent from another task or another context.
        while(!Queue_empty(appMsgQueueHandle)) {
            max32664_msg_t *pMsg = (max32664_msg_t *)Util_dequeueMsg(appMsgQueueHandle);
            if(pMsg) {
                Max32664_processApplicationMessage(pMsg);
                // Free the received message.
                ICall_free(pMsg);
            }
        }
    }
}

/*********************************************************************
 * @fn      Max32663_processApplicationMessage
 *
 * @brief   Process application messages.
 *
 * @param   pMsg  Pointer to the message of type max32664_msg_t.
 */
static void Max32664_processApplicationMessage(max32664_msg_t *pMsg) {
    switch(pMsg->event) {
        case MAX32664_INIT_HEARTRATE_MODE: {
            Log_info0("Initializing Heart Rate Operating Mode");
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

            Util_startClock(&heartrateClock);

            operatingMode = HEARTRATE_MODE;
        }
            break;
        case MAX32664_INIT_ECG_MODE:
            Util_stopClock(&heartrateClock);
            operatingMode = ECG_MODE;
            break;
        case MAX32664_TRIGGER_ALERT: {
            uint8_t *alert_type = (uint8_t*)pMsg->pData;

            ProjectZero_triggerEmergencyAlert(*alert_type);
        }
            break;
        default:
            break;
    }

    if(pMsg->pData != NULL) {
        ICall_free(pMsg->pData);
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
    data->heartRateConfidence = 0;
    data->spO2 = 0;
    data->spO2Confidence = 0;
    data->scdState = 0;

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
        Log_error1("Error reading number of FIFO samples: %d", ret);
        return;
    }
    Log_info1("Reading %d samples from FIFO", num_samples);

    uint8_t num_bytes = num_samples * MAX32664_NORMAL_REPORT_ALGORITHM_ONLY_SIZE;

    // TODO: Read the data stored in the FIFO
//    uint8_t buffer[5];
//    ret = Max32664_readFifoData(reportBuffer, num_bytes);
//    if (ret != STATUS_SUCCESS) {
//        Log_error1("Error reading report from FIFO: %d", ret);
//        return;
//    }
//    Log_info1("Read %d samples from FIFO", num_samples);

//    for (int i = 0; i < num_bytes; i++) {
//        if (reportBuffer[i] != 0) Log_info1("%d", reportBuffer[i]);
//    }

    data->heartRate = 120;
    data->heartRateConfidence = 85;
    data->spO2 = 200;
    data->spO2Confidence = 90;
    data->scdState = 1;
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
static status_t Max32664_readFifoData(uint8_t *data, int num_bytes)
{
    status_t ret = STATUS_UNKNOWN_ERROR;

    uint8_t txBuffer[2];

    txBuffer[0] = MAX32664_READ_OUTPUT_FIFO;
    txBuffer[1] = MAX32664_READ_FIFO_DATA;

    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 2;
    transaction.readBuf    = data;
    transaction.readCount  = num_bytes + 1;

    if (I2C_transfer(i2cHandle, &transaction)) {
        Log_info0("I2C transfer successful");
        for (int i = 0; i < num_bytes; i++) {
            Log_info1("%d", data[i]);
        }
        ret = (status_t)data[num_bytes];
    }
    else {
        Log_error0("I2C transfer failed");
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

    uint8_t txBuffer[3];
    uint8_t rxBuffer[1];

    txBuffer[0] = family;
    txBuffer[1] = index;
    txBuffer[2] = data;

    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 3;
    transaction.readBuf    = rxBuffer;
    transaction.readCount  = 1;

    if (I2C_transfer(i2cHandle, &transaction)) {
        Log_info0("I2C transfer successful");
        ret = (status_t)rxBuffer[0];
    }
    else {
        Log_error0("I2C transfer failed");
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
    (*data) = 0;

    uint8_t txBuffer[2];
    uint8_t rxBuffer[2];

    txBuffer[0] = family;
    txBuffer[1] = index;

    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 2;
    transaction.readBuf    = rxBuffer;
    transaction.readCount  = 2;

    if (I2C_transfer(i2cHandle, &transaction)) {
        Log_info0("I2C transfer successful");
        (*data) = rxBuffer[0];
        ret = (status_t)rxBuffer[1];
        Log_info2("data=%d, ret=%d", (*data), ret);
    }
    else {
        Log_error0("I2C transfer failed");
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


/*********************************************************************
 * @fn      Max32664_i2cInit
 *
 * @brief   Initialization the master I2C connection with device.
 */
static bool Max32664_i2cInit(void)
{
    I2C_Params i2cParams;

    I2C_init();

    // Configure I2C
    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.bitRate = I2C_400kHz;
    i2cHandle = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2cHandle == NULL) {
        Log_error0("I2C initialization failed!");
        return false;
    }
    transaction.slaveAddress = MAX32664_ADDRESS;
    Log_info0("I2C initialized");
    return true;
}

/*********************************************************************
 * @fn     Max32664_enqueueMsg
 *
 * @brief  Utility function that sends the event and data to the application.
 *         Handled in the task loop.
 *
 * @param  event    Event type
 * @param  pData    Pointer to message data
 */
bool Max32664_enqueueMsg(max32664_event_t event, void *pData) {
    uint8_t success;
    max32664_msg_t *pMsg = ICall_malloc(sizeof(max32664_msg_t));

    if(pMsg) {
        pMsg->event = event;
        pMsg->pData = pData;

        success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
        return success;
    }

    return false;
}
