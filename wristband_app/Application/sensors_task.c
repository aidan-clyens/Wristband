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
#include <lis3dh.h>
#include <max32664.h>
#include <project_zero.h>

/*********************************************************************
 * CONSTANTS
 */
#define SENSORS_THREAD_STACK_SIZE                       1024
#define SENSORS_TASK_PRIORITY                           1

// Clocks
#define SENSORS_ACCELEROMETER_POLLING_PERIOD_MS         200    // Change to 200 ms

// Data
#define SENSORS_NUM_ACCELEROMETER_SAMPLES               5
#define SENSORS_ACCELEROMETER_SAMPLE_SIZE               6

/*********************************************************************
 * TYPEDEFS
 */
typedef enum {
    STATE_UNINITIALIZED,
    STATE_RUNNING
} sensors_task_state_t;

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

static bool readHeartRateFlag;

static bool freeFallInterruptFlag;

// Semaphores
static Semaphore_Handle swiSemaphore;

static uint8_t accelerometerSamplesBytes[SENSORS_NUM_ACCELEROMETER_SAMPLES * SENSORS_ACCELEROMETER_SAMPLE_SIZE];

static sensors_task_state_t sensorsTaskState;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
// Task functions
static void Sensors_init(void);
static void Sensors_taskFxn(UArg a0, UArg a1);
static void Sensors_processApplicationMessage(sensors_msg_t *pMsg);

static bool Sensors_initDevices();
static void Sensors_updateHeartRateData(heartrate_data_t heartRateData);
static void Sensors_sendAccelerometerSamples(sensor_data_t samples[]);

static void Sensors_accelerometerReadSwiFxn(UArg a0);
static void Sensors_heartRateReadSwiFxn(UArg a0);
static void Sensors_freeFallSwiFxn(UArg a0);

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
    readHeartRateFlag = false;
    freeFallInterruptFlag = false;

    sensorsTaskState = STATE_UNINITIALIZED;
    if (Sensors_initDevices()) {
        sensorsTaskState = STATE_RUNNING;
    }
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

    max32664_status_t ret;
    uint8_t max32664Status;

    uint8_t numReports;
    int numAccelerometerSamples;

    sensor_data_t accelerometerSamples[SENSORS_NUM_ACCELEROMETER_SAMPLES];
    heartrate_data_t reports[32];

    for (;;) {
        switch (sensorsTaskState) {
            // Running State: Read 5 accelerometer samples every 200 ms and a heart rate report every 40 ms
            case STATE_RUNNING: {
                Semaphore_pend(swiSemaphore, BIOS_WAIT_FOREVER);
                // Deal with free-fall interrupts
                if (freeFallInterruptFlag) {
                    Log_info0("Free-fall detected, triggering alert");

                    // Trigger emergency alert
                    ProjectZero_triggerEmergencyAlert(1);

                    if (!Lis3dh_clearInterrupts()) {
                        Log_error0("Failed to clear free-fall interrupt");
                    }
                    freeFallInterruptFlag = false;
                }

                // Read accelerometer data and send to Biometric Sensor Hub
                if (readAccelerometerFlag) {
                    if(!Lis3dh_getNumUnreadSamples(&numAccelerometerSamples)) {
                        Log_error0("Error getting unread number of accelerometer samples");
                        sensorsTaskState = STATE_UNINITIALIZED;
                    }
                    if (numAccelerometerSamples >= SENSORS_NUM_ACCELEROMETER_SAMPLES) {
                        Log_info1("Accelerometer Samples: %d", numAccelerometerSamples);
                        // Read samples from accelerometer
                        if (!Lis3dh_readSensorData(accelerometerSamples, SENSORS_NUM_ACCELEROMETER_SAMPLES)) {
                            Log_error0("Error reading accelerometer samples");
                            sensorsTaskState = STATE_UNINITIALIZED;
                        }
                        else {
//                            Log_info1("Writing %d accelerometer samples to the MAX32664", SENSORS_NUM_ACCELEROMETER_SAMPLES);
                            // Write samples to MAX32664
//                            Sensors_sendAccelerometerSamples(accelerometerSamples);
                        }
                    }

                    readAccelerometerFlag = false;
                }

                // Read heart rate and SpO2 data
                if (readHeartRateFlag) {
                    // Check MAX32664 status
                    ret = Max32664_readSensorHubStatus(&max32664Status);
                    if (ret == STATUS_SUCCESS) {
                        // Get number of available FIFO reports
                        ret = Max32664_readFifoNumSamples(&numReports);
                        Log_info1("Heart Rate Reports: %d", numReports);
                        if (ret == STATUS_SUCCESS) {
                            // Read reports from MAX32664
                            ret = Max32664_readHeartRate(reports, MAX32664_FIFO_THRESHOLD);
                            if (ret == STATUS_SUCCESS) {
                                int maxHeartRateConfidence = 0;
                                int bestReportIndex = -1;
                                // Find the most accurate heart rate reading
                                for (int i = 0; i < MAX32664_FIFO_THRESHOLD; i++) {
                                    if (reports[i].scdState == 3 && reports[i].heartRateConfidence != 0) {
                                        if (reports[i].heartRateConfidence > maxHeartRateConfidence) {
                                            maxHeartRateConfidence = reports[i].heartRateConfidence;
                                            bestReportIndex = i;
                                        }
                                    }
                                }

                                // Update heart rate data using the most accurate reading
                                if (bestReportIndex > 0) {
                                    if (reports[bestReportIndex].scdState == 3 && reports[bestReportIndex].heartRateConfidence != 0) {
                                        Log_info2("HR: %d, HR confidence: %d", reports[bestReportIndex].heartRate, reports[bestReportIndex].heartRateConfidence);
                                        Log_info2("SpO2: %d, SpO2 confidence: %d", reports[bestReportIndex].spO2, reports[bestReportIndex].spO2Confidence);
                                        Log_info1("SCD state: %d", reports[bestReportIndex].scdState);

                                        // Queue report to be processed
                                        heartrate_data_t heartRateData = reports[bestReportIndex];
                                        Sensors_updateHeartRateData(heartRateData);
                                    }
                                }
                            }
                        }
                    }

                    // If any errors occurred, reinitialize sensors
                    if (ret != STATUS_SUCCESS) {
                        sensorsTaskState = STATE_UNINITIALIZED;
                    }

                    readHeartRateFlag = false;
                }

                break;
            }
            // Uninitialized State: An error caused the sensors to be re-initialized. Attempt to reconnect with the sensors until successful
            case STATE_UNINITIALIZED: {
                Sensors_enqueueMsg(SENSORS_INIT_HEARTRATE_MODE, NULL);

                // Delay for 100 ms
                Task_sleep(100 * (1000 / Clock_tickPeriod));

                break;
            }
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
            // Stop clocks
            Util_stopClock(&accelerometerReadClock);

            if (Sensors_initDevices()) {
                sensorsTaskState = STATE_RUNNING;
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
 * @fn      Sensors_initDevices
 *
 * @brief   Initialize MAX32664 and LIS3DH devices.
 */
static bool Sensors_initDevices() {
    // Start MAX32664 in Application Mode
    max32664_status_t ret;
    Log_info0("Initializing MAX32664 in Application Mode");
    ret = Max32664_initApplicationMode(Sensors_heartRateReadSwiFxn);
    if (ret != STATUS_SUCCESS) {
        Log_error1("Failed to initialize MAX32664 in Application Mode (Error: %d)", ret);
        return false;
    }

    // Initialize LIS3DH accelerometer
    Log_info0("Initializing LIS3DH");
    if (!Lis3dh_init(Sensors_freeFallSwiFxn)) {
        Log_error0("Failed to initialize LIS3DH");
        return false;
    }

    // Start MAX32664 Heart Rate Algorithm
    Log_info0("Initializing MAX32664 Heart Rate Algorithm");
    ret = Max32664_initMaximFastAlgorithm();
    if (ret != STATUS_SUCCESS) {
        Log_error1("Error initializing MAX32664 Heart Rate Algorithm (Error: %d)", ret);
        return false;
    }

    // Start clocks
    Util_startClock(&accelerometerReadClock);

    Log_info0("Sensors initialized");

    return true;
}

/*********************************************************************
 * @fn      Sensors_updateHeartRateData
 *
 * @brief   Update heart rate and SpO2 BLE characteristics.
 *
 * @param   heartRateData - Data containing new heart rate and SpO2 values.
 */
static void Sensors_updateHeartRateData(heartrate_data_t heartRateData) {
    uint8_t heartRate[2];
    heartRate[1] = heartRateData.heartRate >> 8;
    heartRate[0] = heartRateData.heartRate & 0xFF;

    uint8_t spO2[2];
    spO2[1] = heartRateData.spO2 >> 8;
    spO2[0] = heartRateData.spO2 & 0xFF;

    ProjectZero_valueChangeHandler(DATA_HEARTRATE, heartRate);
    ProjectZero_valueChangeHandler(DATA_HEARTRATE_CONFIDENCE, &heartRateData.heartRateConfidence);
    ProjectZero_valueChangeHandler(DATA_SPO2, spO2);
    ProjectZero_valueChangeHandler(DATA_SPO2_CONFIDENCE, &heartRateData.spO2Confidence);
    ProjectZero_valueChangeHandler(DATA_SCD_STATE, &heartRateData.scdState);
}

static void Sensors_sendAccelerometerSamples(sensor_data_t samples[]) {
    int index = 0;
    for (int i = 0; i < SENSORS_NUM_ACCELEROMETER_SAMPLES; i++) {
        accelerometerSamplesBytes[index] = samples[i].x_L;
        accelerometerSamplesBytes[index+1] = samples[i].x_H;
        accelerometerSamplesBytes[index+2] = samples[i].y_L;
        accelerometerSamplesBytes[index+3] = samples[i].y_H;
        accelerometerSamplesBytes[index+4] = samples[i].z_L;
        accelerometerSamplesBytes[index+5] = samples[i].z_H;

        index += SENSORS_ACCELEROMETER_SAMPLE_SIZE;
    }

    max32664_status_t ret = Max32664_writeInputFifo(accelerometerSamplesBytes, SENSORS_NUM_ACCELEROMETER_SAMPLES * SENSORS_ACCELEROMETER_SAMPLE_SIZE);
    if (ret != STATUS_SUCCESS) {
        Log_error1("Error sending accelerometer samples to MAX32664 (Error: %d)", ret);
    }
    else {
        Log_info1("Sent %d accelerometer samples to MAX32664", SENSORS_NUM_ACCELEROMETER_SAMPLES);
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
 * @fn      Sensors_heartRateReadSwiFxn
 *
 * @brief   Callback to read heart rate reports.
 *
 * @param   a0 - not used.
 */
static void Sensors_heartRateReadSwiFxn(UArg a0) {
    readHeartRateFlag = true;
    Semaphore_post(swiSemaphore);
}

/*********************************************************************
 * @fn      Sensors_freeFallSwiFxn
 *
 * @brief   Callback for free-fall interrupts.
 *
 * @param   a0 - not used.
 */
static void Sensors_freeFallSwiFxn(UArg a0) {
    freeFallInterruptFlag = true;
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
