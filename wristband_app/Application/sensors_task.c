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
#include <max32664.h>
#include <mis2dh_task.h>
#include <project_zero.h>

/*********************************************************************
 * CONSTANTS
 */
#define SENSORS_THREAD_STACK_SIZE                       1024
#define SENSORS_TASK_PRIORITY                           1

// Clocks
#define SENSORS_ACCELEROMETER_POLLING_PERIOD_MS         20000     // Change to 200 ms
#define SENSORS_HEART_RATE_POLLING_PERIOD_MS            4000      // Change to 40 ms

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

static Clock_Struct heartRateReadClock;
static Clock_Handle heartRateReadClockHandle;
static bool readHeartRateFlag;

// Semaphores
static Semaphore_Handle swiSemaphore;

static uint8_t accelerometerSamplesBytes[SENSORS_NUM_ACCELEROMETER_SAMPLES * SENSORS_ACCELEROMETER_SAMPLE_SIZE + 2];

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

    heartRateReadClockHandle = Util_constructClock(
            &heartRateReadClock,
            Sensors_heartRateReadSwiFxn,
            SENSORS_HEART_RATE_POLLING_PERIOD_MS,
            SENSORS_HEART_RATE_POLLING_PERIOD_MS,
            0,
            NULL
    );
    readHeartRateFlag = false;

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

    int num_reports;
    sensor_data_t accelerometerSamples[5];
    heartrate_data_t reports[32];

    for (;;) {
        switch (sensorsTaskState) {
            case STATE_RUNNING: {
                Semaphore_pend(swiSemaphore, BIOS_WAIT_FOREVER);
                if (readAccelerometerFlag) {
                    Log_info0("Read from Accelerometer");

                    // Read 5 samples from accelerometer
                    for (int i = 0; i < SENSORS_NUM_ACCELEROMETER_SAMPLES; i++) {
                        sensor_data_t sensorData;
                        sensorData.x_L = 1;
                        sensorData.x_H = 2;
                        sensorData.y_L = 3;
                        sensorData.y_H = 4;
                        sensorData.z_L = 5;
                        sensorData.z_H = 6;
        //                Mis2dh_readSensorData(&sensorData);
                        accelerometerSamples[i] = sensorData;
                    }

                    // Write samples to MAX32664
                    Sensors_sendAccelerometerSamples(accelerometerSamples);

                    readAccelerometerFlag = false;
                }

                if (readHeartRateFlag) {
                    Log_info0("Read Heart Rate");

                    // Read report from MAX32664
                    if (Max32664_readHeartRate(reports, &num_reports)) {
                        Log_info1("Read %d reports", num_reports);
                        for (int i = 0; i < num_reports; i++) {
                            Log_info2("HR: %d, HR confidence: %d", reports[i].heartRate, reports[i].heartRateConfidence);
                            Log_info2("SpO2: %d, SpO2 confidence: %d", reports[i].spO2, reports[i].spO2Confidence);
                            Log_info1("SCD state: %d", reports[i].scdState);

                            // Queue report to be processed
                            heartrate_data_t heartRateData = reports[i];
                            Sensors_updateHeartRateData(heartRateData);
                        }
                    }
                    else {
                        sensorsTaskState = STATE_UNINITIALIZED;
                    }

                    readHeartRateFlag = false;
                }

                break;
            }
            case STATE_UNINITIALIZED: {
                if (Sensors_initDevices()) {
                    sensorsTaskState = STATE_RUNNING;
                }

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
            Util_stopClock(&heartRateReadClock);

            if (Sensors_initDevices()) {
                sensorsTaskState = STATE_RUNNING;
            }
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

static bool Sensors_initDevices() {
    // Start MAX32664 in Application Mode
    Log_info0("Initializing MAX32664 in Application Mode");
    if (!Max32664_initApplicationMode()) {
        Log_error0("Failed to initialize MAX32664 in Application Mode");
        return false;
    }

    // Initialize MIS2DH accelerometer
    Log_info0("Initializing MIS2DH");
    if (!Mis2dh_init()) {
        Log_error0("Failed to initialize MIS2DH");
        return false;
    }

    // Start MAX32664 Heart Rate Algorithm
    Log_info0("Initializing MAX32664 Heart Rate Algorithm");
    if (!Max32664_initHeartRateAlgorithm()) {
        Log_error0("Error initializing MAX32664 Heart Rate Algorithm");
        return false;
    }

    // Start clocks
    Util_startClock(&accelerometerReadClock);
    Util_startClock(&heartRateReadClock);

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
        accelerometerSamplesBytes[index+2] = samples[i].x_L;
        accelerometerSamplesBytes[index+1+2] = samples[i].x_H;
        accelerometerSamplesBytes[index+2+2] = samples[i].y_L;
        accelerometerSamplesBytes[index+3+2] = samples[i].y_H;
        accelerometerSamplesBytes[index+4+2] = samples[i].z_L;
        accelerometerSamplesBytes[index+5+2] = samples[i].z_H;

        index += SENSORS_ACCELEROMETER_SAMPLE_SIZE;
    }

    if(Max32664_writeInputFifo(accelerometerSamplesBytes, SENSORS_NUM_ACCELEROMETER_SAMPLES * SENSORS_ACCELEROMETER_SAMPLE_SIZE + 2)) {
        Log_info0("Sent 5 accelerometer samples to MAX32664");
    }
    else {
        Log_error0("Error sending accelerometer samples to MAX32664");
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
