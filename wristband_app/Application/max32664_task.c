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
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/GPIO.h>

#include <uartlog/UartLog.h>

#include <Board.h>

#include <icall.h>
#include <max32664_task.h>
#include <project_zero.h>
#include <util.h>
#include <i2c_util.h>

#include <services/emergency_alert_service.h>


/*********************************************************************
 * CONSTANTS
 */
#define MAX32664_REPORT_PERIOD_MS           2*40  // Change to 2*40 ms

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

/*********************************************************************
 * GLOBAL VARIABLES
 */
bool heartRateAlgorithmInitialized = false;


/*********************************************************************
 * LOCAL VARIABLES
 */
// Bio Sensor Hub data buffer
static uint8_t reportBuffer[MAX32664_MAX_NUM_SAMPLES * (MAX32664_NORMAL_REPORT_ALGORITHM_ONLY_SIZE + 1)];

static max32664_operating_mode_t operatingMode;

// I2C
static I2C_Transaction transaction;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static max32664_status_t Max32664_readSensorHubStatus(uint8_t *status);
static max32664_status_t Max32664_setOutputMode(uint8_t output_mode);
static max32664_status_t Max32664_readOutputMode(uint8_t *data);
static max32664_status_t Max32664_setFifoInterruptThreshold(uint8_t threshold);
static max32664_status_t Max32664_enableAutoGainControlAlgorithm(uint8_t enable);
static max32664_status_t Max32664_enableWhrmWspo2Algorithm(uint8_t enableMode);
static max32664_status_t Max32664_enableMax86141Sensor(uint8_t enable);
static max32664_status_t Max32664_readFifoData(uint8_t *data, int num_bytes);

// MAX32664 helper functions
static max32664_status_t Max32664_readByte(uint8_t family, uint8_t index, uint8_t *data);
static max32664_status_t Max32664_writeByte(uint8_t family, uint8_t index, uint8_t data, int delay);


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Max32664_initApplicationMode
 *
 * @brief   Initialize the Biometric Sensor Hub in Application Mode.
 */
void Max32664_initApplicationMode()
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
void Max32664_initHeartRateAlgorithm()
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
 * @fn      Max32664_readFifoNumSamples
 *
 * @brief   Get the number of samples in the Biometric Sensor Hub FIFO.
 *
 * @param   Number of samples in FIFO
 */
max32664_status_t Max32664_readFifoNumSamples(uint8_t *num_samples)
{
    return Max32664_readByte(MAX32664_READ_OUTPUT_FIFO, MAX32664_NUM_FIFO_SAMPLES, num_samples);
}

/*********************************************************************
 * @fn      Max32664_readHeartRate
 *
 * @brief   Read heart rate value from Biometric Sensor Hub.
 *
 * @param   Heart rate, SpO2, confidence and status values
 */
void Max32664_readHeartRate(heartrate_data_t reports[], int *num_reports)
{
    max32664_status_t ret;
    uint8_t num_samples;

    // Check sensor status before reading FIFO data
    uint8_t sensor_status;
    ret = Max32664_readSensorHubStatus(&sensor_status);
    if (ret != STATUS_SUCCESS || sensor_status == 1) {
        Log_error0("Error communicating with Sensor Hub, cannot read Heart Rate data");
        return;
    }
    Log_info0("Reading Heart Rate data from Sensor Hub");
    // TODO: Check if bit 3 (FIFO filled to threshold) is set

    if (Max32664_readFifoNumSamples(&num_samples) != STATUS_SUCCESS) {
        Log_error0("Error getting number of FIFO samples");
        return;
    }
    Log_info1("Reading %d samples", num_samples);

    if (Max32664_readFifoData(reportBuffer, num_samples) != STATUS_SUCCESS) {
        Log_error0("Error reading FIFO samples");
        return;
    }
    Log_info1("Read %d FIFO samples", num_samples);

    int number_reports = num_samples / MAX32664_NORMAL_REPORT_ALGORITHM_ONLY_SIZE;
    int index = 1;
    int reportIndex = 0;
    for (int i = 0; i < number_reports; i++) {
        heartrate_data_t heartrate_data;
        heartrate_data.heartRate = reportBuffer[index + 1] << 8; // MSB
        heartrate_data.heartRate |= reportBuffer[index + 2]; // LSB
        heartrate_data.heartRateConfidence = reportBuffer[index + 3];
        heartrate_data.spO2 = reportBuffer[index + 11] << 8;     // MSB
        heartrate_data.spO2 |= reportBuffer[index + 12];     // LSB
        heartrate_data.spO2Confidence = reportBuffer[index + 10];
        heartrate_data.scdState = reportBuffer[index + 19];

        index += (MAX32664_NORMAL_REPORT_ALGORITHM_ONLY_SIZE + 1);

        reports[reportIndex] = heartrate_data;
    }

    (*num_reports) = number_reports;
}


/*********************************************************************
 * @fn      Max32664_readSensorHubStatus
 *
 * @brief   Read current status of the Biometric Sensor Hub.
 */
static max32664_status_t Max32664_readSensorHubStatus(uint8_t *status)
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
static max32664_status_t Max32664_setOutputMode(uint8_t output_mode)
{
    return Max32664_writeByte(MAX32664_SET_OUTPUT_MODE, 0x00, output_mode, MAX32664_CMD_DELAY);
}


/*********************************************************************
 * @fn      Max32664_readOutputMode
 *
 * @brief   Read current output mode of the Biometric Sensor Hub.
 */
static max32664_status_t Max32664_readOutputMode(uint8_t *data)
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
static max32664_status_t Max32664_setFifoInterruptThreshold(uint8_t threshold)
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
static max32664_status_t Max32664_enableAutoGainControlAlgorithm(uint8_t enable)
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
static max32664_status_t Max32664_enableWhrmWspo2Algorithm(uint8_t enableMode)
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
static max32664_status_t Max32664_enableMax86141Sensor(uint8_t enable)
{
    return Max32664_writeByte(MAX32664_SENSOR_MODE_ENABLE, MAX32664_MAX86141_ENABLE, enable, MAX32664_ENABLE_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_readFifoData
 *
 * @brief   Read the samples from the FIFO in the Biometric Sensor Hub FIFO.
 *
 * @param   data - Buffer to store FIFO data
 * @param   num_bytes - Number of bytes to read from the FIFO
 */
static max32664_status_t Max32664_readFifoData(uint8_t *data, int num_bytes)
{
    max32664_status_t ret = STATUS_UNKNOWN_ERROR;

    uint8_t txBuffer[2];

    txBuffer[0] = MAX32664_READ_OUTPUT_FIFO;
    txBuffer[1] = MAX32664_READ_FIFO_DATA;

    transaction.slaveAddress = MAX32664_ADDRESS;
    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 2;
    transaction.readBuf    = data;
    transaction.readCount  = num_bytes + 1;

    if (Util_i2cTransfer(&transaction)) {
        Log_info0("I2C transfer successful");
        ret = (max32664_status_t)data[0];
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
static max32664_status_t Max32664_writeByte(uint8_t family, uint8_t index, uint8_t data, int delay)
{
    max32664_status_t ret = STATUS_UNKNOWN_ERROR;

    uint8_t txBuffer[3];
    uint8_t rxBuffer[1];

    txBuffer[0] = family;
    txBuffer[1] = index;
    txBuffer[2] = data;

    transaction.slaveAddress = MAX32664_ADDRESS;
    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 3;
    transaction.readBuf    = rxBuffer;
    transaction.readCount  = 1;

    if (Util_i2cTransfer(&transaction)) {
        Log_info0("I2C transfer successful");
        ret = (max32664_status_t)rxBuffer[0];
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
static max32664_status_t Max32664_readByte(uint8_t family, uint8_t index, uint8_t *data)
{
    max32664_status_t ret = STATUS_UNKNOWN_ERROR;
    (*data) = 0;

    uint8_t txBuffer[2];
    uint8_t rxBuffer[2];

    txBuffer[0] = family;
    txBuffer[1] = index;

    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 2;
    transaction.readBuf    = rxBuffer;
    transaction.readCount  = 2;

    if (Util_i2cTransfer(&transaction)) {
        Log_info0("I2C transfer successful");
        ret = (max32664_status_t)rxBuffer[0];
        (*data) = rxBuffer[1];
    }
    else {
        Log_error0("I2C transfer failed");
    }

    return ret;
}
