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
#include <project_zero.h>
#include <util.h>
#include <i2c_util.h>
#include <max32664.h>


/*********************************************************************
 * CONSTANTS
 */
#define MAX32664_REPORT_PERIOD_MS           2*40  // Change to 2*40 ms

// I2C
#define MAX32664_ADDRESS                    0x55
#define MAX32664_CMD_DELAY                  6
#define MAX32664_ENABLE_CMD_DELAY           42

// Family names
#define MAX32664_READ_SENSOR_HUB_STATUS     0x00
#define MAX32664_SET_OUTPUT_MODE            0x10
#define MAX32664_READ_OUTPUT_MODE           0x11
#define MAX32664_READ_OUTPUT_FIFO           0x12
#define MAX32664_WRITE_INPUT_FIFO           0x14
#define MAX32664_SENSOR_MODE_ENABLE         0x44
#define MAX32664_ALGORITHM_MODE_ENABLE      0x52

// Read Output
#define MAX32664_NUM_FIFO_SAMPLES           0x00
#define MAX32664_READ_FIFO_DATA             0x01

// Sensors
#define MAX32664_MAX86141_ENABLE            0x00
#define MAX32664_MAX30101_ENABLE            0x03

// Accelerometer
#define MAX32664_EXTERNAL_ACCELEROMETER     0x00
#define MAX32664_ACCELEROMETER_ENABLE       0x04

// Algorithms
#define MAX32664_AGC_ALGORITHM              0x00
#define MAX32664_MAXIMFAST_ALGORITHM        0x02
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

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Bio Sensor Hub data buffer
static uint8_t reportBuffer[MAX32664_FIFO_THRESHOLD * MAX32664_MAXIMFAST_REPORT_ALGO_SIZE];

// I2C
static I2C_Transaction transaction;
static uint8_t txBuffer[128];
static uint8_t rxBuffer[128];


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static max32664_status_t Max32664_setOutputMode(uint8_t output_mode);
static max32664_status_t Max32664_readOutputMode(uint8_t *data);
static max32664_status_t Max32664_setFifoInterruptThreshold(uint8_t threshold);
static max32664_status_t Max32664_enableAutoGainControlAlgorithm(uint8_t enable);
static max32664_status_t Max32664_enableWhrmWspo2Algorithm(uint8_t enableMode);
static max32664_status_t Max32664_enableMaximFastAlgorithm(uint8_t enableMode);
static max32664_status_t Max32664_enableMax86141Sensor(uint8_t enable);
static max32664_status_t Max32664_enableMax30101Sensor(uint8_t enable);
static max32664_status_t Max32664_enableExternalHostAccelerometer(uint8_t enable);

// MAX32664 helper functions
static max32664_status_t Max32664_readByte(uint8_t family, uint8_t index, uint8_t *data);
static max32664_status_t Max32664_readByteArray(uint8_t family, uint8_t index, uint8_t *data, int num_bytes);
static max32664_status_t Max32664_writeByte(uint8_t family, uint8_t index, uint8_t data, int delay);
static max32664_status_t Max32664_writeByteArray(uint8_t family, uint8_t index, uint8_t *data, int num_bytes, int delay);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Max32664_initApplicationMode
 *
 * @brief   Initialize the Biometric Sensor Hub in Application Mode.
 *
 * @param   isr_fxn - ISR to handle interrupt triggered by MFIO pin.
 */
max32664_status_t Max32664_initApplicationMode(void *isr_fxn) {
    uint8_t status;

    // Initialize GPIO pins
    GPIO_setConfig(Board_GPIO_MAX32664_MFIO, GPIO_CFG_OUT_STD);
    GPIO_setConfig(Board_GPIO_MAX32664_RESET, GPIO_CFG_OUT_STD);

    GPIO_write(Board_GPIO_MAX32664_RESET, GPIO_CFG_OUT_LOW);
    GPIO_write(Board_GPIO_MAX32664_MFIO, GPIO_CFG_OUT_HIGH);

    // Delay for 10 ms
    Task_sleep(10 * (1000 / Clock_tickPeriod));

    GPIO_write(Board_GPIO_MAX32664_RESET, GPIO_CFG_OUT_HIGH);

    // Delay for 1 s
    Task_sleep(1100 * (1000 / Clock_tickPeriod));

    // Change MFIO pin to an input to trigger interrupts
    GPIO_setConfig(Board_GPIO_MAX32664_MFIO, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    // Enable MFIO interrupts
    GPIO_setCallback(Board_GPIO_MAX32664_MFIO, (GPIO_CallbackFxn)isr_fxn);
    GPIO_enableInt(Board_GPIO_MAX32664_MFIO);

    // Test I2C connection
    return Max32664_readSensorHubStatus(&status);
}

/*********************************************************************
 * @fn      Max32664_initMaximFastAlgorithm
 *
 * @brief   Initialize the MaximFast Heart Rate algorithm for finger reading on the Biometric Sensor Hub.
 */
max32664_status_t Max32664_initMaximFastAlgorithm() {
    uint8_t enable;
    uint8_t mode;
    max32664_status_t ret = STATUS_UNKNOWN_ERROR;

    // Set output mode to Algorithm Data
    ret = Max32664_setOutputMode(OUTPUT_MODE_ALGORITHM_DATA);
    if (ret != STATUS_SUCCESS) {
        Log_error0("Error setting output mode");
        return ret;
    }
    Log_info0("Set output mode to Algorithm Data");

    // Set FIFO threshold for interrupt to be triggered
    uint8_t threshold = MAX32664_FIFO_THRESHOLD;
    ret = Max32664_setFifoInterruptThreshold(threshold);
    if (ret != STATUS_SUCCESS) {
        Log_error0("Error setting FIFO interrupt threshold");
        return ret;
    }
    Log_info1("Set FIFO interrupt threshold to: %d", threshold);

    // Enable host accelerometer
    ret = Max32664_enableExternalHostAccelerometer(0x01);
    if (ret != STATUS_SUCCESS) {
        Log_error0("Error enabling external host accelerometer");
        return ret;
    }
    Log_info0("Enable external host accelerometer");

    // Enable Automatic Gain Control algorithm
    enable = 0x01;
    ret = Max32664_enableAutoGainControlAlgorithm(enable);
    if (ret != STATUS_SUCCESS) {
        Log_error0("Error enabling Automatic Gain Control algorithm");
        return ret;
    }
    Log_info0("Enable Automatic Gain Control algorithm");

    // Enable MAX30101 sensor
    enable = 0x01;
    ret = Max32664_enableMax30101Sensor(enable);
    if (ret != STATUS_SUCCESS) {
        Log_error0("Error enabling MAX30101 sensor");
        return ret;
    }
    Log_info0("Enable MAX30101 sensor");

    // Enable MaximFast algorithm (version A)
    // Mode 1: Normal Report
    // Mode 2: Extended Report
    mode = MAX32664_NORMAL_REPORT_MODE;
    ret = Max32664_enableMaximFastAlgorithm(mode);
    if (ret != STATUS_SUCCESS) {
        Log_error0("Error enabling MaximFast algorithm (version A)");
        return ret;
    }
    Log_info1("Enable MaximFast algorithm (version A) in mode: %d", mode);

    Log_info0("MAX32664 Heart Rate Algorithm configured");

    return ret;
}

/*********************************************************************
 * @fn      Max32664_readFifoNumSamples
 *
 * @brief   Get the number of samples in the Biometric Sensor Hub FIFO.
 *
 * @param   Number of samples in FIFO
 */
max32664_status_t Max32664_readFifoNumSamples(uint8_t *num_samples) {
    return Max32664_readByte(MAX32664_READ_OUTPUT_FIFO, MAX32664_NUM_FIFO_SAMPLES, num_samples);
}

/*********************************************************************
 * @fn      Max32664_readHeartRate
 *
 * @brief   Read heart rate value from Biometric Sensor Hub.
 *
 * @param   reports - Array to store heart rate, SpO2, confidence and status values
 *          num_report - Number of reports to read
 */
max32664_status_t Max32664_readHeartRate(heartrate_data_t *reports, int num_reports) {
    max32664_status_t ret = STATUS_UNKNOWN_ERROR;

    int num_bytes = num_reports * MAX32664_MAXIMFAST_REPORT_ALGO_SIZE;
    ret = Max32664_readByteArray(MAX32664_READ_OUTPUT_FIFO, MAX32664_READ_FIFO_DATA, reportBuffer, num_bytes);
    if (ret != STATUS_SUCCESS) {
        Log_error0("Error reading output FIFO");
        return ret;
    }

    int index = 0;
    for (int i = 0; i < num_reports; i++) {
        reports[i].heartRate = (uint16_t)reportBuffer[index + 0] << 8; // MSB
        reports[i].heartRate |= reportBuffer[index + 1]; // LSB
        reports[i].heartRateConfidence = reportBuffer[index + 2];
        reports[i].spO2 = (uint16_t)reportBuffer[index + 3] << 8;     // MSB
        reports[i].spO2 |= reportBuffer[index + 4];     // LSB
        reports[i].spO2Confidence = 0;
        reports[i].scdState = reportBuffer[index + 5];

        // Heart rate and SpO2 are multiplied x10
        reports[i].heartRate /= 10;
        reports[i].spO2 /= 10;

        index += MAX32664_MAXIMFAST_REPORT_ALGO_SIZE;
    }

    return ret;
}

/*********************************************************************
 * @fn      Max32664_writeInputFifo
 *
 * @brief   Write to the input FIFO of the Biometric Sensor Hub.
 */
max32664_status_t Max32664_writeInputFifo(uint8_t *data, int num_bytes) {
    return Max32664_writeByteArray(MAX32664_WRITE_INPUT_FIFO, 0, data, num_bytes, MAX32664_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_readSensorHubStatus
 *
 * @brief   Read current status of the Biometric Sensor Hub.
 */
max32664_status_t Max32664_readSensorHubStatus(uint8_t *status) {
    return Max32664_readByte(MAX32664_READ_SENSOR_HUB_STATUS, 0x00, status);
}

/*********************************************************************
 * @fn      Max32664_setOutputMode
 *
 * @brief   Set output mode of the Biometric Sensor Hub.
 *
 * @param   Output mode
 */
static max32664_status_t Max32664_setOutputMode(uint8_t output_mode) {
    return Max32664_writeByte(MAX32664_SET_OUTPUT_MODE, 0x00, output_mode, MAX32664_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_setFifoInterruptThreshold
 *
 * @brief   Set the number of samples required in the Biometric Sensor Hub FIFO to trigger an interrupt.
 *
 * @param   Interrupt threshold for FIFO (number of samples)
 */
static max32664_status_t Max32664_setFifoInterruptThreshold(uint8_t threshold) {
    return Max32664_writeByte(MAX32664_SET_OUTPUT_MODE, 0x01, threshold, MAX32664_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_enableAutoGainControlAlgorithm
 *
 * @brief   Enable or disable the Automatic Gain Control algorithm on the Biometric Sensor Hub.
 *
 * @param   Enable or Disable
 */
static max32664_status_t Max32664_enableAutoGainControlAlgorithm(uint8_t enable) {
    return Max32664_writeByte(MAX32664_ALGORITHM_MODE_ENABLE, MAX32664_AGC_ALGORITHM, enable, MAX32664_ENABLE_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_enableWhrmWspo2Algorithm
 *
 * @brief   Enable or disable the WHRM + WSpO2 algorithm on the Biometric Sensor Hub.
 *
 * @param   Enable Mode or Disable
 */
static max32664_status_t Max32664_enableWhrmWspo2Algorithm(uint8_t enableMode) {
    return Max32664_writeByte(MAX32664_ALGORITHM_MODE_ENABLE, MAX32664_WHRM_WSPO2_ALGORITHM, enableMode, 130);
}

/*********************************************************************
 * @fn      Max32664_enableMaximFastAlgorithm
 *
 * @brief   Enable or disable the MaximFast algorithm on the Biometric Sensor Hub.
 *
 * @param   Enable Mode or Disable
 */
static max32664_status_t Max32664_enableMaximFastAlgorithm(uint8_t enableMode) {
    return Max32664_writeByte(MAX32664_ALGORITHM_MODE_ENABLE, MAX32664_MAXIMFAST_ALGORITHM, enableMode, MAX32664_ENABLE_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_enableMax86141Sensor
 *
 * @brief   Enable or disable the MAX86141 wrist sensor on the Biometric Sensor Hub.
 *
 * @param   enable - Enable or Disable
 */
static max32664_status_t Max32664_enableMax86141Sensor(uint8_t enable) {
    return Max32664_writeByte(MAX32664_SENSOR_MODE_ENABLE, MAX32664_MAX86141_ENABLE, enable, MAX32664_ENABLE_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_enableMax30101Sensor
 *
 * @brief   Enable or disable the MAX30101 finger sensor on the Biometric Sensor Hub.
 *
 * @param   enable - Enable or Disable
 */
static max32664_status_t Max32664_enableMax30101Sensor(uint8_t enable) {
    return Max32664_writeByte(MAX32664_SENSOR_MODE_ENABLE, MAX32664_MAX30101_ENABLE, enable, MAX32664_ENABLE_CMD_DELAY);
}

/*********************************************************************
 * @fn      Max32664_enableExternalHostAccelerometer
 *
 * @brief   Enable or disable the MAX86141 external host accelerometer.
 *
 * @param   enable - Enable or Disable
 */
static max32664_status_t Max32664_enableExternalHostAccelerometer(uint8_t enable) {
    uint8_t data[2] = {MAX32664_EXTERNAL_ACCELEROMETER, enable};
    return Max32664_writeByteArray(MAX32664_SENSOR_MODE_ENABLE, MAX32664_ACCELEROMETER_ENABLE, data, 2, MAX32664_ENABLE_CMD_DELAY);
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
static max32664_status_t Max32664_writeByte(uint8_t family, uint8_t index, uint8_t data, int delay) {
    max32664_status_t ret = STATUS_UNKNOWN_ERROR;

    txBuffer[0] = family;
    txBuffer[1] = index;
    txBuffer[2] = data;

    transaction.slaveAddress = MAX32664_ADDRESS;
    transaction.writeBuf = txBuffer;
    transaction.writeCount = 3;
    transaction.readCount = 0;
    if (!Util_i2cTransfer(&transaction)) {
        Log_error0("I2C write failed");
        return ret;
    }

    Task_sleep(delay * (1000 / Clock_tickPeriod));

    transaction.readBuf = rxBuffer;
    transaction.readCount = 1;
    transaction.writeCount = 0;
    if (!Util_i2cTransfer(&transaction)) {
        Log_error0("I2C read failed");
        return ret;
    }

    ret = (max32664_status_t)rxBuffer[0];

    return ret;
}

/*********************************************************************
 * @fn      Max32664_writeByteArray
 *
 * @brief   Write a byte array to the Biometric Sensor Hub.
 *
 * @param   family  -   Family register byte
 *          index   -   Index byte of family
 *          data    -   Data to write
 *          num_bytes   -   Number of bytes to write
 */
static max32664_status_t Max32664_writeByteArray(uint8_t family, uint8_t index, uint8_t *data, int num_bytes, int delay) {
    max32664_status_t ret = STATUS_UNKNOWN_ERROR;

    txBuffer[0] = family;
    txBuffer[1] = index;

    for (int i = 0; i < num_bytes; i++) {
        txBuffer[i+2] = data[i];
    }

    transaction.slaveAddress = MAX32664_ADDRESS;
    transaction.writeBuf   = txBuffer;
    transaction.writeCount = num_bytes + 2;
    transaction.readCount = 0;
    if (!Util_i2cTransfer(&transaction)) {
        Log_error0("I2C write failed");
        return ret;
    }

    Task_sleep(delay * (1000 / Clock_tickPeriod));

    transaction.readBuf = rxBuffer;
    transaction.readCount = 1;
    transaction.writeCount = 0;
    if (!Util_i2cTransfer(&transaction)) {
        Log_error0("I2C read failed");
        return ret;
    }

    ret = (max32664_status_t)rxBuffer[0];

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
static max32664_status_t Max32664_readByte(uint8_t family, uint8_t index, uint8_t *data) {
    max32664_status_t ret = STATUS_UNKNOWN_ERROR;
    (*data) = 0;

    txBuffer[0] = family;
    txBuffer[1] = index;

    transaction.slaveAddress = MAX32664_ADDRESS;
    transaction.writeBuf = txBuffer;
    transaction.writeCount = 2;
    transaction.readCount = 0;
    if (!Util_i2cTransfer(&transaction)) {
        Log_error0("I2C write failed");
        return ret;
    }

    Task_sleep(MAX32664_CMD_DELAY * (1000 / Clock_tickPeriod));

    transaction.readBuf = rxBuffer;
    transaction.readCount = 2;
    transaction.writeCount = 0;
    if (!Util_i2cTransfer(&transaction)) {
        Log_error0("I2C read failed");
        return ret;
    }

    ret = (max32664_status_t)rxBuffer[0];
    (*data) = rxBuffer[1];

    return ret;
}

/*********************************************************************
 * @fn      Max32664_readByteArray
 *
 * @brief   Read a byte array from the Biometric Sensor Hub.
 *
 * @param   family  -   Family register byte
 *          index   -   Index byte of family
 *          data    -   Data to read
 *          num_bytes   -   Number of bytes to read
 */
static max32664_status_t Max32664_readByteArray(uint8_t family, uint8_t index, uint8_t *data, int num_bytes) {
    max32664_status_t ret = STATUS_UNKNOWN_ERROR;

    txBuffer[0] = family;
    txBuffer[1] = index;

    transaction.slaveAddress = MAX32664_ADDRESS;
    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 2;
    transaction.readCount = 0;
    if (!Util_i2cTransfer(&transaction)) {
        Log_error0("I2C write failed");
        return ret;
    }

    Task_sleep(MAX32664_CMD_DELAY * (1000 / Clock_tickPeriod));

    transaction.readBuf = rxBuffer;
    transaction.readCount = num_bytes + 1;
    transaction.writeCount = 0;
    if (!Util_i2cTransfer(&transaction)) {
        Log_error0("I2C transfer failed");
        return ret;
    }

    ret = (max32664_status_t)rxBuffer[0];
    for (int i = 0; i < num_bytes; i++) {
        data[i] = rxBuffer[i+1];
    }

    return ret;
}
