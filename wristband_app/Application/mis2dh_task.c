/*
 * mis2dh_task.c
 *
 *  Created on: Feb 17, 2021
 *      Author: Aidan Clyens
 */

/*******************************************************************************
 * INCLUDES
 */
#include <xdc/std.h>

#include <ti/drivers/I2C.h>

#include <uartlog/UartLog.h>

#include <Board.h>

#include <icall.h>
#include <mis2dh_task.h>
#include <i2c_util.h>
#include <util.h>

/*********************************************************************
 * CONSTANTS
 */
// I2C
#define MIS2DH_ADDRESS                  0x18

// Registers
#define MIS2DH_CTRL_REG1                0x20
#define MIS2DH_CTRL_REG5                0x24
#define MIS2DH_OUT_X_L                  0x28
#define MIS2DH_OUT_X_H                  0x29
#define MIS2DH_OUT_Y_L                  0x2A
#define MIS2DH_OUT_Y_H                  0x2B
#define MIS2DH_OUT_Z_L                  0x2C
#define MIS2DH_OUT_Z_H                  0x2D
#define MIS2DH_FIFO_CTRL_REG            0x2E
#define MIS2DH_FIFO_SRC_REG             0x2F

// Masks
#define MIS2DH_DATARATE_MASK            0xF0
#define MIS2DH_LOW_POWER_MODE_MASK      0x08
#define MIS2DH_FIFO_ENABLE_MASK         0x40
#define MIS2DH_FIFO_MODE_MASK           0xC0
#define MIS2DH_FIFO_FSS_MASK            0x1F
#define MIS2DH_FIFO_EMPTY_MASK          0x20
#define MIS2DH_FIFO_OVRN_MASK           0x40
#define MIS2DH_FIFO_FSS_MASK            0x1F

/*********************************************************************
 * TYPEDEFS
 */
// Data Rate
typedef enum {
    DATARATE_POWER_DOWN = 0x00,
    DATARATE_1HZ = 0x01,
    DATARATE_10HZ = 0x02,
    DATARATE_25HZ = 0x03,
    DATARATE_50HZ = 0x04,
    DATARATE_100HZ = 0x05,
    DATARATE_200HZ = 0x06,
    DATARATE_400HZ = 0x07,
    DATARATE_1620HZ = 0x08,
    DATARATE_5376HZ = 0x09
} data_rate_t;

// FIFO Mode
typedef enum {
    FIFO_MODE_BYPASS = 0x0,
    FIFO_MODE_FIFO = 0x01,
    FIFO_MODE_STREAM = 0x2,
    FIFO_MODE_STREAM_TO_FIFO = 0x3
} fifo_mode_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// I2C
static I2C_Transaction transaction;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bool Mis2dh_configureDataRate(data_rate_t dataRate);
static bool Mis2dh_setLowPowerMode(bool enable);
static bool Mis2dh_setFifoMode(fifo_mode_t fifoMode);
static bool Mis2dh_enableFifo(bool enable);
static bool Mis2dh_isFifoFull(bool *full);
static bool Mis2dh_isFifoEmpty(bool *empty);
static bool Mis2dh_getNumUnreadSamples(int *num_samples);

// I2C
static bool Mis2dh_writeRegister(uint8_t regAddress, uint8_t data);
static bool Mis2dh_readRegister(uint8_t regAddress, uint8_t *data);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Mis2dh_init
 *
 * @brief   Initialization for MIS2DH task.
 */
bool Mis2dh_init(void) {
    // Set data rate
    if (Mis2dh_configureDataRate(DATARATE_25HZ)) {
        Log_info0("Set data rate to 25Hz");
    }
    else {
        Log_error0("Error setting data rate");
        return false;
    }

    // Enable low power mode
    if (Mis2dh_setLowPowerMode(true)) {
        Log_info0("Enable Low Power Mode");
    }
    else {
        Log_error0("Error enabling Low Power Mode");
        return false;
    }

    // Configure FIFO mode
    if (Mis2dh_setFifoMode(FIFO_MODE_STREAM)) {
        Log_info0("Set FIFO Mode to Stream");
    }
    else {
        Log_error0("Error setting FIFO Mode");
        return false;
    }

    // Enable FIFO
    if (Mis2dh_enableFifo(true)) {
        Log_info0("Enabled FIFO");
    }
    else {
        Log_error0("Error enabling FIFO");
        return false;
    }

    return true;
}

/*********************************************************************
 * @fn      Mis2dh_readSensorData
 *
 * @brief   Read accelerometer data from MIS2DH FIFO.
 *
 * @param   data - X, Y, and Z accelerometer data.
 */
bool Mis2dh_readSensorData(sensor_data_t *data) {
    uint8_t txBuffer[1];
    uint8_t rxBuffer[6];

    txBuffer[0] = MIS2DH_OUT_X_L;

    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 1;
    transaction.readBuf    = rxBuffer;
    transaction.readCount  = 6;

    if (Util_i2cTransfer(&transaction)) {
        Log_info0("I2C transfer successful");
        data->x_L = rxBuffer[0];
        data->x_H = rxBuffer[1];
        data->y_L = rxBuffer[2];
        data->y_H = rxBuffer[3];
        data->z_L = rxBuffer[4];
        data->z_H = rxBuffer[5];
        return true;
    }
    else {
        Log_error0("I2C transfer failed");
        return false;
    }
}

/*********************************************************************
 * @fn      Mis2dh_writeRegister
 *
 * @brief   Write data to register.
 *
 * @param   reg_address - Address of register.
 *          data - Data to write to register.
 */
static bool Mis2dh_writeRegister(uint8_t regAddress, uint8_t data) {
    uint8_t txBuffer[1];
    txBuffer[0] = regAddress;
    txBuffer[1] = data;

    transaction.slaveAddress = MIS2DH_ADDRESS;
    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 2;
    transaction.readBuf    = NULL;
    transaction.readCount  = 0;

    if (Util_i2cTransfer(&transaction)) {
        Log_info0("I2C transfer successful");
        return true;
    }
    else {
        Log_error0("I2C transfer failed");
        return false;
    }
}

/*********************************************************************
 * @fn      Mis2dh_readRegister
 *
 * @brief   Write data to register.
 *
 * @param   reg_address - Address of register.
 *          data - Variable to read data into
 */
static bool Mis2dh_readRegister(uint8_t regAddress, uint8_t *data) {
    uint8_t txBuffer[1];
    uint8_t rxBuffer[1];

    txBuffer[0] = regAddress;

    transaction.slaveAddress = MIS2DH_ADDRESS;
    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 1;
    transaction.readBuf    = rxBuffer;
    transaction.readCount  = 1;

    if (Util_i2cTransfer(&transaction)) {
        Log_info0("I2C transfer successful");
        (*data) = rxBuffer[0];
        return true;
    }
    else {
        Log_error0("I2C transfer failed");
        return false;
    }
}

/*********************************************************************
 * @fn      Mis2dh_configureDataRate
 *
 * @brief   Set MIS2DH data rate.
 *
 * @param   dataRate - Data rate for accelerometer reading.
 */
static bool Mis2dh_configureDataRate(data_rate_t dataRate) {
    uint8_t ctrl_reg1_data;

    if (!Mis2dh_readRegister(MIS2DH_CTRL_REG1, &ctrl_reg1_data)) {
        return false;
    }

    // Clear and write new value
    ctrl_reg1_data &= ~MIS2DH_DATARATE_MASK;
    ctrl_reg1_data |= (dataRate << 4);

    return Mis2dh_writeRegister(MIS2DH_CTRL_REG1, ctrl_reg1_data);
}

/*********************************************************************
 * @fn      Mis2dh_setLowPowerMode
 *
 * @brief   Set MIS2DH Low Power Mode.
 *
 * @param   enable - Enable or disable Low Power Mode.
 */
static bool Mis2dh_setLowPowerMode(bool enable) {
    uint8_t ctrl_reg1_data;

    if (!Mis2dh_readRegister(MIS2DH_CTRL_REG1, &ctrl_reg1_data)) {
        return false;
    }

    // Clear and write new value
    ctrl_reg1_data &= ~MIS2DH_LOW_POWER_MODE_MASK;
    if (enable) ctrl_reg1_data |= MIS2DH_LOW_POWER_MODE_MASK;

    return Mis2dh_writeRegister(MIS2DH_CTRL_REG1, ctrl_reg1_data);
}

/*********************************************************************
 * @fn      Mis2dh_setFifoMode
 *
 * @brief   Set MIS2DH FIFO Mode.
 *
 * @param   fifoMode - FIFO Mode: Bypass, FIFO, Stream, or Stream and FIFO.
 */
static bool Mis2dh_setFifoMode(fifo_mode_t fifoMode) {
    uint8_t fifo_ctrl_reg_data;

    if (!Mis2dh_readRegister(MIS2DH_FIFO_CTRL_REG, &fifo_ctrl_reg_data)) {
        return false;
    }

    // Clear and write new value
    fifo_ctrl_reg_data &= ~MIS2DH_FIFO_MODE_MASK;
    fifo_ctrl_reg_data |= (fifoMode << 6);

    return Mis2dh_writeRegister(MIS2DH_FIFO_CTRL_REG, fifo_ctrl_reg_data);
}

/*********************************************************************
 * @fn      Mis2dh_enableFifo
 *
 * @brief   Enable MIS2DH FIFO.
 *
 * @param   enable - Enable or disable FIFO.
 */
static bool Mis2dh_enableFifo(bool enable) {
    uint8_t ctrl_reg5_data;

    if (!Mis2dh_readRegister(MIS2DH_CTRL_REG5, &ctrl_reg5_data)) {
        return false;
    }

    // Clear and write new value
    ctrl_reg5_data &= ~MIS2DH_FIFO_ENABLE_MASK;
    if (enable) ctrl_reg5_data |= MIS2DH_FIFO_ENABLE_MASK;

    return Mis2dh_writeRegister(MIS2DH_CTRL_REG5, ctrl_reg5_data);
}

/*********************************************************************
 * @fn      Mis2dh_isFifoFull
 *
 * @brief   Check if the MIS2DH FIFO is full.
 *
 * @param   full - Boolean indicating whether the FIFO is full or not.
 */
static bool Mis2dh_isFifoFull(bool *full) {
    uint8_t fifo_src_reg_data;

    if (!Mis2dh_readRegister(MIS2DH_FIFO_SRC_REG, &fifo_src_reg_data)) {
        return false;
    }

    (*full) = (fifo_src_reg_data & MIS2DH_FIFO_OVRN_MASK) > 0;

    return true;
}

/*********************************************************************
 * @fn      Mis2dh_getNumUnreadSamples
 *
 * @brief   Get the number of unread samples in the MIS2DH FIFO.
 *
 * @param   num_samples - Number of unread samples in the FIFO.
 */
static bool Mis2dh_getNumUnreadSamples(int *num_samples) {
    uint8_t fifo_src_reg_data;

    if (!Mis2dh_readRegister(MIS2DH_FIFO_SRC_REG, &fifo_src_reg_data)) {
        return false;
    }

    (*num_samples) = (fifo_src_reg_data & MIS2DH_FIFO_FSS_MASK);

    return true;
}

/*********************************************************************
 * @fn      Mis2dh_isFifoEmpty
 *
 * @brief   Check if the MIS2DH FIFO is empty.
 *
 * @param   empty - Boolean indicating whether the FIFO is empty or not.
 */
static bool Mis2dh_isFifoEmpty(bool *empty) {
    uint8_t fifo_src_reg_data;

    if (!Mis2dh_readRegister(MIS2DH_FIFO_SRC_REG, &fifo_src_reg_data)) {
        return false;
    }

    (*empty) = (fifo_src_reg_data & MIS2DH_FIFO_EMPTY_MASK) > 0;

    return true;
}
