/*
 * lis3dh.c
 *
 *  Created on: Feb 17, 2021
 *      Author: Aidan Clyens
 */

/*******************************************************************************
 * INCLUDES
 */
#include <xdc/std.h>

#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>

#include <uartlog/UartLog.h>

#include <Board.h>

#include <icall.h>
#include <i2c_util.h>
#include <lis3dh.h>
#include <util.h>

/*********************************************************************
 * CONSTANTS
 */
// I2C
#define LIS3DH_ADDRESS                  0x19

// Registers
#define LIS3DH_CTRL_REG1                0x20
#define LIS3DH_CTRL_REG3                0x22
#define LIS3DH_CTRL_REG4                0x23
#define LIS3DH_CTRL_REG5                0x24
#define LIS3DH_OUT_X_L                  0x28
#define LIS3DH_OUT_X_H                  0x29
#define LIS3DH_OUT_Y_L                  0x2A
#define LIS3DH_OUT_Y_H                  0x2B
#define LIS3DH_OUT_Z_L                  0x2C
#define LIS3DH_OUT_Z_H                  0x2D
#define LIS3DH_FIFO_CTRL_REG            0x2E
#define LIS3DH_FIFO_SRC_REG             0x2F
#define LIS3DH_INT1_CFG                 0x30
#define LIS3DH_INT1_SRC                 0x31
#define LIS3DH_INT1_THS                 0x32
#define LIS3DH_INT1_DURATION            0x33

// Masks
#define LIS3DH_DATARATE_MASK            0xF0
#define LIS3DH_LOW_POWER_MODE_MASK      0x08
#define LIS3DH_FIFO_ENABLE_MASK         0x40
#define LIS3DH_FIFO_MODE_MASK           0xC0
#define LIS3DH_FIFO_FSS_MASK            0x1F
#define LIS3DH_FIFO_EMPTY_MASK          0x20
#define LIS3DH_FIFO_OVRN_MASK           0x40
#define LIS3DH_FIFO_FSS_MASK            0x1F

// Bitfield values
#define LIS3DH_X_LOW                    0x01
#define LIS3DH_X_HIGH                   0x02
#define LIS3DH_Y_LOW                    0x04
#define LIS3DH_Y_HIGH                   0x08
#define LIS3DH_Z_LOW                    0x10
#define LIS3DH_Z_HIGH                   0x20
#define LIS3DH_SIX_D                    0x40
#define LIS3DH_AOI                      0x80

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
static bool Lis3dh_configureDataRate(data_rate_t dataRate);
static bool Lis3dh_setLowPowerMode(bool enable);
static bool Lis3dh_setHighResolutionMode(bool enable);
static bool Lis3dh_setFifoMode(fifo_mode_t fifoMode);
static bool Lis3dh_enableFifo(bool enable);
static bool Lis3dh_isFifoFull(bool *full);
static bool Lis3dh_isFifoEmpty(bool *empty);
static bool Lis3dh_getFullScaleSelection(int *scale);

static bool Lis3dh_configureFreeFallInterrupt(bool enable, float threshold, int duration);

// I2C
static bool Lis3dh_writeRegister(uint8_t regAddress, uint8_t data);
static bool Lis3dh_readRegister(uint8_t regAddress, uint8_t *data);
static bool Lis3dh_readRegisterRegion(uint8_t regAddress, int length, uint8_t *data);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Lis3dh_init
 *
 * @brief   Initialization for LIS3DH task.
 */
bool Lis3dh_init(void *isr_fxn) {
    // Configure GPIO
    GPIO_setConfig(Board_GPIO_LIS3DH_INT1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    // Enable INT1 interrupts
    GPIO_setCallback(Board_GPIO_LIS3DH_INT1, (GPIO_CallbackFxn)isr_fxn);
    GPIO_enableInt(Board_GPIO_LIS3DH_INT1);

    // Set data rate
    if (Lis3dh_configureDataRate(DATARATE_100HZ)) {
        Log_info0("Set data rate to 100Hz");
    }
    else {
        Log_error0("Error setting data rate");
        return false;
    }

    // Enable low power mode
    if (Lis3dh_setLowPowerMode(true)) {
        Log_info0("Enable Low Power Mode");
    }
    else {
        Log_error0("Error enabling Low Power Mode");
        return false;
    }

    // Configure FIFO mode
    if (Lis3dh_setFifoMode(FIFO_MODE_STREAM)) {
        Log_info0("Set FIFO Mode to Stream");
    }
    else {
        Log_error0("Error setting FIFO Mode");
        return false;
    }

    // Configure interrupts
    if (Lis3dh_configureFreeFallInterrupt(true, 0.5, 5)) {
        Log_info0("Configured free-fall interrupt");
    }
    else {
        Log_error0("Error configuring free-fall interrupt");
        return false;
    }

    // Enable FIFO
    if (Lis3dh_enableFifo(true)) {
        Log_info0("Enabled FIFO");
    }
    else {
        Log_error0("Error enabling FIFO");
        return false;
    }

    return true;
}

/*********************************************************************
 * @fn      Lis3dh_getNumUnreadSamples
 *
 * @brief   Get the number of unread samples in the LIS3DH FIFO.
 *
 * @param   num_samples - Number of unread samples in the FIFO.
 */
bool Lis3dh_getNumUnreadSamples(int *num_samples) {
    uint8_t fifo_src_reg_data;

    if (!Lis3dh_readRegister(LIS3DH_FIFO_SRC_REG, &fifo_src_reg_data)) {
        return false;
    }

    (*num_samples) = (fifo_src_reg_data & LIS3DH_FIFO_FSS_MASK);

    return true;
}

/*********************************************************************
 * @fn      Lis3dh_readSensorData
 *
 * @brief   Read accelerometer data from LIS3DH FIFO.
 *
 * @param   data - X, Y, and Z accelerometer data.
 *          numSamples - Number of sensor samples to read.
 */
bool Lis3dh_readSensorData(sensor_data_t *data, int numSamples) {
    uint8_t sample[6];

    for (int i = 0; i < numSamples; i++) {
        if (!Lis3dh_readRegisterRegion(LIS3DH_OUT_X_L, 6, sample)) {
            return false;
        }

        sensor_data_t sensor_data;
        sensor_data.x_L = sample[0];
        sensor_data.x_H = sample[1];
        sensor_data.y_L = sample[2];
        sensor_data.y_H = sample[3];
        sensor_data.z_L = sample[4];
        sensor_data.z_H = sample[5];

        *data = sensor_data;
        data++;
    }

    return true;
}

/*********************************************************************
 * @fn      Lis3dh_printSample
 *
 * @brief   Print accelerometer sensor sample to the serial log.
 *
 * @param   data - X, Y, and Z accelerometer data.
 */
void Lis3dh_printSample(sensor_data_t data) {
    int16_t x, y, z;
    x = (int16_t)(data.x_H << 8) | (int16_t)data.x_L;
    y = (int16_t)(data.y_H << 8) | (int16_t)data.y_L;
    z = (int16_t)(data.z_H << 8) | (int16_t)data.z_L;

    Log_info3("(%d, %d, %d)", x, y, z);
}

/*********************************************************************
 * @fn      Lis3dh_clearInterrupts
 *
 * @brief   Clear any active interrupts.
 */
bool Lis3dh_clearInterrupts() {
    uint8_t int1_src;
    return Lis3dh_readRegister(LIS3DH_INT1_SRC, &int1_src);
}


/*********************************************************************
 * @fn      Lis3dh_writeRegister
 *
 * @brief   Write data to register.
 *
 * @param   reg_address - Address of register.
 *          data - Data to write to register.
 */
static bool Lis3dh_writeRegister(uint8_t regAddress, uint8_t data) {
    uint8_t txBuffer[1];
    txBuffer[0] = regAddress;
    txBuffer[1] = data;

    transaction.slaveAddress = LIS3DH_ADDRESS;
    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 2;
    transaction.readBuf    = NULL;
    transaction.readCount  = 0;

    if (Util_i2cTransfer(&transaction)) {
        return true;
    }
    else {
        Log_error0("I2C transfer failed");
        return false;
    }
}

/*********************************************************************
 * @fn      Lis3dh_readRegister
 *
 * @brief   Write data to register.
 *
 * @param   reg_address - Address of register.
 *          data - Variable to read data into.
 */
static bool Lis3dh_readRegister(uint8_t regAddress, uint8_t *data) {
    uint8_t txBuffer[1];
    uint8_t rxBuffer[1];

    txBuffer[0] = regAddress;

    transaction.slaveAddress = LIS3DH_ADDRESS;
    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 1;
    transaction.readBuf    = rxBuffer;
    transaction.readCount  = 1;

    if (Util_i2cTransfer(&transaction)) {
        (*data) = rxBuffer[0];
        return true;
    }
    else {
        Log_error0("I2C transfer failed");
        return false;
    }
}

/*********************************************************************
 * @fn      Lis3dh_readRegisterRegion
 *
 * @brief   Read data from a sequence of registers.
 *
 * @param   reg_address - Address of starting register.
 *          length - Number of registers to read.
 *          data - Variable to read data into.
 */
static bool Lis3dh_readRegisterRegion(uint8_t regAddress, int length, uint8_t *data) {
    uint8_t txBuffer[1];

    // Set auto-increment bit
    regAddress |= 0x80;
    txBuffer[0] = regAddress;

    transaction.slaveAddress = LIS3DH_ADDRESS;
    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 1;
    transaction.readBuf    = data;
    transaction.readCount  = length;

    if (Util_i2cTransfer(&transaction)) {
        return true;
    }
    else {
        Log_error0("I2C transfer failed");
        return false;
    }
}


/*********************************************************************
 * @fn      Lis3dh_configureDataRate
 *
 * @brief   Set LIS3DH data rate.
 *
 * @param   dataRate - Data rate for accelerometer reading.
 */
static bool Lis3dh_configureDataRate(data_rate_t dataRate) {
    uint8_t ctrl_reg1_data;

    if (!Lis3dh_readRegister(LIS3DH_CTRL_REG1, &ctrl_reg1_data)) {
        return false;
    }

    // Clear and write new value
    ctrl_reg1_data &= ~LIS3DH_DATARATE_MASK;
    ctrl_reg1_data |= (dataRate << 4);

    return Lis3dh_writeRegister(LIS3DH_CTRL_REG1, ctrl_reg1_data);
}

/*********************************************************************
 * @fn      Lis3dh_setLowPowerMode
 *
 * @brief   Set LIS3DH Low Power Mode.
 *
 * @param   enable - Enable or disable Low Power Mode.
 */
static bool Lis3dh_setLowPowerMode(bool enable) {
    uint8_t ctrl_reg1_data;

    if (!Lis3dh_readRegister(LIS3DH_CTRL_REG1, &ctrl_reg1_data)) {
        return false;
    }

    // Clear and write new value
    ctrl_reg1_data &= ~LIS3DH_LOW_POWER_MODE_MASK;
    if (enable) ctrl_reg1_data |= LIS3DH_LOW_POWER_MODE_MASK;

    return Lis3dh_writeRegister(LIS3DH_CTRL_REG1, ctrl_reg1_data);
}

/*********************************************************************
 * @fn      Lis3dh_setFifoMode
 *
 * @brief   Set LIS3DH FIFO Mode.
 *
 * @param   fifoMode - FIFO Mode: Bypass, FIFO, Stream, or Stream and FIFO.
 */
static bool Lis3dh_setFifoMode(fifo_mode_t fifoMode) {
    uint8_t fifo_ctrl_reg_data;

    if (!Lis3dh_readRegister(LIS3DH_FIFO_CTRL_REG, &fifo_ctrl_reg_data)) {
        return false;
    }

    // Clear and write new value
    fifo_ctrl_reg_data &= ~LIS3DH_FIFO_MODE_MASK;
    fifo_ctrl_reg_data |= (fifoMode << 6);

    return Lis3dh_writeRegister(LIS3DH_FIFO_CTRL_REG, fifo_ctrl_reg_data);
}

/*********************************************************************
 * @fn      Lis3dh_enableFifo
 *
 * @brief   Enable LIS3DH FIFO.
 *
 * @param   enable - Enable or disable FIFO.
 */
static bool Lis3dh_enableFifo(bool enable) {
    uint8_t ctrl_reg5_data;

    if (!Lis3dh_readRegister(LIS3DH_CTRL_REG5, &ctrl_reg5_data)) {
        return false;
    }

    // Clear and write new value
    ctrl_reg5_data &= ~LIS3DH_FIFO_ENABLE_MASK;
    if (enable) ctrl_reg5_data |= LIS3DH_FIFO_ENABLE_MASK;

    return Lis3dh_writeRegister(LIS3DH_CTRL_REG5, ctrl_reg5_data);
}

/*********************************************************************
 * @fn      Lis3dh_isFifoFull
 *
 * @brief   Check if the LIS3DH FIFO is full.
 *
 * @param   full - Boolean indicating whether the FIFO is full or not.
 */
static bool Lis3dh_isFifoFull(bool *full) {
    uint8_t fifo_src_reg_data;

    if (!Lis3dh_readRegister(LIS3DH_FIFO_SRC_REG, &fifo_src_reg_data)) {
        return false;
    }

    (*full) = (fifo_src_reg_data & LIS3DH_FIFO_OVRN_MASK) > 0;

    return true;
}

/*********************************************************************
 * @fn      Lis3dh_isFifoEmpty
 *
 * @brief   Check if the LIS3DH FIFO is empty.
 *
 * @param   empty - Boolean indicating whether the FIFO is empty or not.
 */
static bool Lis3dh_isFifoEmpty(bool *empty) {
    uint8_t fifo_src_reg_data;

    if (!Lis3dh_readRegister(LIS3DH_FIFO_SRC_REG, &fifo_src_reg_data)) {
        return false;
    }

    (*empty) = (fifo_src_reg_data & LIS3DH_FIFO_EMPTY_MASK) > 0;

    return true;
}

/*********************************************************************
 * @fn      Lis3dh_getFullScaleSelection
 *
 * @brief   Get the full-scale range of the accelerometer data.
 *
 * @param   scale - Range of the accelerometer data.
 */
static bool Lis3dh_getFullScaleSelection(int *scale) {
    uint8_t ctrl_reg4;
    uint8_t range_bits;
    if (!Lis3dh_readRegister(LIS3DH_CTRL_REG4, &ctrl_reg4)) {
        Log_error0("Error getting the full-scale range selection");
        return false;
    }

    range_bits = (ctrl_reg4 & 0x30) >> 4;
    if (range_bits == 0x00) {
        (*scale) = 2;
    }
    else if (range_bits == 0x01) {
        (*scale) = 4;
    }
    else if (range_bits == 0x02) {
        (*scale) = 8;
    }
    else if (range_bits == 0x03) {
        (*scale) = 16;
    }

    return true;
}

/*********************************************************************
 * @fn      Lis3dh_configureFreeFallInterrupt
 *
 * @brief   Configure interrupt for free-fall events.
 *
 * @param   enable - Enable or disable.
 *          threshold - Inertial interrupt threshold.
 *          duration - Number of samples exceeding threshold required.
 */
static bool Lis3dh_configureFreeFallInterrupt(bool enable, float threshold, int duration) {
    uint8_t options = LIS3DH_AOI | LIS3DH_X_LOW | LIS3DH_Y_LOW | LIS3DH_Z_LOW;

    // Set IA1 enable bit (LIS3DH_CTRL_REG3)
    uint8_t ctrl_reg3;
    if(!Lis3dh_readRegister(LIS3DH_CTRL_REG3, &ctrl_reg3)) {
        Log_error0("Error getting the INT1 IA1 enable bit");
        return false;
    }

    ctrl_reg3 &= ~(0x01 << 6);
    if (enable) {
        ctrl_reg3 |= (0x01 << 6);
    }

    if (!Lis3dh_writeRegister(LIS3DH_CTRL_REG3, ctrl_reg3)) {
        Log_error0("Error setting the INT1 IA1 enable bit");
        return false;
    }

    // If disabling, do not continue
    if (!enable) {
        return true;
    }

    // Set the threshold (LIS3DH_INT1_THS)
    int scale;
    if (!Lis3dh_getFullScaleSelection(&scale)) {
        Log_error0("Error getting scale");
        return false;
    }

    uint8_t t = (int)((((threshold * 1.0) / (scale)) * 127));
    if (!Lis3dh_writeRegister(LIS3DH_INT1_THS, (t & 0x7F))) {
        Log_error0("Error setting the INT1 threshold");
        return false;
    }

    // Set the duration (LIS3DH_INT1_DURATION)
    if (!Lis3dh_writeRegister(LIS3DH_INT1_DURATION, (duration & 0x7F))) {
        Log_error0("Error setting INT1 duration");
        return false;
    }

    // Set the options (LIS3DH_INT1_CFG)
    if (!Lis3dh_writeRegister(LIS3DH_INT1_CFG, options)) {
        Log_error0("Error setting INT1 options");
        return false;
    }

    return true;
}

