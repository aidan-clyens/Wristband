/*
 * i2c_util.c
 *
 *  Created on: Jan 29, 2021
 *      Author: aidan
 */

/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/I2C.h>

#include <uartlog/UartLog.h>
#include <Board.h>

#include "i2c_util.h"

/*********************************************************************
 * DEFINES
 */
#define UTIL_I2C_BUFFER_SIZE        32

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// I2C
static I2C_Handle i2cHandle;
static I2C_Transaction transaction;
static uint8_t txBuffer[UTIL_I2C_BUFFER_SIZE];
static uint8_t rxBuffer[UTIL_I2C_BUFFER_SIZE];

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Util_i2cInit
 *
 * @brief   Initialization the master I2C connection with device.
 */
bool Util_i2cInit(void)
{
    I2C_Params i2cParams;

    I2C_init();

    // Configure I2C
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cHandle = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2cHandle == NULL) {
        Log_error0("I2C initialization failed!");
        return false;
    }
    Log_info0("I2C initialized");
    return true;
}

/*********************************************************************
 * @fn      Util_i2cBeginTransmission
 *
 * @brief   Begin I2C data transmission.
 *
 * @param   address - Address of slave device.
 */
void Util_i2cBeginTransmission(uint8_t address)
{
    transaction.writeBuf = txBuffer;
    transaction.readBuf = rxBuffer;
    transaction.readCount = 0;
    transaction.writeCount = 0;
    transaction.slaveAddress = address;
}

/*********************************************************************
 * @fn      Util_i2cEndTransmission
 *
 * @brief   End I2C data transmission.
 */
bool Util_i2cEndTransmission(void)
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
 * @fn      Util_i2cWrite
 *
 * @brief   Write a byte to transmit over I2C.
 */
void Util_i2cWrite(uint8_t data)
{
    txBuffer[transaction.writeCount] = data;
    transaction.writeCount++;
}

/*********************************************************************
 * @fn      Util_i2cReadRequest
 *
 * @brief   Write a byte to transmit over I2C.
 */
void Util_i2cReadRequest(int num_bytes)
{
    transaction.readCount = num_bytes;
}

/*********************************************************************
 * @fn      Util_i2cRead
 *
 * @brief   Read a byte to over I2C.
 */
uint8_t Util_i2cRead(void)
{
    uint8_t data = -1;
    if (transaction.readCount > 0) {
        data = rxBuffer[transaction.readCount-1];
        transaction.readCount--;
    }

    return data;
}

/*********************************************************************
 * @fn      Util_i2cAvailable
 *
 * @brief   Check how many data bytes have been received over I2C.
 */
int Util_i2cAvailable(void)
{
    return transaction.readCount;
}
