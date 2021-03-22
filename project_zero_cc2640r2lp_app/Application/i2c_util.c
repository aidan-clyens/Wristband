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

#ifdef CC2640R2_LAUNCHXL
#include <Board.h>
#else
#include <Board_PCB.h>
#endif

#include "i2c_util.h"

/*********************************************************************
 * DEFINES
 */

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
static I2C_Handle i2cHandle;


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Util_i2cInit
 *
 * @brief   Initialization the master I2C connection with device.
 */
bool Util_i2cInit(void) {
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

    Log_info0("I2C initialized");
    return true;
}

/*********************************************************************
 * @fn      Util_i2cTransfer
 *
 * @brief   Start I2C transfer with slave device.
 *
 * @param   transaction - I2C transaction containing read buffer and write buffer
 */
bool Util_i2cTransfer(I2C_Transaction *transaction) {
    return I2C_transfer(i2cHandle, transaction);
}

