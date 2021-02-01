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

static uint8_t txBuffer[UTIL_I2C_BUFFER_SIZE];
static uint8_t rxBuffer[UTIL_I2C_BUFFER_SIZE];

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

