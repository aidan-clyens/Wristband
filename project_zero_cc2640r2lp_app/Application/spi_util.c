/*
 * spi_util.c
 *
 *  Created on: Mar 14, 2021
 *      Author: aidan
 */

/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/SPI.h>

#include <uartlog/UartLog.h>
#include <Board.h>

#include "spi_util.h"

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
static SPI_Handle spiHandle;


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Util_spiInit
 *
 * @brief   Initialization the master SPI connection with device.
 */
bool Util_spiInit(void) {
    // Configure SPI
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.mode = SPI_MASTER;
    spiParams.transferMode = SPI_MODE_BLOCKING;
    spiParams.frameFormat = SPI_POL0_PHA0;
    spiParams.bitRate = 400000;
    spiParams.dataSize = 8;

    SPI_init();

    spiHandle = SPI_open(Board_SPI_MASTER, &spiParams);
    if (spiHandle == NULL) {
        Log_error0("SPI initialization failed");
        return false;
    }

    Log_info0("SPI initialized");
    return true;
}

/*********************************************************************
 * @fn      Util_spiTransfer
 *
 * @brief   Start SPI transfer with slave device.
 *
 * @param   transaction - SPI transaction containing read buffer and write buffer
 */
bool Util_spiTransfer(SPI_Transaction *transaction) {
    return SPI_transfer(spiHandle, transaction);
}

