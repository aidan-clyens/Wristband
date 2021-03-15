/*
 * spi_util.h
 *
 *  Created on: Mar 14, 2021
 *      Author: aidan
 */

#ifndef APPLICATION_SPI_UTIL_H_
#define APPLICATION_SPI_UTIL_H_

/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/SPI.h>

/*********************************************************************
 *  EXTERNAL VARIABLES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
extern bool Util_spiInit(void);
extern bool Util_spiTransfer(SPI_Transaction *transaction);


#endif /* APPLICATION_SPI_UTIL_H_ */
