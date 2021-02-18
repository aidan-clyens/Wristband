/*
 * i2c_util.h
 *
 *  Created on: Jan 29, 2021
 *      Author: aidan
 */

#ifndef APPLICATION_I2C_UTIL_H_
#define APPLICATION_I2C_UTIL_H_


/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/I2C.h>

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
extern bool Util_i2cInit(void);
extern bool Util_i2cTransfer(I2C_Transaction *transaction);


#endif /* APPLICATION_I2C_UTIL_H_ */
