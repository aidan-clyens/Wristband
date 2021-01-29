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

/*********************************************************************
 * @fn      Util_i2cInit
 *
 * @brief   Initialization the master I2C connection with device.
 */
extern bool Util_i2cInit(void);

/*********************************************************************
 * @fn      Util_i2cBeginTransmission
 *
 * @brief   Begin I2C data transmission.
 *
 * @param   address - Address of slave device.
 */
extern void Util_i2cBeginTransmission(uint8_t address);

/*********************************************************************
 * @fn      Util_i2cEndTransmission
 *
 * @brief   End I2C data transmission.
 */
extern bool Util_i2cEndTransmission(void);

/*********************************************************************
 * @fn      Util_i2cWrite
 *
 * @brief   Write a byte to transmit over I2C.
 */
extern void Util_i2cWrite(uint8_t data);

/*********************************************************************
 * @fn      Util_i2cReadRequest
 *
 * @brief   Write a byte to transmit over I2C.
 */
extern void Util_i2cReadRequest(int num_bytes);

/*********************************************************************
 * @fn      Util_i2cRead
 *
 * @brief   Read a byte to over I2C.
 */
extern uint8_t Util_i2cRead(void);

/*********************************************************************
 * @fn      Util_i2cAvailable
 *
 * @brief   Check how many data bytes have been received over I2C.
 */
extern int Util_i2cAvailable(void);

#endif /* APPLICATION_I2C_UTIL_H_ */
