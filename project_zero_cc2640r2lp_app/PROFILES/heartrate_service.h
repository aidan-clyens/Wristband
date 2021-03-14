/**********************************************************************************************
 * Filename:       heartrate_service.h
 *
 * Description:    This file contains the heartrate_service service definitions and
 *                 prototypes.
 *
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


#ifndef _HEARTRATE_SERVICE_H_
#define _HEARTRATE_SERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define HEARTRATE_SERVICE_SERV_UUID 0xA000

//  Characteristic defines
#define HEARTRATE_SERVICE_HEARTRATEVALUE_ID   0
#define HEARTRATE_SERVICE_HEARTRATEVALUE_UUID 0xA001
#define HEARTRATE_SERVICE_HEARTRATEVALUE_LEN  2

//  Characteristic defines
#define HEARTRATE_SERVICE_HEARTRATECONFIDENCE_ID   1
#define HEARTRATE_SERVICE_HEARTRATECONFIDENCE_UUID 0xA002
#define HEARTRATE_SERVICE_HEARTRATECONFIDENCE_LEN  1

//  Characteristic defines
#define HEARTRATE_SERVICE_SPO2VALUE_ID   2
#define HEARTRATE_SERVICE_SPO2VALUE_UUID 0xA003
#define HEARTRATE_SERVICE_SPO2VALUE_LEN  2

//  Characteristic defines
#define HEARTRATE_SERVICE_SPO2CONFIDENCE_ID   3
#define HEARTRATE_SERVICE_SPO2CONFIDENCE_UUID 0xA004
#define HEARTRATE_SERVICE_SPO2CONFIDENCE_LEN  1

//  Characteristic defines
#define HEARTRATE_SERVICE_SCDSTATE_ID   4
#define HEARTRATE_SERVICE_SCDSTATE_UUID 0xA005
#define HEARTRATE_SERVICE_SCDSTATE_LEN  1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*heartrate_serviceChange_t)( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

typedef struct
{
  heartrate_serviceChange_t        pfnChangeCb;  // Called when characteristic value changes
  heartrate_serviceChange_t        pfnCfgChangeCb;
} heartrate_serviceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * Heartrate_service_AddService- Initializes the Heartrate_service service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Heartrate_service_AddService( uint8_t rspTaskId);

/*
 * Heartrate_service_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Heartrate_service_RegisterAppCBs( heartrate_serviceCBs_t *appCallbacks );

/*
 * Heartrate_service_SetParameter - Set a Heartrate_service parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Heartrate_service_SetParameter(uint8_t param, uint16_t len, void *value);

/*
 * Heartrate_service_GetParameter - Get a Heartrate_service parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Heartrate_service_GetParameter(uint8_t param, uint16_t *len, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _HEARTRATE_SERVICE_H_ */
