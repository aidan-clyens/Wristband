/**********************************************************************************************
 * Filename:       heartrate_service.c
 *
 * Description:    This file contains the implementation of the service.
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


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "heartrate_service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// heartrate_service Service UUID
CONST uint8_t heartrate_serviceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HEARTRATE_SERVICE_SERV_UUID), HI_UINT16(HEARTRATE_SERVICE_SERV_UUID)
};

// heartRateValue UUID
CONST uint8_t heartrate_service_HeartRateValueUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(HEARTRATE_SERVICE_HEARTRATEVALUE_UUID)
};
// spO2Value UUID
CONST uint8_t heartrate_service_SpO2ValueUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(HEARTRATE_SERVICE_SPO2VALUE_UUID)
};
// statusValue UUID
CONST uint8_t heartrate_service_StatusValueUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(HEARTRATE_SERVICE_STATUSVALUE_UUID)
};
// confidenceValue UUID
CONST uint8_t heartrate_service_ConfidenceValueUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(HEARTRATE_SERVICE_CONFIDENCEVALUE_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static heartrate_serviceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t heartrate_serviceDecl = { ATT_BT_UUID_SIZE, heartrate_serviceUUID };

// Characteristic "HeartRateValue" Properties (for declaration)
static uint8_t heartrate_service_HeartRateValueProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "HeartRateValue" Value variable
static uint8_t heartrate_service_HeartRateValueVal[HEARTRATE_SERVICE_HEARTRATEVALUE_LEN] = {0};

// Characteristic "HeartRateValue" CCCD
static gattCharCfg_t *heartrate_service_HeartRateValueConfig;
// Characteristic "SpO2Value" Properties (for declaration)
static uint8_t heartrate_service_SpO2ValueProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "SpO2Value" Value variable
static uint8_t heartrate_service_SpO2ValueVal[HEARTRATE_SERVICE_SPO2VALUE_LEN] = {0};

// Characteristic "SpO2Value" CCCD
static gattCharCfg_t *heartrate_service_SpO2ValueConfig;
// Characteristic "StatusValue" Properties (for declaration)
static uint8_t heartrate_service_StatusValueProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "StatusValue" Value variable
static uint8_t heartrate_service_StatusValueVal[HEARTRATE_SERVICE_STATUSVALUE_LEN] = {0};

// Characteristic "StatusValue" CCCD
static gattCharCfg_t *heartrate_service_StatusValueConfig;
// Characteristic "ConfidenceValue" Properties (for declaration)
static uint8_t heartrate_service_ConfidenceValueProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "ConfidenceValue" Value variable
static uint8_t heartrate_service_ConfidenceValueVal[HEARTRATE_SERVICE_CONFIDENCEVALUE_LEN] = {0};

// Characteristic "ConfidenceValue" CCCD
static gattCharCfg_t *heartrate_service_ConfidenceValueConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t heartrate_serviceAttrTbl[] =
{
  // heartrate_service Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&heartrate_serviceDecl
  },
    // HeartRateValue Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &heartrate_service_HeartRateValueProps
    },
      // HeartRateValue Characteristic Value
      {
        { ATT_UUID_SIZE, heartrate_service_HeartRateValueUUID },
        GATT_PERMIT_READ,
        0,
        heartrate_service_HeartRateValueVal
      },
      // HeartRateValue CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&heartrate_service_HeartRateValueConfig
      },
    // SpO2Value Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &heartrate_service_SpO2ValueProps
    },
      // SpO2Value Characteristic Value
      {
        { ATT_UUID_SIZE, heartrate_service_SpO2ValueUUID },
        GATT_PERMIT_READ,
        0,
        heartrate_service_SpO2ValueVal
      },
      // SpO2Value CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&heartrate_service_SpO2ValueConfig
      },
    // StatusValue Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &heartrate_service_StatusValueProps
    },
      // StatusValue Characteristic Value
      {
        { ATT_UUID_SIZE, heartrate_service_StatusValueUUID },
        GATT_PERMIT_READ,
        0,
        heartrate_service_StatusValueVal
      },
      // StatusValue CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&heartrate_service_StatusValueConfig
      },
    // ConfidenceValue Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &heartrate_service_ConfidenceValueProps
    },
      // ConfidenceValue Characteristic Value
      {
        { ATT_UUID_SIZE, heartrate_service_ConfidenceValueUUID },
        GATT_PERMIT_READ,
        0,
        heartrate_service_ConfidenceValueVal
      },
      // ConfidenceValue CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&heartrate_service_ConfidenceValueConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t heartrate_service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t heartrate_service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t heartrate_serviceCBs =
{
  heartrate_service_ReadAttrCB,  // Read callback function pointer
  heartrate_service_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * Heartrate_service_AddService- Initializes the Heartrate_service service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Heartrate_service_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  heartrate_service_HeartRateValueConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( heartrate_service_HeartRateValueConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( CONNHANDLE_INVALID, heartrate_service_HeartRateValueConfig );
  // Allocate Client Characteristic Configuration table
  heartrate_service_SpO2ValueConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( heartrate_service_SpO2ValueConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( CONNHANDLE_INVALID, heartrate_service_SpO2ValueConfig );
  // Allocate Client Characteristic Configuration table
  heartrate_service_StatusValueConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( heartrate_service_StatusValueConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( CONNHANDLE_INVALID, heartrate_service_StatusValueConfig );
  // Allocate Client Characteristic Configuration table
  heartrate_service_ConfidenceValueConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( heartrate_service_ConfidenceValueConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( CONNHANDLE_INVALID, heartrate_service_ConfidenceValueConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( heartrate_serviceAttrTbl,
                                        GATT_NUM_ATTRS( heartrate_serviceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &heartrate_serviceCBs );

  return ( status );
}

/*
 * Heartrate_service_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t Heartrate_service_RegisterAppCBs( heartrate_serviceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

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
bStatus_t Heartrate_service_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case HEARTRATE_SERVICE_HEARTRATEVALUE_ID:
      if ( len == HEARTRATE_SERVICE_HEARTRATEVALUE_LEN )
      {
        memcpy(heartrate_service_HeartRateValueVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( heartrate_service_HeartRateValueConfig, (uint8_t *)&heartrate_service_HeartRateValueVal, FALSE,
                                    heartrate_serviceAttrTbl, GATT_NUM_ATTRS( heartrate_serviceAttrTbl ),
                                    INVALID_TASK_ID,  heartrate_service_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case HEARTRATE_SERVICE_SPO2VALUE_ID:
      if ( len == HEARTRATE_SERVICE_SPO2VALUE_LEN )
      {
        memcpy(heartrate_service_SpO2ValueVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( heartrate_service_SpO2ValueConfig, (uint8_t *)&heartrate_service_SpO2ValueVal, FALSE,
                                    heartrate_serviceAttrTbl, GATT_NUM_ATTRS( heartrate_serviceAttrTbl ),
                                    INVALID_TASK_ID,  heartrate_service_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case HEARTRATE_SERVICE_STATUSVALUE_ID:
      if ( len == HEARTRATE_SERVICE_STATUSVALUE_LEN )
      {
        memcpy(heartrate_service_StatusValueVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( heartrate_service_StatusValueConfig, (uint8_t *)&heartrate_service_StatusValueVal, FALSE,
                                    heartrate_serviceAttrTbl, GATT_NUM_ATTRS( heartrate_serviceAttrTbl ),
                                    INVALID_TASK_ID,  heartrate_service_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case HEARTRATE_SERVICE_CONFIDENCEVALUE_ID:
      if ( len == HEARTRATE_SERVICE_CONFIDENCEVALUE_LEN )
      {
        memcpy(heartrate_service_ConfidenceValueVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( heartrate_service_ConfidenceValueConfig, (uint8_t *)&heartrate_service_ConfidenceValueVal, FALSE,
                                    heartrate_serviceAttrTbl, GATT_NUM_ATTRS( heartrate_serviceAttrTbl ),
                                    INVALID_TASK_ID,  heartrate_service_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * Heartrate_service_GetParameter - Get a Heartrate_service parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Heartrate_service_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          heartrate_service_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t heartrate_service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the HeartRateValue Characteristic Value
if ( ! memcmp(pAttr->type.uuid, heartrate_service_HeartRateValueUUID, pAttr->type.len) )
  {
    if ( offset > HEARTRATE_SERVICE_HEARTRATEVALUE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, HEARTRATE_SERVICE_HEARTRATEVALUE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the SpO2Value Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, heartrate_service_SpO2ValueUUID, pAttr->type.len) )
  {
    if ( offset > HEARTRATE_SERVICE_SPO2VALUE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, HEARTRATE_SERVICE_SPO2VALUE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the StatusValue Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, heartrate_service_StatusValueUUID, pAttr->type.len) )
  {
    if ( offset > HEARTRATE_SERVICE_STATUSVALUE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, HEARTRATE_SERVICE_STATUSVALUE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the ConfidenceValue Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, heartrate_service_ConfidenceValueUUID, pAttr->type.len) )
  {
    if ( offset > HEARTRATE_SERVICE_CONFIDENCEVALUE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, HEARTRATE_SERVICE_CONFIDENCEVALUE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      heartrate_service_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t heartrate_service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
    {
      uint16_t svcUuid = HEARTRATE_SERVICE_SERV_UUID;
      pAppCBs->pfnChangeCb(connHandle, svcUuid, paramID, len, pValue); // Call app function from stack task context.
    }
  return status;
}
