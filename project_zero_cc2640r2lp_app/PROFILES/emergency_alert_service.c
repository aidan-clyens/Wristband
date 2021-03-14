/**********************************************************************************************
 * Filename:       emergency_alert_service.c
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

#include "emergency_alert_service.h"

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

// emergency_alert_service Service UUID
CONST uint8_t emergency_alert_serviceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(EMERGENCY_ALERT_SERVICE_SERV_UUID), HI_UINT16(EMERGENCY_ALERT_SERVICE_SERV_UUID)
};

// alertType UUID
CONST uint8_t emergency_alert_service_AlertTypeUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(EMERGENCY_ALERT_SERVICE_ALERTTYPE_UUID)
};
// alertActive UUID
CONST uint8_t emergency_alert_service_AlertActiveUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(EMERGENCY_ALERT_SERVICE_ALERTACTIVE_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static emergency_alert_serviceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t emergency_alert_serviceDecl = { ATT_BT_UUID_SIZE, emergency_alert_serviceUUID };

// Characteristic "AlertType" Properties (for declaration)
static uint8_t emergency_alert_service_AlertTypeProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "AlertType" Value variable
static uint8_t emergency_alert_service_AlertTypeVal[EMERGENCY_ALERT_SERVICE_ALERTTYPE_LEN] = {0};

// Characteristic "AlertType" CCCD
static gattCharCfg_t *emergency_alert_service_AlertTypeConfig;
// Characteristic "AlertActive" Properties (for declaration)
static uint8_t emergency_alert_service_AlertActiveProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP | GATT_PROP_NOTIFY;

// Characteristic "AlertActive" Value variable
static uint8_t emergency_alert_service_AlertActiveVal[EMERGENCY_ALERT_SERVICE_ALERTACTIVE_LEN] = {0};

// Characteristic "AlertActive" CCCD
static gattCharCfg_t *emergency_alert_service_AlertActiveConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t emergency_alert_serviceAttrTbl[] =
{
  // emergency_alert_service Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&emergency_alert_serviceDecl
  },
    // AlertType Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &emergency_alert_service_AlertTypeProps
    },
      // AlertType Characteristic Value
      {
        { ATT_UUID_SIZE, emergency_alert_service_AlertTypeUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        emergency_alert_service_AlertTypeVal
      },
      // AlertType CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&emergency_alert_service_AlertTypeConfig
      },
    // AlertActive Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &emergency_alert_service_AlertActiveProps
    },
      // AlertActive Characteristic Value
      {
        { ATT_UUID_SIZE, emergency_alert_service_AlertActiveUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        emergency_alert_service_AlertActiveVal
      },
      // AlertActive CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&emergency_alert_service_AlertActiveConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t emergency_alert_service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t emergency_alert_service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t emergency_alert_serviceCBs =
{
  emergency_alert_service_ReadAttrCB,  // Read callback function pointer
  emergency_alert_service_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * Emergency_alert_service_AddService- Initializes the Emergency_alert_service service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Emergency_alert_service_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  emergency_alert_service_AlertTypeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( emergency_alert_service_AlertTypeConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, emergency_alert_service_AlertTypeConfig );
  // Allocate Client Characteristic Configuration table
  emergency_alert_service_AlertActiveConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( emergency_alert_service_AlertActiveConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, emergency_alert_service_AlertActiveConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( emergency_alert_serviceAttrTbl,
                                        GATT_NUM_ATTRS( emergency_alert_serviceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &emergency_alert_serviceCBs );

  return ( status );
}

/*
 * Emergency_alert_service_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t Emergency_alert_service_RegisterAppCBs( emergency_alert_serviceCBs_t *appCallbacks )
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
 * Emergency_alert_service_SetParameter - Set a Emergency_alert_service parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Emergency_alert_service_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case EMERGENCY_ALERT_SERVICE_ALERTTYPE_ID:
      if ( len == EMERGENCY_ALERT_SERVICE_ALERTTYPE_LEN )
      {
        memcpy(emergency_alert_service_AlertTypeVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( emergency_alert_service_AlertTypeConfig, (uint8_t *)&emergency_alert_service_AlertTypeVal, FALSE,
                                    emergency_alert_serviceAttrTbl, GATT_NUM_ATTRS( emergency_alert_serviceAttrTbl ),
                                    INVALID_TASK_ID,  emergency_alert_service_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case EMERGENCY_ALERT_SERVICE_ALERTACTIVE_ID:
      if ( len == EMERGENCY_ALERT_SERVICE_ALERTACTIVE_LEN )
      {
        memcpy(emergency_alert_service_AlertActiveVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( emergency_alert_service_AlertActiveConfig, (uint8_t *)&emergency_alert_service_AlertActiveVal, FALSE,
                                    emergency_alert_serviceAttrTbl, GATT_NUM_ATTRS( emergency_alert_serviceAttrTbl ),
                                    INVALID_TASK_ID,  emergency_alert_service_ReadAttrCB);
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
 * Emergency_alert_service_GetParameter - Get a Emergency_alert_service parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Emergency_alert_service_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case EMERGENCY_ALERT_SERVICE_ALERTTYPE_ID:
      memcpy(value, emergency_alert_service_AlertTypeVal, EMERGENCY_ALERT_SERVICE_ALERTTYPE_LEN);
      break;

    case EMERGENCY_ALERT_SERVICE_ALERTACTIVE_ID:
      memcpy(value, emergency_alert_service_AlertActiveVal, EMERGENCY_ALERT_SERVICE_ALERTACTIVE_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          emergency_alert_service_ReadAttrCB
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
static bStatus_t emergency_alert_service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the AlertType Characteristic Value
if ( ! memcmp(pAttr->type.uuid, emergency_alert_service_AlertTypeUUID, pAttr->type.len) )
  {
    if ( offset > EMERGENCY_ALERT_SERVICE_ALERTTYPE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, EMERGENCY_ALERT_SERVICE_ALERTTYPE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the AlertActive Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, emergency_alert_service_AlertActiveUUID, pAttr->type.len) )
  {
    if ( offset > EMERGENCY_ALERT_SERVICE_ALERTACTIVE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, EMERGENCY_ALERT_SERVICE_ALERTACTIVE_LEN - offset);  // Transmit as much as possible
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
 * @fn      emergency_alert_service_WriteAttrCB
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
static bStatus_t emergency_alert_service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
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
  // See if request is regarding the AlertActive Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, emergency_alert_service_AlertActiveUUID, pAttr->type.len) )
  {
    if ( offset + len > EMERGENCY_ALERT_SERVICE_ALERTACTIVE_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == EMERGENCY_ALERT_SERVICE_ALERTACTIVE_LEN)
        paramID = EMERGENCY_ALERT_SERVICE_ALERTACTIVE_ID;
    }
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
      uint16_t svcUuid = EMERGENCY_ALERT_SERVICE_SERV_UUID;
      pAppCBs->pfnChangeCb(connHandle, svcUuid, paramID, pValue, len); // Call app function from stack task context.
    }
  return status;
}
