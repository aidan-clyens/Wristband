/******************************************************************************

 @file  project_zero.c

 @brief This file contains the Project Zero sample application implementation

 Group: CMCU, LPRF
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2013-2021, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>

#include <xdc/runtime/Diags.h>
#include <uartlog/UartLog.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include <icall.h>

#include <osal_snv.h>
#include <peripheral.h>
#include <devinfoservice.h>

#include "util.h"

#ifdef CC2640R2_LAUNCHXL
#include <Board.h>
#else
#include <Board_PCB.h>
#endif

#include "project_zero.h"

// Bluetooth Developer Studio services
#include "config_service.h"
#include "heartrate_service.h"
#include "emergency_alert_service.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Default pass-code used for pairing.
#define DEFAULT_PASSCODE                      000000

// Task configuration
#define PRZ_TASK_PRIORITY                     2

#ifndef PRZ_TASK_STACK_SIZE
#define PRZ_TASK_STACK_SIZE                   512
#endif

// Internal Events for RTOS application
#define PRZ_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define PRZ_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define PRZ_STATE_CHANGE_EVT                  Event_Id_00
#define PRZ_CHAR_CHANGE_EVT                   Event_Id_01
#define PRZ_PERIODIC_EVT                      Event_Id_02
#define PRZ_APP_MSG_EVT                       Event_Id_03

#define PRZ_ALL_EVENTS                       (PRZ_ICALL_EVT        | \
                                              PRZ_QUEUE_EVT        | \
                                              PRZ_STATE_CHANGE_EVT | \
                                              PRZ_CHAR_CHANGE_EVT  | \
                                              PRZ_PERIODIC_EVT     | \
                                              PRZ_APP_MSG_EVT)

// Period to update RSSI of connected device
#define PRZ_RSSI_UPDATE_PERIOD_MS             5000

// Period to blink LED
#define PRZ_LED_BLINK_PERIOD_MS               100

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(registerCause) (connectionEventRegisterCauseBitMap |= registerCause )

// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(registerCause) (connectionEventRegisterCauseBitMap &= (~registerCause) )

// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)

// Gets whether the registerCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(registerCause) (connectionEventRegisterCauseBitMap & registerCause )

/*********************************************************************
 * TYPEDEFS
 */
// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
typedef enum
{
  APP_MSG_SERVICE_WRITE = 0,   /* A characteristic value has been written     */
  APP_MSG_SERVICE_CFG,         /* A characteristic configuration has changed  */
  APP_MSG_UPDATE_CHARVAL,      /* Request from ourselves to update a value    */
  APP_MSG_GAP_STATE_CHANGE,    /* The GAP / connection state has changed      */
  APP_MSG_BUTTON_DEBOUNCED,    /* A button has been debounced with new value  */
  APP_MSG_SEND_PASSCODE,       /* A pass-code/PIN is requested during pairing */
  APP_MSG_PRZ_CONN_EVT,        /* Connection Event finished report            */
  APP_MSG_UPDATE_RSSI,         /* Update RSSI of connected devices            */
  APP_MSG_LED_EVT,             /* Change value of LEDs                        */
  APP_MSG_LED_BLINK,           /* Blink LED                                   */
} app_msg_types_t;

// State indicated LED change
typedef enum
{
    LED_BLE_CONNECTED,
    LED_BLE_DISCONNECTED,
    LED_HEARTRATE_READING,
    LED_HEARTRATE_COMPLETE,
    LED_HEARTRATE_UNDETECTED,
} led_event_t;

// Struct for messages sent to the application task
typedef struct
{
  Queue_Elem       _elem;
  app_msg_types_t  type;
  uint8_t          pdu[];
} app_msg_t;

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct
{
  uint16_t connHandle;
  uint8_t  uiInputs;
  uint8_t  uiOutputs;
  uint32   numComparison;
} passcode_req_t;

// Struct for message about button state
typedef struct
{
  PIN_Id   pinId;
  uint8_t  state;
} button_state_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

// Task configuration
Task_Struct przTask;
Char przTaskStack[PRZ_TASK_STACK_SIZE];


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // No scan response data provided.
  0x00 // Placeholder to keep the compiler happy.
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) or general
  // discoverable mode (advertises indefinitely), depending
  // on the DEFAULT_DISCOVERY_MODE define.
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // complete name
  16,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'W','r','i','s','t','b','a','n','d'

};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Aidan Wristband";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Handle for connected BLE device
static uint16_t peerConnHandle;

/* Pin driver handles */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;
static PIN_State ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_LED0 & Board_LED1 are off.
 */
PIN_Config ledPinTable[] = {
  Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

// Clock objects for debouncing the buttons
static Clock_Struct button0DebounceClock;
static Clock_Struct button1DebounceClock;

// Clock object for updating RSSI
static Clock_Struct rssiUpdateClock;
static Clock_Handle rssiUpdateClockHandle;

// Clock object for blinking the LED
static Clock_Struct ledBlinkClock;
static Clock_Handle ledBlinkClockHandle;

// State of the buttons
static uint8_t button0State = 0;
static uint8_t button1State = 0;

// Global display handle
Display_Handle dispHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void ProjectZero_init( void );
static void ProjectZero_taskFxn(UArg a0, UArg a1);

static void user_processApplicationMessage(app_msg_t *pMsg);
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);

static void ProjectZero_sendAttRsp(void);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);
static void ProjectZero_freeAttRsp(uint8_t status);

static void ProjectZero_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void ProjectZero_processConnEvt(Gap_ConnEventRpt_t *pReport);
static void ProjectZero_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);

static void user_processGapStateChangeEvt(gaprole_States_t newState);
static void user_gapStateChangeCB(gaprole_States_t newState);
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status);

static void buttonDebounceSwiFxn(UArg buttonId);
static void user_handleButtonPress(button_state_t *pState);

static void user_handleLedEvt(led_event_t event);

// Generic callback handlers for value changes in services.
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

// Task context handlers for generated services.
static void user_Emergency_alert_service_ValueChangeHandler(char_data_t *pCharData);

// Task handler for sending notifications.
static void user_updateCharVal(char_data_t *pCharData);

// Utility functions
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len );
static void user_enqueueCharDataMsg(app_msg_types_t appMsgType, uint16_t connHandle,
                                    uint16_t serviceUUID, uint8_t paramID,
                                    uint8_t *pValue, uint16_t len);
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);

// RSSI update SWI
static void rssiUpdateSwi();

// LED blink SWI
static void ledBlinkSwi();

static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len);
#if defined(UARTLOG_ENABLE)
static char *Util_getLocalNameStr(const uint8_t *data);
#endif
/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t user_gapRoleCBs =
{
  user_gapStateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t user_bondMgrCBs =
{
  user_gapBondMgr_passcodeCB, // Passcode callback
  user_gapBondMgr_pairStateCB // Pairing / Bonding state Callback
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */
static config_serviceCBs_t user_Config_ServiceCBs =
{
  .pfnChangeCb    = NULL, // No writable chars in Config Service, so no change handler.
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

static heartrate_serviceCBs_t user_Heartrate_ServiceCBs =
{
  .pfnChangeCb    = NULL, // No writable chars in Heartrate Service, so no change handler.
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

static emergency_alert_serviceCBs_t user_EmergencyAlert_ServiceCBs =
{
  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum
{
   NONE_REGISTERED    = 0,
   FOR_ATT_RSP        = 1,
} connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t connectionEventRegisterCauseBitMap = NONE_REGISTERED; // See connectionEventRegisterCause_u


/*
 * @brief  Register to receive connection event reports for all the connections
 *
 * @param  connectionEventRegisterCause  Represents the reason for registration
 *
 * @return @ref SUCCESS
 */
bStatus_t ProjectZero_RegistertToAllConnectionEvent(connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // In case  there is no registration for the connection event, register for report
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(ProjectZero_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }
  
  if(status == SUCCESS)
  {
    // Add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t ProjectZero_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  
  // In case there are no more subscribers for the connection event then unregister for report
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(ProjectZero_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */
void ProjectZero_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = przTaskStack;
  taskParams.stackSize = PRZ_TASK_STACK_SIZE;
  taskParams.priority = PRZ_TASK_PRIORITY;

  Task_construct(&przTask, ProjectZero_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      ProjectZero_updateHeartRateValue
 *
 * @brief   Update heart rate characteristic value.
 *
 * @param   value - 16-bit heart rate value.
 */
void ProjectZero_updateHeartRateValue(uint16_t value)
{
    uint16_t serviceUUID = HEARTRATE_SERVICE_SERV_UUID;
    uint8_t paramID = HEARTRATE_SERVICE_HEARTRATEVALUE_ID;
    uint16_t len = HEARTRATE_SERVICE_HEARTRATEVALUE_LEN;
    uint8_t pValue[2];
    pValue[1] = value >> 8;
    pValue[0] = value & 0xFF;

    user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0, serviceUUID, paramID, pValue, len);

    // Change LED to indicate heart rate reading complete
    led_event_t event = LED_HEARTRATE_COMPLETE;
    user_enqueueRawAppMsg(APP_MSG_LED_EVT, (uint8_t *)&event, sizeof(event));
}

/*********************************************************************
 * @fn      ProjectZero_updateHeartRateConfidence
 *
 * @brief   Update heart rate confidence characteristic value.
 *
 * @param   value - 8-bit heart rate confidence value.
 */
void ProjectZero_updateHeartRateConfidence(uint8_t value)
{
    uint16_t serviceUUID = HEARTRATE_SERVICE_SERV_UUID;
    uint8_t paramID = HEARTRATE_SERVICE_HEARTRATECONFIDENCE_ID;
    uint16_t len = HEARTRATE_SERVICE_HEARTRATECONFIDENCE_LEN;

    user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0, serviceUUID, paramID, &value, len);
}

/*********************************************************************
 * @fn      ProjectZero_updateSpO2Value
 *
 * @brief   Update SpO2 characteristic value.
 *
 * @param   value - 16-bit SpO2 value.
 */
void ProjectZero_updateSpO2Value(uint16_t value)
{
    uint16_t serviceUUID = HEARTRATE_SERVICE_SERV_UUID;
    uint8_t paramID = HEARTRATE_SERVICE_SPO2VALUE_ID;
    uint16_t len = HEARTRATE_SERVICE_SPO2VALUE_LEN;
    uint8_t pValue[2];
    pValue[1] = value >> 8;
    pValue[0] = value & 0xFF;

    user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0, serviceUUID, paramID, pValue, len);
}

/*********************************************************************
 * @fn      ProjectZero_updateSpO2Confidence
 *
 * @brief   Update SpO2 confidence characteristic value.
 *
 * @param   value - 8-bit SpO2 confidence value.
 */
void ProjectZero_updateSpO2Confidence(uint8_t value)
{
    uint16_t serviceUUID = HEARTRATE_SERVICE_SERV_UUID;
    uint8_t paramID = HEARTRATE_SERVICE_SPO2CONFIDENCE_ID;
    uint16_t len = HEARTRATE_SERVICE_SPO2CONFIDENCE_LEN;

    user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0, serviceUUID, paramID, &value, len);
}

/*********************************************************************
 * @fn      ProjectZero_updateScdState
 *
 * @brief   Update SCD state characteristic value.
 *
 * @param   value - 8-bit SCD state value.
 */
void ProjectZero_updateScdState(uint8_t value)
{
    uint16_t serviceUUID = HEARTRATE_SERVICE_SERV_UUID;
    uint8_t paramID = HEARTRATE_SERVICE_SCDSTATE_ID;
    uint16_t len = HEARTRATE_SERVICE_SCDSTATE_LEN;

    user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0, serviceUUID, paramID, &value, len);

    // Determine LED state during heart rate measurement
    led_event_t event;
    if (value == 0) {
        // Change LED to indicate heart rate undetected
        event = LED_HEARTRATE_UNDETECTED;
        user_enqueueRawAppMsg(APP_MSG_LED_EVT, (uint8_t *)&event, sizeof(event));
    }
    else if (value == 1) {
        // Change LED to indicate heart rate started reading
        event = LED_HEARTRATE_READING;
        user_enqueueRawAppMsg(APP_MSG_LED_EVT, (uint8_t *)&event, sizeof(event));
    }
}

/*********************************************************************
 * @fn      ProjectZero_triggerEmergencyAlert
 *
 * @brief   Trigger an emergency alert to notify the Hub.
 *
 * @param   alertType - Alert type (e.g. manual request (0), fall event(1)).
 */
void ProjectZero_triggerEmergencyAlert(uint8_t alertType)
{
    uint16_t serviceUUID = EMERGENCY_ALERT_SERVICE_SERV_UUID;
    uint8_t paramID = EMERGENCY_ALERT_SERVICE_ALERTACTIVE_ID;
    uint16_t len = EMERGENCY_ALERT_SERVICE_ALERTACTIVE_LEN;
    uint8_t alertActive = 1;
    user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0, serviceUUID, paramID, &alertActive, len);

    paramID = EMERGENCY_ALERT_SERVICE_ALERTTYPE_ID;
    len = EMERGENCY_ALERT_SERVICE_ALERTTYPE_LEN;
    user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0, serviceUUID, paramID, &alertType, len);
}

/*
 * @brief   Called before the task loop and contains application-specific
 *          initialization of the BLE stack, hardware setup, power-state
 *          notification if used, and BLE profile/service initialization.
 *
 * @param   None.
 *
 * @return  None.
 */
static void ProjectZero_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages via ICall to Stack.
  ICall_registerApp(&selfEntity, &syncEvent);

  Log_info0("Initializing the user task, hardware, BLE stack and services.");

  // Open display. By default this is disabled via the predefined symbol Display_DISABLE_ALL.
  dispHandle = Display_open(Display_Type_UART, NULL);

  // Initialize queue for application messages.
  // Note: Used to transfer control to application thread from e.g. interrupts.
  Queue_construct(&applicationMsgQ, NULL);
  hApplicationMsgQ = Queue_handle(&applicationMsgQ);

  // ******************************************************************
  // Hardware initialization
  // ******************************************************************

  // Open LED pins
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);
  if(!ledPinHandle) {
    Log_error0("Error initializing board LED pins");
    Task_exit();
  }

  buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
  if(!buttonPinHandle) {
    Log_error0("Error initializing button pins");
    Task_exit();
  }

  // Setup callback for button pins
  if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
    Log_error0("Error registering button callback function");
    Task_exit();
  }

  // Create the debounce clock objects for Button 0 and Button 1
  Clock_Params clockParams;
  Clock_Params_init(&clockParams);

  // Both clock objects use the same callback, so differentiate on argument
  // given to the callback in Swi context
  clockParams.arg = Board_BUTTON0;

  // Initialize to 50 ms timeout when Clock_start is called.
  // Timeout argument is in ticks, so convert from ms to ticks via tickPeriod.
  Clock_construct(&button0DebounceClock, buttonDebounceSwiFxn,
                  50 * (1000/Clock_tickPeriod),
                  &clockParams);

  // Second button
  clockParams.arg = Board_BUTTON1;
  Clock_construct(&button1DebounceClock, buttonDebounceSwiFxn,
                  50 * (1000/Clock_tickPeriod),
                  &clockParams);

  // Create the clock object for updating RSSI
  rssiUpdateClockHandle = Util_constructClock(&rssiUpdateClock,
                                              rssiUpdateSwi,
                                              PRZ_RSSI_UPDATE_PERIOD_MS,
                                              PRZ_RSSI_UPDATE_PERIOD_MS,
                                              1,
                                              0);

  // Create the clock object for blinking the LED (do not start)
  ledBlinkClockHandle = Util_constructClock(&ledBlinkClock,
                                            ledBlinkSwi,
                                            PRZ_LED_BLINK_PERIOD_MS,
                                            PRZ_LED_BLINK_PERIOD_MS,
                                            0,
                                            0);

  // ******************************************************************
  // BLE Stack initialization
  // ******************************************************************

  // Setup the GAP Peripheral Role Profile
  uint8_t initialAdvertEnable = TRUE;  // Advertise on power-up

  // By setting this to zero, the device will go into the waiting state after
  // being discoverable. Otherwise wait this long [ms] before advertising again.
  uint16_t advertOffTime = 0; // miliseconds

  // Set advertisement enabled.
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &initialAdvertEnable);

  // Configure the wait-time before restarting advertisement automatically
  GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                       &advertOffTime);

  // Initialize Scan Response data
  GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);

  // Initialize Advertisement data
  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

  Log_info1("Name in advertData array: \x1b[33m%s\x1b[0m",
            (IArg)Util_getLocalNameStr(advertData));

  // Set advertising interval
  uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

  // Set duration of advertisement before stopping in Limited adv mode.
  GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 30); // Seconds

  // ******************************************************************
  // BLE Bond Manager initialization
  // ******************************************************************
  uint32_t passkey = 0; // passkey "000000"
  uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
  uint8_t mitm = TRUE;
  uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  uint8_t bonding = TRUE;

  GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                          &passkey);
  GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

  // ******************************************************************
  // BLE Service initialization
  // ******************************************************************

  // Add services to GATT server
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  // Set the device name characteristic in the GAP Profile
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Add services to GATT server and give ID of this task for Indication acks.
  Config_service_AddService( selfEntity );
  Heartrate_service_AddService( selfEntity );
  Emergency_alert_service_AddService( selfEntity );

  // Register callbacks with the generated services that
  // can generate events (writes received) to the application
  Config_service_RegisterAppCBs( &user_Config_ServiceCBs );
  Heartrate_service_RegisterAppCBs( &user_Heartrate_ServiceCBs );
  Emergency_alert_service_RegisterAppCBs( &user_EmergencyAlert_ServiceCBs );

  // Placeholder variable for characteristic intialization
  uint8_t initVal[1] = {0};

  // Initialization of characteristics in Config Service that can provide data.
  Config_service_SetParameter(CONFIG_SERVICE_RSSI_ID, CONFIG_SERVICE_RSSI_LEN, initVal);

  // Initialization of characteristics in Heartrate Service that can provide data.
  Heartrate_service_SetParameter(HEARTRATE_SERVICE_HEARTRATEVALUE_ID, HEARTRATE_SERVICE_HEARTRATEVALUE_LEN, initVal);
  Heartrate_service_SetParameter(HEARTRATE_SERVICE_HEARTRATECONFIDENCE_ID, HEARTRATE_SERVICE_HEARTRATECONFIDENCE_LEN, initVal);
  Heartrate_service_SetParameter(HEARTRATE_SERVICE_SPO2VALUE_ID, HEARTRATE_SERVICE_SPO2VALUE_LEN, initVal);
  Heartrate_service_SetParameter(HEARTRATE_SERVICE_SPO2CONFIDENCE_ID, HEARTRATE_SERVICE_SPO2CONFIDENCE_LEN, initVal);
  Heartrate_service_SetParameter(HEARTRATE_SERVICE_SCDSTATE_ID, HEARTRATE_SERVICE_SCDSTATE_LEN, initVal);

  // Initialization of characteristics in Emergency Alert Service that can provide data.
  Emergency_alert_service_SetParameter(EMERGENCY_ALERT_SERVICE_ALERTTYPE_ID, EMERGENCY_ALERT_SERVICE_ALERTTYPE_LEN, initVal);
  Emergency_alert_service_SetParameter(EMERGENCY_ALERT_SERVICE_ALERTACTIVE_ID, EMERGENCY_ALERT_SERVICE_ALERTACTIVE_LEN, initVal);

  // Start the stack in Peripheral mode.
  VOID GAPRole_StartDevice(&user_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&user_bondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  peerConnHandle = LINKDB_CONNHANDLE_INVALID;
}


/*
 * @brief   Application task entry point.
 *
 *          Invoked by TI-RTOS when BIOS_start is called. Calls an init function
 *          and enters an infinite loop waiting for messages.
 *
 *          Messages can be either directly from the BLE stack or from user code
 *          like Hardware Interrupt (Hwi) or a callback function.
 *
 *          The reason for sending messages to this task from e.g. Hwi's is that
 *          some RTOS and Stack APIs are not available in callbacks and so the
 *          actions that may need to be taken is dispatched to this Task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void ProjectZero_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  ProjectZero_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, PRZ_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Check if we got a signal because of a stack message
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = ProjectZero_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // Process messages sent from another task or another context.
      while (!Queue_empty(hApplicationMsgQ))
      {
        app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);

        // Process application-layer message probably sent from ourselves.
        user_processApplicationMessage(pMsg);

        // Free the received message.
        ICall_free(pMsg);
      }
    }
  }
}


/*
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type app_msg_t.
 *
 * @return  None.
 */
static void user_processApplicationMessage(app_msg_t *pMsg)
{
  char_data_t *pCharData = (char_data_t *)pMsg->pdu;

  switch (pMsg->type)
  {
    case APP_MSG_SERVICE_WRITE: /* Message about received value write */
      /* Call different handler per service */
      switch(pCharData->svcUUID) {
        case EMERGENCY_ALERT_SERVICE_SERV_UUID:
          user_Emergency_alert_service_ValueChangeHandler(pCharData);
          break;
      }
      break;

    case APP_MSG_SERVICE_CFG: /* Message about received CCCD write */
      break;

   case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
     AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
     break;

    case APP_MSG_UPDATE_CHARVAL: /* Message from ourselves to send  */
      user_updateCharVal(pCharData);
      break;

    case APP_MSG_GAP_STATE_CHANGE: /* Message that GAP state changed  */
      user_processGapStateChangeEvt( *(gaprole_States_t *)pMsg->pdu );
      break;

    case APP_MSG_SEND_PASSCODE: /* Message about pairing PIN request */
      {
        passcode_req_t *pReq = (passcode_req_t *)pMsg->pdu;
        Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
                  (IArg)(pReq->uiInputs?"Sending":"Displaying"),
                  DEFAULT_PASSCODE);
        // Send passcode response.
        GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, DEFAULT_PASSCODE);
      }
      break;

    case APP_MSG_BUTTON_DEBOUNCED: /* Message from swi about pin change */
      {
        button_state_t *pButtonState = (button_state_t *)pMsg->pdu;
        user_handleButtonPress(pButtonState);
      }
      break;

    case APP_MSG_PRZ_CONN_EVT:
    {
        ProjectZero_processConnEvt((Gap_ConnEventRpt_t *)pMsg->pdu);
        break;
    }

    case APP_MSG_UPDATE_RSSI:
      {
          if (peerConnHandle != LINKDB_CONNHANDLE_INVALID) {
              Log_info0("Read RSSI");
              HCI_ReadRssiCmd(peerConnHandle);
          }
      }
      break;
    case APP_MSG_LED_EVT:
      {
          led_event_t *ledEvt = (led_event_t *)pMsg->pdu;
          user_handleLedEvt(*ledEvt);
      }
      break;
    case APP_MSG_LED_BLINK:
      {
          int value = PIN_getOutputValue(Board_RLED);
          value = (value == 0) ? 1 : 0;
          PIN_setOutputValue(ledPinHandle, Board_RLED, value);
      }
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of system/application events deferred to the user Task context.
 *  Invoked from the application Task function above.
 *
 *  Further down you can find the callback handler section containing the
 *  functions that defer their actions via messages to the application task.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void user_processGapStateChangeEvt(gaprole_States_t newState)
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        char *cstr_ownAddress = Util_convertBdAddr2Str(ownAddress);
        Log_info1("GAP is started. Our address: \x1b[32m%s\x1b[0m", (IArg)cstr_ownAddress);
      }
      break;

    case GAPROLE_ADVERTISING:
      Log_info0("Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &peerConnHandle);

        char *cstr_peerAddress = Util_convertBdAddr2Str(peerAddress);
        Log_info1("Connected. Peer address: \x1b[32m%s\x1b[0m", (IArg)cstr_peerAddress);

        // Change LED to indicated BLE connected
        led_event_t event = LED_BLE_CONNECTED;
        user_enqueueRawAppMsg(APP_MSG_LED_EVT, (uint8_t *)&event, sizeof(event));
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Log_info0("Connected and advertising");
      break;

    case GAPROLE_WAITING:
      {
        Log_info0("Disconnected / Idle");
        peerConnHandle = LINKDB_CONNHANDLE_INVALID;

        // Change LED to indicated BLE disconnected
        led_event_t event = LED_BLE_DISCONNECTED;
        user_enqueueRawAppMsg(APP_MSG_LED_EVT, (uint8_t *)&event, sizeof(event));
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      Log_info0("Connection timed out");
      break;

    case GAPROLE_ERROR:
      Log_info0("Error");
      break;

    default:
      break;
  }
}


/*
 * @brief   Handle a debounced button press or release in Task context.
 *          Invoked by the taskFxn based on a message received from a callback.
 *
 * @see     buttonDebounceSwiFxn
 * @see     buttonCallbackFxn
 *
 * @param   pState  pointer to button_state_t message sent from debounce Swi.
 *
 * @return  None.
 */
static void user_handleButtonPress(button_state_t *pState)
{
  Log_info2("%s %s",
    (IArg)(pState->pinId == Board_BUTTON0?"Button 0":"Button 1"),
    (IArg)(pState->state?"\x1b[32mpressed\x1b[0m":
                         "\x1b[33mreleased\x1b[0m"));

  // Update the service with the new value.
  // Will automatically send notification/indication if enabled.
  switch (pState->pinId)
  {
    case Board_BUTTON0:
    case Board_BUTTON1:
    {
        // Trigger emergency alert
        uint8_t alertType = 0;  // TODO: Add enum for alert types
        ProjectZero_triggerEmergencyAlert(alertType);
    }
    break;
  }
}

/*
 * @brief   Change LED states depending on different events.

 * @param   event - Event used to switch between different LED states.
 *
 * @return  None.
 */
static void user_handleLedEvt(led_event_t event)
{
    switch (event) {
      case LED_BLE_CONNECTED:
        Log_info0("LED Event: BLE Connected, turning on GREEN LED");
        PIN_setOutputValue(ledPinHandle, Board_GLED, 1);
        break;
      case LED_BLE_DISCONNECTED:
        Log_info0("LED Event: BLE Disconnected, turning off GREEN LED");
        PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
        break;
      case LED_HEARTRATE_UNDETECTED:
        Log_info0("LED Event: Heart Rate undetected, turning off RED LED");
        Util_stopClock(&ledBlinkClock);

        PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
        break;
      case LED_HEARTRATE_READING:
        Log_info0("LED Event: Heart Rate reading started, blinking RED LED");
        Util_startClock(&ledBlinkClock);
        break;
      case LED_HEARTRATE_COMPLETE:
        Log_info0("LED Event: Heart Rate reading complete, turning on RED LED");
        Util_stopClock(&ledBlinkClock);

        PIN_setOutputValue(ledPinHandle, Board_RLED, 1);
        break;
    }
}

/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
static void user_Emergency_alert_service_ValueChangeHandler(char_data_t *pCharData)
{
    switch (pCharData->paramID)
    {
      case EMERGENCY_ALERT_SERVICE_ALERTACTIVE_ID:
        Log_info3("Value Change msg: %s: %s, %d",
                  (IArg)"Emergency Alert Service",
                  (IArg)"ALERT_ACTIVE",
                  (IArg)pCharData->data);
        // -------------------------
        // Do something useful with pCharData->data here
        break;

    default:
      return;
    }
}

/*
 * @brief   Process an incoming BLE stack message.
 *
 *          This could be a GATT message from a peer device like acknowledgement
 *          of an Indication we sent, or it could be a response from the stack
 *          to an HCI message that the user application sent.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = ProjectZero_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            ProjectZero_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    Log_warning1("Outgoing RF FIFO full. Re-schedule transmission of msg with opcode 0x%02x",
      pMsg->method);

    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if(ProjectZero_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
    {
      // First free any pending response
      ProjectZero_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Log the opcode of the message that caused the violation.
    Log_error1("Flow control violated. Opcode of offending ATT msg: 0x%02x",
      pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Log_info1("MTU Size change: %d bytes", pMsg->msg.mtuEvt.MTU);
  }
  else
  {
    // Got an expected GATT message from a peer.
    Log_info1("Recevied GATT Message. Opcode: 0x%02x", pMsg->method);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      ProjectZero_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void ProjectZero_processConnEvt(Gap_ConnEventRpt_t *pReport)
{

  if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
  {
    // The GATT server might have returned a blePending as it was trying
    // to process an ATT Response. Now that we finished with this
    // connection event, let's try sending any remaining ATT Responses
    // on the next connection event.
    // Try to retransmit pending ATT Response (if any)
    ProjectZero_sendAttRsp();
  }

}

/*********************************************************************
 * @fn      ProjectZero_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 */
static void ProjectZero_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
    uint8_t status = pMsg->pReturnParam[0];

    //Find which command this command complete is for
    switch(pMsg->cmdOpcode)
    {
    case HCI_READ_RSSI:
    {
            int8 rssi = (int8)pMsg->pReturnParam[3];

        // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
        if(status == SUCCESS)
        {
            uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1],
                                           pMsg->pReturnParam[2]);

            Log_info2("RSSI:%d, connHandle %d",
                      (uint32_t)(rssi),
                      (uint32_t)handle);

            // Update RSSI value
            rssi *= -1;

            user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL,
                                    handle,
                                    CONFIG_SERVICE_SERV_UUID,
                                    CONFIG_SERVICE_RSSI_ID,
                                    &rssi,
                                    CONFIG_SERVICE_RSSI_LEN);

        } // end of if (status == SUCCESS)
        break;
    }

    case HCI_LE_READ_PHY:
    {
        if(status == SUCCESS)
        {
            Log_info2("RXPh: %d, TXPh: %d",
                      pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
        }
        break;
    }

    default:
        break;
    } // end of switch (pMsg->cmdOpcode)
}



/*
 *  Application error handling functions
 *****************************************************************************/

/*
 * @brief   Send a pending ATT response message.
 *
 *          The message is one that the stack was trying to send based on a
 *          peer request, but the response couldn't be sent because the
 *          user application had filled the TX queue with other data.
 *
 * @param   none
 *
 * @return  none
 */
static void ProjectZero_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      ProjectZero_UnRegistertToAllConnectionEvent (FOR_ATT_RSP);

      // We're done with the response message
      ProjectZero_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Log_warning2("Retrying message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
  }
}

/*
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void ProjectZero_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Log_info2("Sent message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
    else
    {
      Log_error2("Gave up message with opcode 0x%02x. Status: %d",
        pAttRsp->method, status);

      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/

/*********************************************************************
 * @fn      ProjectZero_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void ProjectZero_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  user_enqueueRawAppMsg(APP_MSG_PRZ_CONN_EVT, (uint8_t *)pReport, sizeof(pReport));
  ICall_free(pReport);
}

/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/

/**
 * Callback from GAP Role indicating a role state change.
 */
static void user_gapStateChangeCB(gaprole_States_t newState)
{
  Log_info1("(CB) GAP State change: %d, Sending msg to app.", (IArg)newState);
  user_enqueueRawAppMsg( APP_MSG_GAP_STATE_CHANGE, (uint8_t *)&newState, sizeof(newState) );
}

/*
 * @brief   Passcode callback.
 *
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode?
 * @param   uiOutputs  - display passcode?
 * @param   numComparison - numeric comparison value
 *
 * @return  none
 */
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison)
{
  passcode_req_t req =
  {
    .connHandle = connHandle,
    .uiInputs = uiInputs,
    .uiOutputs = uiOutputs,
    .numComparison = numComparison
  };

  // Defer handling of the passcode request to the application, in case
  // user input is required, and because a BLE API must be used from Task.
  user_enqueueRawAppMsg(APP_MSG_SEND_PASSCODE, (uint8_t *)&req, sizeof(req));
}

/*
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Log_info0("Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Log_info0("Pairing completed successfully.");
    }
    else
    {
      Log_error1("Pairing failed. Error: %02x", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
     Log_info0("Re-established pairing from stored bond info.");
    }
  }
}

/**
 * Callback handler for characteristic value changes in services.
 */
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                        uint8_t paramID, uint8_t *pValue,
                                        uint16_t len )
{
  // See the service header file to compare paramID with characteristic.
  Log_info2("(CB) Characteristic value change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  user_enqueueCharDataMsg(APP_MSG_SERVICE_WRITE, connHandle, svcUuid, paramID,
                          pValue, len);
}

/**
 * Callback handler for characteristic configuration changes in services.
 */
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                      uint8_t paramID, uint8_t *pValue,
                                      uint16_t len )
{
  Log_info2("(CB) Char config change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  user_enqueueCharDataMsg(APP_MSG_SERVICE_CFG, connHandle, svcUuid,
                          paramID, pValue, len);
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/

/*
 * @brief  Callback from Clock module on timeout
 *
 *         Determines new state after debouncing
 *
 * @param  buttonId    The pin being debounced
 */
static void buttonDebounceSwiFxn(UArg buttonId)
{
  // Used to send message to app
  button_state_t buttonMsg = { .pinId = buttonId };
  uint8_t        sendMsg   = FALSE;

  // Get current value of the button pin after the clock timeout
  uint8_t buttonPinVal = PIN_getInputValue(buttonId);

  // Set interrupt direction to opposite of debounced state
  // If button is now released (button is active low, so release is high)
  if (buttonPinVal)
  {
    // Enable negative edge interrupts to wait for press
    PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
  }
  else
  {
    // Enable positive edge interrupts to wait for relesae
    PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_POSEDGE);
  }

  switch(buttonId)
  {
    case Board_BUTTON0:
      // If button is now released (buttonPinVal is active low, so release is 1)
      // and button state was pressed (buttonstate is active high so press is 1)
      if (buttonPinVal && button0State)
      {
        // Button was released
        buttonMsg.state = button0State = 0;
        sendMsg = TRUE;
      }
      else if (!buttonPinVal && !button0State)
      {
        // Button was pressed
        buttonMsg.state = button0State = 1;
        sendMsg = TRUE;
      }
      break;

    case Board_BUTTON1:
      // If button is now released (buttonPinVal is active low, so release is 1)
      // and button state was pressed (buttonstate is active high so press is 1)
      if (buttonPinVal && button1State)
      {
        // Button was released
        buttonMsg.state = button1State = 0;
        sendMsg = TRUE;
      }
      else if (!buttonPinVal && !button1State)
      {
        // Button was pressed
        buttonMsg.state = button1State = 1;
        sendMsg = TRUE;
      }
      break;
  }

  if (sendMsg == TRUE)
  {
    user_enqueueRawAppMsg(APP_MSG_BUTTON_DEBOUNCED,
                      (uint8_t *)&buttonMsg, sizeof(buttonMsg));
  }
}

/*********************************************************************
 * @fn     rssiUpdateSwi
 *
 * @brief  Callback to update RSSI value of connected device.
 */
static void rssiUpdateSwi()
{
    user_enqueueRawAppMsg(APP_MSG_UPDATE_RSSI, NULL, 0);
}

/*********************************************************************
 * @fn     ledBlinkSwi
 *
 * @brief  Callback to toggle the LED to blink.
 */
static void ledBlinkSwi()
{
    user_enqueueRawAppMsg(APP_MSG_LED_BLINK, NULL, 0);
}

/*
 *  Callbacks from Hwi-context
 *****************************************************************************/

/*
 * @brief  Callback from PIN driver on interrupt
 *
 *         Sets in motion the debouncing.
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
  Log_info1("Button interrupt: %s",
            (IArg)((pinId == Board_BUTTON0)?"Button 0":"Button 1"));

  // Disable interrupt on that pin for now. Re-enabled after debounce.
  PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);

  // Start debounce timer
  switch (pinId)
  {
    case Board_BUTTON0:
      Clock_start(Clock_handle(&button0DebounceClock));
      break;
    case Board_BUTTON1:
      Clock_start(Clock_handle(&button1DebounceClock));
      break;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*
 * @brief  Generic message constructor for characteristic data.
 *
 *         Sends a message to the application for handling in Task context where
 *         the message payload is a char_data_t struct.
 *
 *         From service callbacks the appMsgType is APP_MSG_SERVICE_WRITE or
 *         APP_MSG_SERVICE_CFG, and functions running in another context than
 *         the Task itself, can set the type to APP_MSG_UPDATE_CHARVAL to
 *         make the user Task loop invoke user_updateCharVal function for them.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @param  connHandle    GAP Connection handle of the relevant connection
 * @param  serviceUUID   16-bit part of the relevant service UUID
 * @param  paramID       Index of the characteristic in the service
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueCharDataMsg( app_msg_types_t appMsgType,
                                     uint16_t connHandle,
                                     uint16_t serviceUUID, uint8_t paramID,
                                     uint8_t *pValue, uint16_t len )
{
  // Called in Stack's Task context, so can't do processing here.
  // Send message to application message queue about received data.
  uint16_t readLen = len; // How much data was written to the attribute

  // Allocate memory for the message.
  // Note: The pCharData message doesn't have to contain the data itself, as
  //       that's stored in a variable in the service implementation.
  //
  //       However, to prevent data loss if a new value is received before the
  //       service's container is read out via the GetParameter API is called,
  //       we copy the characteristic's data now.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + sizeof(char_data_t) +
                                  readLen );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    char_data_t *pCharData = (char_data_t *)pMsg->pdu;
    pCharData->svcUUID = serviceUUID; // Use 16-bit part of UUID.
    pCharData->paramID = paramID;
    // Copy data from service now.
    memcpy(pCharData->data, pValue, readLen);
    // Update pCharData with how much data we received.
    pCharData->dataLen = readLen;
    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
  // Let application know there's a message.
  Event_post(syncEvent, PRZ_APP_MSG_EVT);
  }
}

/*
 * @brief  Generic message constructor for application messages.
 *
 *         Sends a message to the application for handling in Task context.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
                                  uint16_t len)
{
  // Allocate memory for the message.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + len );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    // Copy data into message
    memcpy(pMsg->pdu, pData, len);

    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
//    // Let application know there's a message.
    Event_post(syncEvent, PRZ_APP_MSG_EVT);
  }
}


/*
 * @brief  Convenience function for updating characteristic data via char_data_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void user_updateCharVal(char_data_t *pCharData)
{
  switch(pCharData->svcUUID) {
    case CONFIG_SERVICE_SERV_UUID:
        Config_service_SetParameter(pCharData->paramID, pCharData->dataLen,
                                    pCharData->data);
        break;
    case HEARTRATE_SERVICE_SERV_UUID:
        Heartrate_service_SetParameter(pCharData->paramID, pCharData->dataLen,
                                       pCharData->data);
        break;
    case EMERGENCY_ALERT_SERVICE_SERV_UUID:
        Emergency_alert_service_SetParameter(pCharData->paramID, pCharData->dataLen,
                                             pCharData->data);
        break;
  }
}

/*
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len)
{
  char        hex[] = "0123456789ABCDEF";
  uint8_t     *pStr = dst;
  uint8_t     avail = dst_len-1;

  memset(dst, 0, avail);

  while (src_len && avail > 3)
  {
    if (avail < dst_len-1) { *pStr++ = ':'; avail -= 1; };
    *pStr++ = hex[*src >> 4];
    *pStr++ = hex[*src++ & 0x0F];
    avail -= 2;
    src_len--;
  }

  if (src_len && avail)
    *pStr++ = ':'; // Indicate not all data fit on line.

  return (char *)dst;
}

#if defined(UARTLOG_ENABLE)
/*
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char *Util_getLocalNameStr(const uint8_t *data) {
  uint8_t nuggetLen = 0;
  uint8_t nuggetType = 0;
  uint8_t advIdx = 0;

  static char localNameStr[32] = { 0 };
  memset(localNameStr, 0, sizeof(localNameStr));

  for (advIdx = 0; advIdx < 32;) {
    nuggetLen = data[advIdx++];
    nuggetType = data[advIdx];
    if ( (nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
          nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) && nuggetLen < 31) {
      memcpy(localNameStr, &data[advIdx + 1], nuggetLen - 1);
      break;
    } else {
      advIdx += nuggetLen;
    }
  }

  return localNameStr;
}
#endif

/*********************************************************************
*********************************************************************/
