/**************************************************************************************************
  Filename:       zcl_samplethermostat.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $

  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2013 - 2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED â€œAS ISâ€� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application implements a ZigBee Thermostat, based on Z-Stack 3.0.

  This application is based on the common sample-application user interface. Please see the main
  comment in zcl_sampleapp_ui.c. The rest of this comment describes only the content specific for
  this sample applicetion.
  
  Application-specific UI peripherals being used:

  - LEDs:
    LED1 on indicates that the system is currently heating or cooling.
    LED1 off indicates that the system is currently off.

  Application-specific menu system:

    <REMOTE TEMP> View the temperature of the remote temperature sensor
      Buttons have no affect on this screen
      This screen shows the following information:
        Line2:
          Shows the temperature of the remote temperature sensor

    <SET HEAT TEMP> Changes the heating point temperature
      Up/Down changes the temperature at which heating will activate
      This screen shows the following information:
        Line2:
          Shows current heating point temperature

    <SET COOL TEMP> Changes the cooling point temperature
      Up/Down changes the temperature at which cooling will activate
      This screen shows the following information:
        Line2:
          Shows current cooling point temperature

*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "zcomdef.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_hvac.h"
#include "zcl_ms.h"
#include "zcl_samplethermostat.h"

#include "zcl_sampleapps_ui.h"
#include "bdb_interface.h"
#include "on_board.h"

#include "board_led.h"
#include "board_key.h"
#include "nvintf.h"
#include "zstackmsg.h"
#include "zcl_port.h"
#include "znwk_config.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include "zstackapi.h"
#include "zcl_sampleapps_ui.h"
#include "util_timer.h"
#include "string.h"
#include "util_timer.h"
#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
#include "gp_common.h"
#endif

/*********************************************************************
 * MACROS
 */

#define GUI_REMOTE_TEMP    1
#define GUI_SET_HEATING   2         
#define GUI_SET_COOLING   3

#define APP_TITLE "   Thermostat   "

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Semaphore used to post events to the application thread
static ICall_Semaphore sem;
static ICall_EntityID  zclSampleThermostat_Entity;
static endPointDesc_t  zclSampleThermostatEpDesc = {0};

#if ZG_BUILD_ENDDEVICE_TYPE
static Clock_Handle EndDeviceRejoinClkHandle;
static Clock_Struct EndDeviceRejoinClkStruct;
#endif

// Passed in function pointers to the NV driver
static NVINTF_nvFuncts_t *pfnZdlNV = NULL;

// Key press parameters
static uint8_t keys;

// Task pending events
static uint16_t events = 0;

#ifdef BDB_REPORTING
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 8
  uint8 reportableChange[] = {0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x2C01 is 300 in int16
#endif
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 4
  uint8 reportableChange[] = {0x2C, 0x01, 0x00, 0x00}; // 0x2C01 is 300 in int16
#endif 
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 2
  uint8 reportableChange[] = {0x2C, 0x01}; // 0x2C01 is 300 in int16
#endif 
#endif

  

afAddrType_t sensordstAddr;
uint8 data = 0x01;
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void zclSampleThermostat_initialization(void);
static void zclSampleThermostat_process_loop(void);
static void zclSampleThermostat_initParameters(void);
static void zclSampleThermostat_processZStackMsgs(zstackmsg_genericReq_t *pMsg);
static void SetupZStackCallbacks(void);
static void zclSampleThermostat_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg);
static void zclSampleThermostat_initializeClocks(void);
#if ZG_BUILD_ENDDEVICE_TYPE
static void zclSampleThermostat_processEndDeviceRejoinTimeoutCallback(UArg a0);
#endif
static void zclSampleThermostat_changeKeyCallback(uint8_t keysPressed);
static void zclSampleThermostat_processKey(uint8 keysPressed);
static void zclSampleThermostat_Init( void );


static void zclSampleThermostat_BasicResetCB( void );
#ifdef MT_APP_FUNC
static void zclSampleThermostat_ProcessAppMsg( uint8 srcEP, uint8 len, uint8 *msg );
static void zclSampleThermostat_ProcessFoundationMsg( afAddrType_t *dstAddr, uint16 clusterID,
                                                      zclFrameHdr_t *hdr, zclParseCmd_t *pParseCmd );
#endif

static void zclSampleThermostat_ProcessCommissioningStatus(bdbCommissioningModeMsg_t* bdbCommissioningModeMsg);


// app display functions
void zclSampleThermostat_LcdDisplayUpdate(void);
void zclSampleThermostat_LcdDisplayMainMode(void);
void zclSampleThermostat_LcdDisplayHeatMode(void);
void zclSampleThermostat_LcdDisplayCoolMode(void);
void zclSampleThermostat_LcdDisplayHelpMode(void);



// Functions to process ZCL Foundation incoming Command/Response messages
static uint8 zclSampleThermostat_ProcessIncomingMsg( zclIncoming_t *pInMsg);
#ifdef ZCL_READ
static uint8 zclSampleThermostat_ProcessInReadRspCmd( zclIncoming_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleThermostat_ProcessInWriteRspCmd( zclIncoming_t *pInMsg );
#endif
#ifdef ZCL_REPORT_DESTINATION_DEVICE
static void zclSampleThermostat_ProcessInReportCmd( zclIncoming_t *pInMsg );
#endif  // ZCL_REPORT_DESTINATION_DEVICE
static uint8 zclSampleThermostat_ProcessInDefaultRspCmd( zclIncoming_t *pInMsg );

static void zclSampleThermostat_UiActionSetHeating(uint16 keys);
static void zclSampleThermostat_UiActionSetCooling(uint16 keys);
void zclSampleThermostat_UiAppUpdateLcd(uint8 uiCurrentState, char * line[3]);
static void zclSampleThermostat_UpdateLedState(void);

/*********************************************************************
 * STATUS STRINGS
 */

/*********************************************************************
 * CONSTANTS
 */
const uiState_t zclSampleThermostat_UiAppStatesMain[] =
{
  /*  UI_STATE_BACK_FROM_APP_MENU  */   {UI_STATE_DEFAULT_MOVE, GUI_SET_COOLING, UI_KEY_SW_5_PRESSED, &UI_ActionBackFromAppMenu}, //do not change this line, except for the second item, which should point to the last entry in this menu
  /*  GUI_REMOTE_TEMP         */   {UI_STATE_DEFAULT_MOVE, UI_STATE_DEFAULT_MOVE, NULL},
  /*  GUI_SET_HEATING          */   {UI_STATE_DEFAULT_MOVE, UI_STATE_DEFAULT_MOVE, UI_KEY_SW_1_PRESSED | UI_KEY_SW_3_PRESSED, &zclSampleThermostat_UiActionSetHeating},
  /*  GUI_SET_COOLING         */   {UI_STATE_BACK_FROM_APP_MENU, UI_STATE_DEFAULT_MOVE, UI_KEY_SW_1_PRESSED | UI_KEY_SW_3_PRESSED, &zclSampleThermostat_UiActionSetCooling},
};

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleThermostat_CmdCallbacks =
{
  zclSampleThermostat_BasicResetCB,            // Basic Cluster Reset command
  NULL,                                        // Identify Trigger Effect command
  NULL,                                                // On/Off cluster command
  NULL,                                        // On/Off cluster enhanced command Off with Effect
  NULL,                                        // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                        // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                        // Level Control Move to Level command
  NULL,                                        // Level Control Move command
  NULL,                                        // Level Control Step command
  NULL,                                        // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                        // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                        // Scene Store Request command
  NULL,                                        // Scene Recall Request command
  NULL,                                        // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                        // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                        // Get Event Log command
  NULL,                                        // Publish Event Log command
#endif
  NULL,                                        // RSSI Location command
  NULL                                         // RSSI Location Response command
};



/*******************************************************************************
 * @fn          zclSampleThermostat_task
 *
 * @brief       Application task entry point for the ZStack Sample Thermostat
 *              Application
 *
 * @param       pfnNV - pointer to the NV functions
 *
 * @return      none
 */

void zclSampleThermostat_task(NVINTF_nvFuncts_t *pfnNV)
{
  // Save and register the function pointers to the NV drivers
  pfnZdlNV = pfnNV;
  zclport_registerNV(pfnZdlNV, ZCL_PORT_SCENE_TABLE_NV_ID);

  // Initialize application
  zclSampleThermostat_initialization();

  // No return from task process
  zclSampleThermostat_process_loop();
}



/*******************************************************************************
 * @fn          zclSampleThermostat_initialization
 *
 * @brief       Initialize the application
 *
 * @param       none
 *
 * @return      none
 */
static void zclSampleThermostat_initialization(void)
{

    /* Initialize user clocks */
    zclSampleThermostat_initializeClocks();

    /* Initialize keys */
    Board_Key_initialize(zclSampleThermostat_changeKeyCallback);

    /* Initialize the LEDS */
    Board_Led_initialize();

    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&zclSampleThermostat_Entity, &sem);


    //Initialize stack
    zclSampleThermostat_Init();


}



/*******************************************************************************
 * @fn      SetupZStackCallbacks
 *
 * @brief   Setup the Zstack Callbacks wanted
 *
 * @param   none
 *
 * @return  none
 */
static void SetupZStackCallbacks(void)
{
    zstack_devZDOCBReq_t zdoCBReq = {0};

    // Register for Callbacks, turn on:
    //  Device State Change,
    //  ZDO Match Descriptor Response,
    zdoCBReq.has_devStateChange = true;
    zdoCBReq.devStateChange = true;
    zdoCBReq.has_deviceAnnounce = true;;
    zdoCBReq.deviceAnnounce = true;
    (void)Zstackapi_DevZDOCBReq(zclSampleThermostat_Entity, &zdoCBReq);
}



/*********************************************************************
 * @fn          zclSampleThermostat_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
static void zclSampleThermostat_Init( void )
{

#ifdef BDB_REPORTING
  zstack_bdbRepAddAttrCfgRecordDefaultToListReq_t  Req = {0};
#endif
  // Register the Application to receive the unprocessed Foundation command/response messages
  zclport_registerZclHandleExternal(zclSampleThermostat_ProcessIncomingMsg);

  //Register Endpoint
  zclSampleThermostatEpDesc.endPoint = SAMPLETHERMOSTAT_ENDPOINT;
  zclSampleThermostatEpDesc.simpleDesc = &zclSampleThermostat_SimpleDesc;
  zclport_registerEndpoint(zclSampleThermostat_Entity, &zclSampleThermostatEpDesc);


  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLETHERMOSTAT_ENDPOINT, &zclSampleThermostat_CmdCallbacks );

  // Register the application's attribute list and reset to default values
  zclSampleThermostat_ResetAttributesToDefaultValues();
  zcl_registerAttrList( SAMPLETHERMOSTAT_ENDPOINT, zclSampleThermostat_NumAttributes, zclSampleThermostat_Attrs );

  //Write the bdb initialization parameters
  zclSampleThermostat_initParameters();

  //Setup ZDO callbacks
  SetupZStackCallbacks();

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
  gp_endpointInit(zclSampleThermostat_Entity);
#endif

#ifdef BDB_REPORTING
    //Adds the default configuration values for the reportable attributes of the ZCL_CLUSTER_ID_HVAC_THERMOSTAT cluster, for endpoint SAMPLETHERMOSTAT_ENDPOINT
    //Default maxReportingInterval value is 10 seconds
    //Default minReportingInterval value is 3 seconds
    //Default reportChange value is 300 (3 degrees)
    
    Req.attrID = ATTRID_HVAC_THERMOSTAT_LOCAL_TEMPERATURE;
    Req.cluster = ZCL_CLUSTER_ID_HVAC_THERMOSTAT;
    Req.endpoint = SAMPLETHERMOSTAT_ENDPOINT;
    Req.maxReportInt = 10;
    Req.minReportInt =  3;
    memcpy(&Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
    Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(zclSampleThermostat_Entity, &Req);
    
    Req.attrID = ATTRID_HVAC_THERMOSTAT_PI_COOLING_DEMAND;
    Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(zclSampleThermostat_Entity, &Req);
    
    Req.attrID = ATTRID_HVAC_THERMOSTAT_PI_HEATING_DEMAND;
    Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(zclSampleThermostat_Entity, &Req);
#endif

  UI_Init(zclSampleThermostat_Entity, &events, sem, SAMPLEAPP_LCD_AUTO_UPDATE_EVT, SAMPLEAPP_PROCESS_UI_UART_EVT, &zclSampleThermostat_IdentifyTime, APP_TITLE, &zclSampleThermostat_UiAppUpdateLcd, zclSampleThermostat_UiAppStatesMain);

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
  app_Green_Power_Init(zclSampleThermostat_Entity, &events, sem, SAMPLEAPP_PROCESS_GP_DATA_SEND_EVT,
                       SAMPLEAPP_PROCESS_GP_EXPIRE_DUPLICATE_EVT);
#endif

  UI_UpdateLcd();

}

static void zclSampleThermostat_initParameters(void)
{
    zstack_bdbSetAttributesReq_t zstack_bdbSetAttrReq;

    zstack_bdbSetAttrReq.bdbCommissioningGroupID              = BDB_DEFAULT_COMMISSIONING_GROUP_ID;
    zstack_bdbSetAttrReq.bdbPrimaryChannelSet                 = BDB_DEFAULT_PRIMARY_CHANNEL_SET;
    zstack_bdbSetAttrReq.bdbScanDuration                      = BDB_DEFAULT_SCAN_DURATION;
    zstack_bdbSetAttrReq.bdbSecondaryChannelSet               = BDB_DEFAULT_SECONDARY_CHANNEL_SET;
    zstack_bdbSetAttrReq.has_bdbCommissioningGroupID          = TRUE;
    zstack_bdbSetAttrReq.has_bdbPrimaryChannelSet             = TRUE;
    zstack_bdbSetAttrReq.has_bdbScanDuration                  = TRUE;
    zstack_bdbSetAttrReq.has_bdbSecondaryChannelSet           = TRUE;
#if (ZG_BUILD_COORDINATOR_TYPE)
    zstack_bdbSetAttrReq.has_bdbJoinUsesInstallCodeKey        = TRUE;
    zstack_bdbSetAttrReq.has_bdbTrustCenterNodeJoinTimeout    = TRUE;
    zstack_bdbSetAttrReq.has_bdbTrustCenterRequireKeyExchange = TRUE;
    zstack_bdbSetAttrReq.bdbJoinUsesInstallCodeKey            = BDB_DEFAULT_JOIN_USES_INSTALL_CODE_KEY;
    zstack_bdbSetAttrReq.bdbTrustCenterNodeJoinTimeout        = BDB_DEFAULT_TC_NODE_JOIN_TIMEOUT;
    zstack_bdbSetAttrReq.bdbTrustCenterRequireKeyExchange     = BDB_DEFAULT_TC_REQUIRE_KEY_EXCHANGE;
#endif
#if (ZG_BUILD_JOINING_TYPE)
    zstack_bdbSetAttrReq.has_bdbTCLinkKeyExchangeAttemptsMax  = TRUE;
    zstack_bdbSetAttrReq.has_bdbTCLinkKeyExchangeMethod       = TRUE;
    zstack_bdbSetAttrReq.bdbTCLinkKeyExchangeAttemptsMax      = BDB_DEFAULT_TC_LINK_KEY_EXCHANGE_ATTEMPS_MAX;
    zstack_bdbSetAttrReq.bdbTCLinkKeyExchangeMethod           = BDB_DEFAULT_TC_LINK_KEY_EXCHANGE_METHOD;
#endif

    Zstackapi_bdbSetAttributesReq(zclSampleThermostat_Entity, &zstack_bdbSetAttrReq);
}

/*******************************************************************************
 * @fn      zclSampleThermostat_initializeClocks
 *
 * @brief   Initialize Clocks
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleThermostat_initializeClocks(void)
{
#if ZG_BUILD_ENDDEVICE_TYPE
    // Initialize the timers needed for this application
    EndDeviceRejoinClkHandle = Timer_construct(
    &EndDeviceRejoinClkStruct,
    zclSampleThermostat_processEndDeviceRejoinTimeoutCallback,
    SAMPLEAPP_END_DEVICE_REJOIN_DELAY,
    0, false, 0);
#endif

}

#if ZG_BUILD_ENDDEVICE_TYPE
/*******************************************************************************
 * @fn      zclSampleThermostat_processEndDeviceRejoinTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void zclSampleThermostat_processEndDeviceRejoinTimeoutCallback(UArg a0)
{
    (void)a0; // Parameter is not used

    events |= SAMPLEAPP_END_DEVICE_REJOIN_EVT;

    // Wake up the application thread when it waits for clock event
    Semaphore_post(sem);
}
#endif


/*******************************************************************************
 * @fn      zclSampleThermostat_process_loop
 *
 * @brief   Application task processing start.
 *
 * @param   none
 *
 * @return  void
 */
static void zclSampleThermostat_process_loop(void)
{
    /* Forever loop */
    for(;;)
    {
        ICall_ServiceEnum stackid;
        ICall_EntityID dest;
        zstackmsg_genericReq_t *pMsg = NULL;

        /* Wait for response message */
        if(ICall_wait(ICALL_TIMEOUT_FOREVER) == ICALL_ERRNO_SUCCESS)
        {
            /* Retrieve the response message */
            if(ICall_fetchServiceMsg(&stackid, &dest, (void **)&pMsg)
               == ICALL_ERRNO_SUCCESS)
            {
                if( (stackid == ICALL_SERVICE_CLASS_ZSTACK)
                    && (dest == zclSampleThermostat_Entity) )
                {
                    if(pMsg)
                    {
                        zclSampleThermostat_processZStackMsgs(pMsg);

                        // Free any separately allocated memory
                        Zstackapi_freeIndMsg(pMsg);
                    }
                }

                if(pMsg)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            if(events & SAMPLEAPP_KEY_EVT)
            {
                // Process Key Presses
                zclSampleThermostat_processKey(keys);
                keys = 0;
                events &= ~SAMPLEAPP_KEY_EVT;
            }

            if(events & SAMPLEAPP_LCD_AUTO_UPDATE_EVT)
            {
                UI_UpdateLcd();
                events &= ~SAMPLEAPP_LCD_AUTO_UPDATE_EVT;
            }

            if(events & SAMPLEAPP_PROCESS_UI_UART_EVT)
            {
                zclSampleAppsUI_ProcessUART();
                events &= ~SAMPLEAPP_PROCESS_UI_UART_EVT;
            }

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
            if(events & SAMPLEAPP_PROCESS_GP_DATA_SEND_EVT)
            {
                zcl_gpSendNotification();
                events &= ~SAMPLEAPP_PROCESS_GP_DATA_SEND_EVT;
            }

            if(events & SAMPLEAPP_PROCESS_GP_EXPIRE_DUPLICATE_EVT)
            {
                gp_expireDuplicateFiltering();
                events &= ~SAMPLEAPP_PROCESS_GP_EXPIRE_DUPLICATE_EVT;
            }


#endif

#if ZG_BUILD_ENDDEVICE_TYPE
            if ( events & SAMPLEAPP_END_DEVICE_REJOIN_EVT )
            {
              zstack_bdbZedAttemptRecoverNwkRsp_t zstack_bdbZedAttemptRecoverNwkRsp;

              Zstackapi_bdbZedAttemptRecoverNwkReq(zclSampleThermostat_Entity,&zstack_bdbZedAttemptRecoverNwkRsp);

              events &= ~SAMPLEAPP_END_DEVICE_REJOIN_EVT;
            }
#endif


        }
    }
}




/*******************************************************************************
 * @fn      zclSampleThermostat_processZStackMsgs
 *
 * @brief   Process event from Stack
 *
 * @param   pMsg - pointer to incoming ZStack message to process
 *
 * @return  void
 */
static void zclSampleThermostat_processZStackMsgs(zstackmsg_genericReq_t *pMsg)
{
    switch(pMsg->hdr.event)
    {
        case zstackmsg_CmdIDs_BDB_NOTIFICATION:
            {
                zstackmsg_bdbNotificationInd_t *pInd;
                pInd = (zstackmsg_bdbNotificationInd_t*)pMsg;
                zclSampleThermostat_ProcessCommissioningStatus(&(pInd->Req));
            }
            break;
            
        case zstackmsg_CmdIDs_BDB_IDENTIFY_TIME_CB:
            {
                zstackmsg_bdbIdentifyTimeoutInd_t *pInd;
                pInd = (zstackmsg_bdbIdentifyTimeoutInd_t*) pMsg;
                uiProcessIdentifyTimeChange(&(pInd->EndPoint));
            }
            break;
            
        case zstackmsg_CmdIDs_BDB_BIND_NOTIFICATION_CB:
            {
                zstackmsg_bdbBindNotificationInd_t *pInd;
                pInd = (zstackmsg_bdbBindNotificationInd_t*) pMsg;
                uiProcessBindNotification(&(pInd->Req));
            }
            break;

        case zstackmsg_CmdIDs_AF_INCOMING_MSG_IND:
            {
                // Process incoming data messages
                zstackmsg_afIncomingMsgInd_t *pInd;
                pInd = (zstackmsg_afIncomingMsgInd_t *)pMsg;
                zclSampleThermostat_processAfIncomingMsgInd( &(pInd->req) );
            }
            break;

        case zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND:
            {
                zstackmsg_devPermitJoinInd_t *pInd;
                pInd = (zstackmsg_devPermitJoinInd_t*)pMsg;
                uiProcessPermitJoin(&(pInd->Req));
            }
            break;

            
#if (ZG_BUILD_JOINING_TYPE)
        case zstackmsg_CmdIDs_BDB_CBKE_TC_LINK_KEY_EXCHANGE_IND:
        {
          zstack_bdbCBKETCLinkKeyExchangeAttemptReq_t zstack_bdbCBKETCLinkKeyExchangeAttemptReq;
          /* Z3.0 has not defined CBKE yet, so lets attempt default TC Link Key exchange procedure
           * by reporting CBKE failure.
           */

          zstack_bdbCBKETCLinkKeyExchangeAttemptReq.didSuccess = FALSE;

          Zstackapi_bdbCBKETCLinkKeyExchangeAttemptReq(zclSampleThermostat_Entity, 
                                                       &zstack_bdbCBKETCLinkKeyExchangeAttemptReq);
        }
        break;

        case zstackmsg_CmdIDs_BDB_FILTER_NWK_DESCRIPTOR_IND:

         /*   User logic to remove networks that do not want to join
          *   Networks to be removed can be released with Zstackapi_bdbNwkDescFreeReq
          */
       
          Zstackapi_bdbFilterNwkDescComplete(zclSampleThermostat_Entity);
        break;
            
#endif
        case zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND:
        {
            // The ZStack Thread is indicating a State change
            zstackmsg_devStateChangeInd_t *pInd =
                (zstackmsg_devStateChangeInd_t *)pMsg;
                if(pInd->req.state==zstack_DevState_DEV_END_DEVICE)
                {
                  zAddrType_t dstAddr;
                    dstAddr.addrMode = Addr16Bit;
                    dstAddr.addr.shortAddr = 0x0000; // Coordinator
                    ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                        SAMPLETHERMOSTAT_ENDPOINT,
                        zclSampleThermostat_SimpleDesc.AppProfId,
                        zclSampleThermostat_SimpleDesc.AppNumInClusters, (cId_t *)zclSampleThermostat_SimpleDesc.pAppInClusterList,
                        zclSampleThermostat_SimpleDesc.AppNumOutClusters, (cId_t *)zclSampleThermostat_SimpleDesc.pAppOutClusterList,
                        FALSE );



                }

        }
        break;



        /*
         * These are messages/indications from ZStack that this
         * application doesn't process.  These message can be
         * processed by your application, remove from this list and
         * process them here in this switch statement.
         */

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
          case zstackmsg_CmdIDs_GP_DATA_IND:
          {
              zstackmsg_gpDataInd_t *pInd;
              pInd = (zstackmsg_gpDataInd_t*)pMsg;
              gp_processDataIndMsg( &(pInd->Req) );
          }
          break;

          case zstackmsg_CmdIDs_GP_SECURITY_REQ:
          {
              zstackmsg_gpSecReq_t *pInd;
              pInd = (zstackmsg_gpSecReq_t*)pMsg;
              gp_processSecRecMsg( &(pInd->Req) );
          }
          break;




          case zstackmsg_CmdIDs_GP_CHECK_ANNCE:
          {
              zstackmsg_gpCheckAnnounce_t *pInd;
              pInd = (zstackmsg_gpCheckAnnounce_t*)pMsg;
              gp_processCheckAnnceMsg( &(pInd->Req) );
          }
#endif
         
#ifdef BDB_TL_TARGET
        case zstackmsg_CmdIDs_BDB_TOUCHLINK_TARGET_ENABLE_IND:
        {
          zstackmsg_bdbTouchLinkTargetEnableInd_t *pInd = 
            (zstackmsg_bdbTouchLinkTargetEnableInd_t*)pMsg;
        
          uiProcessTouchlinkTargetEnable(pInd->Enable);
        }
        break;
#endif
        case zstackmsg_CmdIDs_BDB_TC_LINK_KEY_EXCHANGE_NOTIFICATION_IND:
        case zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND:

        case zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP:
        case zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP:
        case zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP:
        case zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_USER_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_USER_DESC_SET_RSP:
        case zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP:
        case zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP:
        case zstackmsg_CmdIDs_ZDO_BIND_RSP:
        case zstackmsg_CmdIDs_ZDO_UNBIND_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY:
        case zstackmsg_CmdIDs_ZDO_SRC_RTG_IND:
        case zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND:
        case zstackmsg_CmdIDs_ZDO_LEAVE_CNF:
        case zstackmsg_CmdIDs_ZDO_LEAVE_IND:
        case zstackmsg_CmdIDs_SYS_RESET_IND:
        case zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND:
        case zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND:
            break;
        case zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE:
        {


               zstackmsg_zdoDeviceAnnounceInd_t *pInd;
               pInd = (zstackmsg_zdoDeviceAnnounceInd_t*)pMsg;
               //sensordstAddr.addr.shortAddr=pInd->req.devAddr;
               //sensordstAddr.endPoint=SAMPLETHERMOSTAT_ENDPOINT;
               //sensordstAddr.addrMode=(afAddrMode_t)Addr16Bit;


        }
            break;
        default:
            break;
    }
}



/*******************************************************************************
 *
 * @fn          zclSampleThermostat_processAfIncomingMsgInd
 *
 * @brief       Process AF Incoming Message Indication message
 *
 * @param       pInMsg - pointer to incoming message
 *
 * @return      none
 *
 */
static void zclSampleThermostat_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg)
{
    afIncomingMSGPacket_t afMsg;

    /*
     * All incoming messages are passed to the ZCL message processor,
     * first convert to a structure that ZCL can process.
     */
    afMsg.groupId = pInMsg->groupID;
    afMsg.clusterId = pInMsg->clusterId;
    afMsg.srcAddr.endPoint = pInMsg->srcAddr.endpoint;
    afMsg.srcAddr.panId = pInMsg->srcAddr.panID;
    afMsg.srcAddr.addrMode = (afAddrMode_t)pInMsg->srcAddr.addrMode;
    if( (afMsg.srcAddr.addrMode == afAddr16Bit)
        || (afMsg.srcAddr.addrMode == afAddrGroup)
        || (afMsg.srcAddr.addrMode == afAddrBroadcast) )
    {
        afMsg.srcAddr.addr.shortAddr = pInMsg->srcAddr.addr.shortAddr;
    }
    else if(afMsg.srcAddr.addrMode == afAddr64Bit)
    {
        memcpy(afMsg.srcAddr.addr.extAddr, &(pInMsg->srcAddr.addr.extAddr), 8);
    }
    afMsg.macDestAddr = pInMsg->macDestAddr;
    afMsg.endPoint = pInMsg->endpoint;
    afMsg.wasBroadcast = pInMsg->wasBroadcast;
    afMsg.LinkQuality = pInMsg->linkQuality;
    afMsg.correlation = pInMsg->correlation;
    afMsg.rssi = pInMsg->rssi;
    afMsg.SecurityUse = pInMsg->securityUse;
    afMsg.timestamp = pInMsg->timestamp;
    afMsg.nwkSeqNum = pInMsg->nwkSeqNum;
    afMsg.macSrcAddr = pInMsg->macSrcAddr;
    afMsg.radius = pInMsg->radius;
    afMsg.cmd.TransSeqNumber = pInMsg->transSeqNum;
    afMsg.cmd.DataLength = pInMsg->n_payload;
    afMsg.cmd.Data = pInMsg->pPayload;

    zcl_ProcessMessageMSG(&afMsg);
}




/*********************************************************************
 * @fn      zclSampleThermostat_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleThermostat_BasicResetCB( void )
{
  zclSampleThermostat_ResetAttributesToDefaultValues();
  
  UI_UpdateLcd();  
  zclSampleThermostat_UpdateLedState();
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleThermostat_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  uint8 - TRUE if got handled
 */
static uint8 zclSampleThermostat_ProcessIncomingMsg( zclIncoming_t *pInMsg)
{
  uint8 handled = FALSE;
  switch ( pInMsg->hdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleThermostat_ProcessInReadRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleThermostat_ProcessInWriteRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
#ifdef ZCL_REPORT
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleThermostat_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleThermostat_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleThermostat_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleThermostat_ProcessInReadReportCfgRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT_DESTINATION_DEVICE
    case ZCL_CMD_REPORT:
      zclSampleThermostat_ProcessInReportCmd( pInMsg );
      handled = TRUE;
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleThermostat_ProcessInDefaultRspCmd( pInMsg );
      handled = TRUE;      
      break;

    default:
      break;
  }

  return handled;
}


#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleThermostat_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleThermostat_ProcessInReadRspCmd( zclIncoming_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;


  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  if(readRspCmd->attrList[0].attrID==ATTRID_TEST_SENSOR)
  {
      Board_Led_toggle(board_led_type_LED1);
  }
  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleThermostat_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleThermostat_ProcessInWriteRspCmd( zclIncoming_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

#ifdef ZCL_REPORT_DESTINATION_DEVICE
/*********************************************************************
 * @fn      zclSampleThermostat_ProcessInReportCmd
 *
 * @brief   Process the "Profile" Report Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static void zclSampleThermostat_ProcessInReportCmd( zclIncoming_t *pInMsg )
{
  zclReportCmd_t *pInTempSensorReport;

  pInTempSensorReport = (zclReportCmd_t *)pInMsg->attrCmd;

  if (pInTempSensorReport->attrList[0].attrID == ATTRID_TEST_SENSOR)
  {



  }
  



  // store the current temperature value sent over the air from temperature sensor
 // zclSampleThermostat_LocalTemperature = BUILD_UINT16(pInTempSensorReport->attrList[0].attrData[0], pInTempSensorReport->attrList[0].attrData[1]);
  
  //UI_UpdateLcd();
 // zclSampleThermostat_UpdateLedState();
  return;
}
#endif  // ZCL_REPORT_DESTINATION_DEVICE

/*********************************************************************
 * @fn      zclSampleThermostat_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleThermostat_ProcessInDefaultRspCmd( zclIncoming_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}




/*********************************************************************
 * @fn      zclSampleThermostat_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
static void zclSampleThermostat_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
{
  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
  {
    case BDB_COMMISSIONING_FORMATION:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        zstack_bdbStartCommissioningReq_t zstack_bdbStartCommissioningReq;
        
        //After formation, perform nwk steering again plus the remaining commissioning modes that has not been process yet        
        zstack_bdbStartCommissioningReq.commissioning_mode = BDB_COMMISSIONING_MODE_NWK_STEERING | bdbCommissioningModeMsg->bdbRemainingCommissioningModes;
        Zstackapi_bdbStartCommissioningReq(zclSampleThermostat_Entity,&zstack_bdbStartCommissioningReq);
      }
      else
      {
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_NWK_STEERING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
        //We are on the nwk, what now?
      }
      else
      {
        //See the possible errors for nwk steering procedure
        //No suitable networks found
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_FINDING_BINDING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
      }
      else
      {
        //YOUR JOB:
        //retry?, wait for user interaction?
      }
    break;
    case BDB_COMMISSIONING_INITIALIZATION:
      //Initialization notification can only be successful. Failure on initialization 
      //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification
      
      //YOUR JOB:
      //We are on a network, what now?
      
    break;
#if ZG_BUILD_ENDDEVICE_TYPE    
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        Timer_setTimeout( EndDeviceRejoinClkHandle, SAMPLEAPP_END_DEVICE_REJOIN_DELAY );
        Timer_start(&EndDeviceRejoinClkStruct);
      }
    break;
#endif 
  }
  
  UI_UpdateComissioningStatus(bdbCommissioningModeMsg);
}




static void zclSampleThermostat_UiActionSetHeating(uint16 keys)
{
  if ( keys & UI_KEY_SW_1_PRESSED )
  {
    // increase heating setpoint, considering whole numbers where necessary
    if ( zclSampleThermostat_OccupiedHeatingSetpoint < zclSampleThermostat_MaxHeatSetpointLimit )
    {
      zclSampleThermostat_OccupiedHeatingSetpoint = zclSampleThermostat_OccupiedHeatingSetpoint + 100;
    }
    else if ( zclSampleThermostat_OccupiedHeatingSetpoint >= zclSampleThermostat_MaxHeatSetpointLimit )
    {
      zclSampleThermostat_OccupiedHeatingSetpoint = zclSampleThermostat_MaxHeatSetpointLimit;
    }
  }
  
  if ( keys & UI_KEY_SW_3_PRESSED )
  {
    // decrease heating setpoint, considering whole numbers where necessary
    if ( zclSampleThermostat_OccupiedHeatingSetpoint > zclSampleThermostat_MinHeatSetpointLimit )
    {
      zclSampleThermostat_OccupiedHeatingSetpoint = zclSampleThermostat_OccupiedHeatingSetpoint - 100;
    }
    else if ( zclSampleThermostat_OccupiedHeatingSetpoint <= zclSampleThermostat_MinHeatSetpointLimit )
    {
      zclSampleThermostat_OccupiedHeatingSetpoint = zclSampleThermostat_MinHeatSetpointLimit;
    }
  }
  
  UI_UpdateLcd();
  zclSampleThermostat_UpdateLedState();
}

static void zclSampleThermostat_UiActionSetCooling(uint16 keys)
{
  if ( keys & UI_KEY_SW_1_PRESSED )
  {
    // increase cooling setpoint, considering whole numbers where necessary
    if ( zclSampleThermostat_OccupiedCoolingSetpoint < zclSampleThermostat_MaxCoolSetpointLimit )
    {
      zclSampleThermostat_OccupiedCoolingSetpoint = zclSampleThermostat_OccupiedCoolingSetpoint + 100;
    }
    else if ( zclSampleThermostat_OccupiedCoolingSetpoint >= zclSampleThermostat_MaxCoolSetpointLimit )
    {
      zclSampleThermostat_OccupiedCoolingSetpoint = zclSampleThermostat_MaxCoolSetpointLimit;
    }
  }
  
  if ( keys & UI_KEY_SW_3_PRESSED )
  {
    // decrease cooling setpoint, considering whole numbers where necessary
    if ( zclSampleThermostat_OccupiedCoolingSetpoint > zclSampleThermostat_MinCoolSetpointLimit )
    {
      zclSampleThermostat_OccupiedCoolingSetpoint = zclSampleThermostat_OccupiedCoolingSetpoint - 100;
    }
    else if ( zclSampleThermostat_OccupiedCoolingSetpoint <= zclSampleThermostat_MinCoolSetpointLimit )
    {
      zclSampleThermostat_OccupiedCoolingSetpoint = zclSampleThermostat_MinCoolSetpointLimit;
    }
  }
  
  UI_UpdateLcd();
  zclSampleThermostat_UpdateLedState();
}

void zclSampleThermostat_UpdateLedState(void)
{
  // use LEDs to show heating or cooling cycles based off local temperature
  if ( zclSampleThermostat_LocalTemperature != NULL )
  {
    if ( zclSampleThermostat_LocalTemperature <= zclSampleThermostat_OccupiedHeatingSetpoint )
    {
      // turn on heating
      zclSampleThermostat_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_HEAT;
      Board_Led_control(board_led_type_LED1, board_led_state_ON);
      Board_Led_control(board_led_type_LED2, board_led_state_OFF);
    }
    else if ( zclSampleThermostat_LocalTemperature >= zclSampleThermostat_OccupiedCoolingSetpoint )
    {
      // turn on cooling
      zclSampleThermostat_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_COOL;
      Board_Led_control(board_led_type_LED2, board_led_state_ON);
      Board_Led_control(board_led_type_LED1, board_led_state_OFF);
    }
    else
    {
      // turn off heating/cooling
      zclSampleThermostat_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_OFF;
      Board_Led_control(board_led_type_LED1, board_led_state_OFF);
      Board_Led_control(board_led_type_LED2, board_led_state_OFF);
    }
  }
}

void zclSampleThermostat_UiAppUpdateLcd(uint8 gui_state, char * line[3])
{ 
  static char sDisplayTemp[16];
  
  //Clear the string
  memset(sDisplayTemp,0,16);

  switch(gui_state)
  {
    case GUI_REMOTE_TEMP:
      memcpy( sDisplayTemp, "TEMP: ", 6 );
      
      // if local temperature has not been set, make note on display
      if ( zclSampleThermostat_LocalTemperature == NULL )
      {
        memcpy( &sDisplayTemp[6], "N/A", 4 );
      }
      else
      {
        _ltoa( ( zclSampleThermostat_LocalTemperature / 100 ), (void *)(&sDisplayTemp[6]), 10 ); // only use whole number
        memcpy( &sDisplayTemp[8], "C", 2 );
      }
      line[1] = (char *)sDisplayTemp;
      line[2] = "< REMOTE TEMP  >";
      break;
      
    case GUI_SET_HEATING:
      memcpy( sDisplayTemp, "HEAT TEMP: ", 11 );
      _ltoa( ( zclSampleThermostat_OccupiedHeatingSetpoint / 100 ), (void *)(&sDisplayTemp[11]), 10 ); // only use whole number
      memcpy( &sDisplayTemp[13], "C", 2 );

      line[1] = (char *)sDisplayTemp;
      line[2] = "<SET HEAT TEMP >";
      break;
      
    case GUI_SET_COOLING:
      memcpy(sDisplayTemp, "COOL TEMP: ", 11);
      _ltoa( ( zclSampleThermostat_OccupiedCoolingSetpoint / 100 ), (void *)(&sDisplayTemp[11]), 10 ); // only use whole number
      memcpy( &sDisplayTemp[13], "C", 2 );
      
      line[1] = (char *)sDisplayTemp;
      line[2] = "<SET COOL TEMP >";
      break;
      
    default:
      break;
  }
}

/****************************************************************************
****************************************************************************/

/*********************************************************************
 * @fn      zclSampleThermostat_changeKeyCallback
 *
 * @brief   Key event handler function
 *
 * @param   keysPressed - ignored
 *
 * @return  none
 */
static void zclSampleThermostat_changeKeyCallback(uint8_t keysPressed)
{
    keys = keysPressed;

    events |= SAMPLEAPP_KEY_EVT;

    // Wake up the application thread when it waits for clock event
    Semaphore_post(sem);
}


static void zclSampleThermostat_processKey(uint8 keysPressed)
{
    //Button 1
    if(keysPressed == KEY_LEFT)
    {
        zstack_bdbStartCommissioningReq_t zstack_bdbStartCommissioningReq;

        if(ZG_BUILD_COORDINATOR_TYPE && ZG_DEVICE_COORDINATOR_TYPE)
        {
            zstack_bdbStartCommissioningReq.commissioning_mode = BDB_COMMISSIONING_MODE_NWK_FORMATION | BDB_COMMISSIONING_MODE_NWK_STEERING;
            Zstackapi_bdbStartCommissioningReq(zclSampleThermostat_Entity,&zstack_bdbStartCommissioningReq);
        }
        else if (ZG_BUILD_JOINING_TYPE && ZG_DEVICE_JOINING_TYPE)
        {
            zstack_bdbStartCommissioningReq.commissioning_mode = BDB_COMMISSIONING_MODE_NWK_STEERING;
            Zstackapi_bdbStartCommissioningReq(zclSampleThermostat_Entity,&zstack_bdbStartCommissioningReq);
        }
    }
    //Button 2
    if(keysPressed == KEY_RIGHT)
    {


        sensordstAddr.addr.shortAddr=0;
        sensordstAddr.endPoint=0;
        sensordstAddr.addrMode= (afAddrMode_t)AddrNotPresent;
        zclWriteCmd_t *Sensorcmd;
        Sensorcmd = osal_mem_alloc( sizeof(zclWriteCmd_t) + sizeof(zclWriteRec_t) );
        if ( Sensorcmd != NULL )
        {
        Sensorcmd->numAttr=1;

        Sensorcmd->attrList[0].dataType=ZCL_DATATYPE_UINT8;
        Sensorcmd->attrList[0].attrID=ATTRID_TEST_SENSOR;
        Sensorcmd->attrList[0].attrData=&data;


        zcl_SendWrite(  SAMPLETHERMOSTAT_ENDPOINT,  &sensordstAddr,
                        ZCL_CLUSTER_ID_TEST, Sensorcmd,
                        ZCL_FRAME_CLIENT_SERVER_DIR,1,
        bdb_getZCLFrameCounter() );
        }
        osal_mem_free( Sensorcmd );
        data+=0x01;
       /* zclReadCmd_t *Sensocmd;
        Sensocmd = osal_mem_alloc( sizeof(zclReadCmd_t));

        if ( Sensocmd != NULL )
        {
            Sensocmd->numAttr=1;
            Sensocmd->attrID[0]=ATTRID_TEST_SENSOR;

            zcl_SendRead(SAMPLETHERMOSTAT_ENDPOINT,&sensordstAddr,ZCL_CLUSTER_ID_TEST,Sensocmd,
                         ZCL_FRAME_CLIENT_SERVER_DIR,1,
                         bdb_getZCLFrameCounter() );
        }
        osal_mem_free( Sensorcmd );*/
    }

}



