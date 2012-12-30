/**************************************************************************************************
  Filename:       AXD.c
  Revised:        $Date: 2007-10-27 17:16:54 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15793 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 15 seconds.  The application will also
  receive "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "AXD.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/*用户自己的头文件*/
#include "adxl345.h"
#include "readaxd.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define ZB_USER_EVENTS                    0x00FF

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t AXD_ClusterList[AXD_MAX_CLUSTERS] =
{
  AXD_CLUSTERID
};

const SimpleDescriptionFormat_t AXD_SimpleDesc =
{
  AXD_ENDPOINT,              //  int Endpoint;
  AXD_PROFID,                //  uint16 AppProfId[2];
  AXD_DEVICEID,              //  uint16 AppDeviceId[2];
  AXD_DEVICE_VERSION,        //  int   AppDevVer:4;
  AXD_FLAGS,                 //  int   AppFlags:4;
  AXD_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)AXD_ClusterList,  //  byte *pAppInClusterList;
  AXD_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)AXD_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in AXD_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t AXD_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
//char TEMP[8] = {0,0,0,0,0,0,0,0};
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte AXD_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // AXD_Init() is called.
devStates_t AXD_NwkState;


byte AXD_TransID;  // This is the unique message ID (counter)

afAddrType_t AXD_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void AXD_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void AXD_HandleKeys( byte shift, byte keys );
void AXD_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void AXD_SendTheMessage( void );

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AXD_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void AXD_Init( byte task_id )
{
  AXD_TaskID = task_id;
  AXD_NwkState = DEV_INIT;
  AXD_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  AXD_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  AXD_DstAddr.endPoint = 0;
  AXD_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  AXD_epDesc.endPoint = AXD_ENDPOINT;
  AXD_epDesc.task_id = &AXD_TaskID;
  AXD_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&AXD_SimpleDesc;
  AXD_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &AXD_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( AXD_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "AXD", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( AXD_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( AXD_TaskID, Match_Desc_rsp );
  
  //下面是用户自定义的初始化

  Init_ADXL345();
  
}

/*********************************************************************
 * @fn      AXD_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 AXD_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
//  osal_event_hdr_t *pMsg;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( AXD_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          AXD_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          AXD_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
    //      MSGpkt = (afIncomingMSGPacket_t *) pMsg;
#ifdef AXD_COR
          AXD_ReceiveDataIndication( MSGpkt->srcAddr.addr.shortAddr, MSGpkt->clusterId,
                                    MSGpkt->cmd.DataLength, MSGpkt->cmd.Data);
          //AXD_MessageMSGCB( MSGpkt );
#endif
          break;

        case ZDO_STATE_CHANGE:
          AXD_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (AXD_NwkState == DEV_ZB_COORD)
              || (AXD_NwkState == DEV_ROUTER)
              || (AXD_NwkState == DEV_END_DEVICE) )
          {
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( AXD_TaskID,
                                AXD_SEND_MSG_EVT,
                                AXD_SEND_MSG_TIMEOUT );
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( AXD_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in AXD_Init()).
  if ( events & AXD_SEND_MSG_EVT )
  {
    // Send "the" message
  //  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
    zb_HandleOsalEvent(events);
    AXD_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( AXD_TaskID,
                        AXD_SEND_MSG_EVT,
                        (AXD_SEND_MSG_TIMEOUT/5) );

    // return unprocessed events
    return (events ^ AXD_SEND_MSG_EVT);
  }
    if ( events & ( ZB_USER_EVENTS ) )
  {
    // User events are passed to the application
    zb_HandleOsalEvent( events );

    // Do not return here, return 0 later
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      AXD_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void AXD_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            AXD_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            AXD_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            AXD_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      AXD_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void AXD_HandleKeys( byte shift, byte keys )
{
  zAddrType_t dstAddr;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            AXD_epDesc.endPoint,
                            AXD_PROFID,
                            AXD_MAX_CLUSTERS, (cId_t *)AXD_ClusterList,
                            AXD_MAX_CLUSTERS, (cId_t *)AXD_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        AXD_PROFID,
                        AXD_MAX_CLUSTERS, (cId_t *)AXD_ClusterList,
                        AXD_MAX_CLUSTERS, (cId_t *)AXD_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      AXD_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void AXD_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case AXD_CLUSTERID:
      // "the" message
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      AXD_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void AXD_SendTheMessage( void )
{
//  char theMessageData[] = "Hello World";
#ifdef AXD_END
//  int i;

//  for(i=0;i<8;i++)
//  {
//    TEMP[i]++;
//  }

  Multiple_Read_ADXL345();
 // displayXYZ(BUFFER);

  if ( AF_DataRequest( &AXD_DstAddr, &AXD_epDesc,
                       AXD_CLUSTERID,
                       (byte)(sizeof(BUFFER)),
                       (byte *)(BUFFER),
                       &AXD_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
#endif
}

/*********************************************************************
*********************************************************************/
void AXD_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
#if defined ( MT_SAPI_CB_FUNC )
  /* First check if MT has subscribed for this callback. If so , pass it as
  a event to MonitorTest and return control to calling function after that */
  if ( SAPICB_CHECK( SPI_CB_SAPI_RCV_DATA_IND ) )
  {
    zb_MTCallbackReceiveDataIndication( source, command, len, pData  );
  }
  else
#endif  //MT_SAPI_CB_FUNC
  {
    zb_ReceiveDataIndication( source, command, len, pData  );
  }
}

void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
//  uint8 buf[8];
  displayXYZ(pData);

    
}