/**************************************************************************************************
  Filename:       eegservice.c
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the Heart Rate sample service 
                  for use with the Heart Rate sample application.

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "eegservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of heart rate measurement value in attribute array
// ???Where/how is this used? 
#define ECG_MEAS_VALUE_POS            2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// ECG service
CONST uint8 ecgServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECG_SERV_UUID), HI_UINT16(ECG_SERV_UUID)
};

// ECG measurement characteristic
CONST uint8 ecgMeasUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECG_MEAS_UUID), HI_UINT16(ECG_MEAS_UUID)
};

// Number of Channels per Sample Set characteristic
CONST uint8 ecgNumChansUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECG_NUM_CHANS_UUID), HI_UINT16(ECG_NUM_CHANS_UUID)
};

// Samples Sets Per Packet Characteristic
CONST uint8 ecgSampleSetsUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECG_SAMPLE_SETS_UUID), HI_UINT16(ECG_SAMPLE_SETS_UUID)
};

// Command characteristic
CONST uint8 ecgCommandUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECG_COMMAND_UUID), HI_UINT16(ECG_COMMAND_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static ecgServiceCB_t ecgServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// Heart Rate Service attribute
static CONST gattAttrType_t ecgService = { ATT_BT_UUID_SIZE, ecgServUUID };

// Heart Rate Measurement Characteristic
// Note characteristic value is not stored here
static uint8 ecgMeasProps = GATT_PROP_NOTIFY;
//static uint8 ecgMeas[ECG_PACKET_SIZE_BYTES];   
static uint8 ecgMeas = 0; // vishy
static gattCharCfg_t ecgMeasClientCharCfg[GATT_MAX_NUM_CONN];
static uint8 ecgMeasUserDesp[21] = "ECG Measurement Data\0";

// ECG Num Chans Characteristic
static uint8 ecgNumChansProps = GATT_PROP_READ;
static uint8 ecgNumChans = ECG_NUM_CHANNELS;
static uint8 ecgNumChansUserDesp[23] = "Number of ECG Channels\0";

// Number of ECG Sample sets (containing 3 bytes of data for each ECG channel)
static uint8 ecgSampeSetsProps = GATT_PROP_READ;
static uint8 ecgSampleSetsPerPacket = ECG_SAMPLE_SETS_PER_PACKET;
static uint8 ecgSampleSetsUserDesp[27] = "ECG Sample Sets Per Packet\0";

// Command Characteristic
static uint8 ecgCommandProps = GATT_PROP_WRITE;
static uint8 ecgCommand = 0;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t ecgAttrTbl[] = 
{
  // Heart Rate Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&ecgService                      /* pValue */
  },

    // ECG Measurement Declaration - Characteristic 1
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ecgMeasProps 
    },

      // ECG Measurement Values for one sample - Characteristic 1
    { 
      { ATT_BT_UUID_SIZE, ecgMeasUUID },
      0, 
      0, 
      &ecgMeas 
    },

    // ECG Measurement Client Characteristic Configuration - Characteristic 1
    { 
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
      0, 
      (uint8 *) &ecgMeasClientCharCfg 
    },      


    // ECG Measure User Description - Characteristic 1
    { 
      { ATT_BT_UUID_SIZE, charUserDescUUID },
      GATT_PERMIT_READ, 
      0, 
      ecgMeasUserDesp 
    },           

    
    // ECG Num Channels Declaration - Characteristic 2
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ecgNumChansProps 
    },

    // ECG Num Channels Value - Characteristic 2
    { 
      { ATT_BT_UUID_SIZE, ecgNumChansUUID },
      GATT_PERMIT_READ, 
      0, 
      &ecgNumChans 
    },

    // ECG Num Channels User Description - Characteristic 2
    { 
      { ATT_BT_UUID_SIZE, charUserDescUUID },
      GATT_PERMIT_READ, 
      0, 
      ecgNumChansUserDesp 
    },  
    
    // ECG Sample Sets Declaration - Characteristic 3
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ecgSampeSetsProps 
    },

    // ECG Sample Sets Value - Characteristic 3
    { 
      { ATT_BT_UUID_SIZE, ecgSampleSetsUUID },
      GATT_PERMIT_READ, 
      0, 
      &ecgSampleSetsPerPacket 
    },

    // ECG Sample Sets User Description - Characteristic 3
    { 
      { ATT_BT_UUID_SIZE, charUserDescUUID },
      GATT_PERMIT_READ, 
      0, 
      ecgSampleSetsUserDesp 
    },           


    // Command Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ecgCommandProps 
    },

      // Command Value
      { 
        { ATT_BT_UUID_SIZE, ecgCommandUUID },
        GATT_PERMIT_WRITE, 
        0, 
        &ecgCommand 
      }
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 ecg_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t ecg_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Heart Rate Service Callbacks
CONST gattServiceCBs_t ecgCBs =
{
  ecg_ReadAttrCB,  // Read callback function pointer
  ecg_WriteAttrCB, // Write callback function pointer
  NULL                   // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HeartRate_AddService
 *
 * @brief   Initializes the Heart Rate service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t ecg_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, ecgMeasClientCharCfg );

  if ( services & ECG_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( ecgAttrTbl, 
                                          GATT_NUM_ATTRS( ecgAttrTbl ),
                                          &ecgCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      HeartRate_Register
 *
 * @brief   Register a callback function with the Heart Rate Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void ecg_Register( ecgServiceCB_t pfnServiceCB )
{
  ecgServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      HeartRate_SetParameter
 *
 * @brief   Set a Heart Rate parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t ecg_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
     case ECG_MEAS_CHAR_CFG:
      // Need connection handle
      //heartRateMeasClientCharCfg.value = *((uint16*)value);
      break;      

    case ECG_NUM_CHANS:
      ecgNumChans = *((uint8*)value);
      break;
    
    // vishy  
    case ECG_SAMPLE_SETS:
      ecgSampleSetsPerPacket = *((uint8*)value);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      HeartRate_GetParameter
 *
 * @brief   Get a Heart Rate parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t ecg_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case ECG_MEAS_CHAR_CFG:
      // Need connection handle
      //*((uint16*)value) = heartRateMeasClientCharCfg.value;
      break;      

    case ECG_NUM_CHANS:
      *((uint8*)value) = ecgNumChans;
      break;
    
    // vishy  
    case ECG_SAMPLE_SETS:
      *((uint8*)value) = ecgSampleSetsPerPacket;
      break;
    
      
    case ECG_COMMAND:
      *((uint8*)value) = ecgCommand;
      break;  

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          HeartRate_MeasNotify
 *
 * @brief       Send a notification containing a heart rate
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t ecg_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, ecgMeasClientCharCfg );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = ecgAttrTbl[ECG_MEAS_VALUE_POS].handle;
  
    // Send the notification
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleIncorrectMode;
}
                               
/*********************************************************************
 * @fn          heartRate_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 ecg_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  if (uuid == ECG_NUM_CHANS_UUID)
  {
    *pLen = 1;
    pValue[0] = *pAttr->pValue;
  }
  // vishy
  else if (uuid == ECG_SAMPLE_SETS_UUID) 
  {  
    *pLen = 1;
    pValue[0] = *pAttr->pValue;
    
  }
  else
  {
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return ( status );
}

/*********************************************************************
 * @fn      heartRate_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   complete - whether this is the last packet
 * @param   oper - whether to validate and/or write attribute value  
 *
 * @return  Success or Failure
 */
static bStatus_t ecg_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
 
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch ( uuid )
  {
    case ECG_COMMAND_UUID:
      if ( offset > 0 )
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      else if (len != 1)
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else if (*pValue != ECG_COMMAND_ENERGY_EXP)
      {
        status = ECG_ERR_NOT_SUP;
      }
      else
      {
        *(pAttr->pValue) = pValue[0];
        
        (*ecgServiceCB)(ECG_COMMAND_SET);
        
      }
      break;

    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY );
      if ( status == SUCCESS )
      {
        uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

        (*ecgServiceCB)( (charCfg == GATT_CFG_NO_OPERATION) ?
                                ECG_MEAS_NOTI_DISABLED :
                                ECG_MEAS_NOTI_ENABLED );
      }
      break;       
 
    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}

/*********************************************************************
 * @fn          HeartRate_HandleConnStatusCB
 *
 * @brief       Heart Rate Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
void ecg_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, ecgMeasClientCharCfg );
    }
  }
}


/*********************************************************************
*********************************************************************/
