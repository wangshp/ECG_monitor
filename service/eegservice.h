/**************************************************************************************************
  Filename:       eegservice.h
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the EEG service definitions and
                  prototypes.

**************************************************************************************************/

#ifndef EEGSERVICE_H
#define EEGSERVICE_H

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


// Heart Rate Service Parameters
#define ECG_MEAS                      0
#define ECG_MEAS_CHAR_CFG             1
#define ECG_NUM_CHANS                 2
#define ECG_SAMPLE_SETS               3
#define ECG_COMMAND                   4

// Heart Rate Service UUIDs
#define ECG_SERV_UUID                 0x2D0D
#define ECG_MEAS_UUID                 0x2D37
#define ECG_NUM_CHANS_UUID            0x2D38
#define ECG_SAMPLE_SETS_UUID          0x2D39
#define ECG_COMMAND_UUID              0x2D3A

// Maximum length of heart rate measurement characteristic
#define ECG_MEAS_MAX                  (ATT_MAX_MTU_SIZE -5)

#define ECG_SAMPLE_SETS_PER_PACKET    2      // We are packing together 2 samples
#define ECG_BYTES_PER_CHANNEL         3       // 24 bits per ecg channel
#define ECG_NUM_CHANNELS              3
#define ECG_PACKET_SIZE_BYTES         (ECG_SAMPLE_SETS_PER_PACKET *(ECG_BYTES_PER_CHANNEL * ECG_NUM_CHANNELS ))+2
    
// Value for command characteristic
#define ECG_COMMAND_ENERGY_EXP        0x01

// ATT Error code
// Control point value not supported
#define ECG_ERR_NOT_SUP               0x80

// Heart Rate Service bit fields
#define ECG_SERVICE                   0x00000001

// Callback events
#define ECG_MEAS_NOTI_ENABLED         1
#define ECG_MEAS_NOTI_DISABLED        2
#define ECG_COMMAND_SET               3

/*********************************************************************
 * TYPEDEFS
 */

// Heart Rate Service callback function
typedef void (*ecgServiceCB_t)(uint8 event);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * HeartRate_AddService- Initializes the Heart Rate service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t ecg_AddService( uint32 services );

/*
 * HeartRate_Register - Register a callback function with the
 *          Heart Rate Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void ecg_Register( ecgServiceCB_t pfnServiceCB );

/*
 * HeartRate_SetParameter - Set a Heart Rate parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t ecg_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * HeartRate_GetParameter - Get a Heart Rate parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t ecg_GetParameter( uint8 param, void *value );

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
extern bStatus_t ecg_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );

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
extern void ecg_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* EEGSERVICE_H */
