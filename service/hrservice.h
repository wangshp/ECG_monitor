/**************************************************************************************************
  Filename:       hrservice.h
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the hr service definitions and
                  prototypes.

**************************************************************************************************/

#ifndef hrSERVICE_H
#define hrSERVICE_H

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
#define HR_MEAS                      0
#define HR_MEAS_CHAR_CFG             1
#define HR_NUM_CHANS                 2
#define HR_SAMPLE_SETS               3
#define HR_COMMAND                   4

// Heart Rate Service UUIDs
#define HR_SERV_UUID                 0x2E0D
#define HR_MEAS_UUID                 0x2E37
#define HR_NUM_CHANS_UUID            0x2E38
#define HR_SAMPLE_SETS_UUID          0x2E39
#define HR_COMMAND_UUID              0x2E3A

// Maximum length of heart rate measurement characteristic
#define HR_MEAS_MAX                  (ATT_MAX_MTU_SIZE -5)

#define HR_SAMPLE_SETS_PER_PACKET    2      // We are packing together 2 samples
#define HR_BYTES_PER_CHANNEL         3       // 24 bits per hr channel
#define HR_NUM_CHANNELS              3
#define HR_PACKET_SIZE_BYTES         (HR_SAMPLE_SETS_PER_PACKET *(HR_BYTES_PER_CHANNEL * HR_NUM_CHANNELS ))+2
    
// Value for command characteristic
#define HR_COMMAND_ENERGY_EXP        0x01

// ATT Error code
// Control point value not supported
#define HR_ERR_NOT_SUP               0x80

// Heart Rate Service bit fields
#define HR_SERVICE                   0x00000001

// Callback events
#define HR_MEAS_NOTI_ENABLED         1
#define HR_MEAS_NOTI_DISABLED        2
#define HR_COMMAND_SET               3

/*********************************************************************
 * TYPEDEFS
 */

// Heart Rate Service callback function
typedef void (*hrServiceCB_t)(uint8 event);

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

extern bStatus_t hr_AddService( uint32 services );

/*
 * HeartRate_Register - Register a callback function with the
 *          Heart Rate Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void hr_Register( hrServiceCB_t pfnServiceCB );

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
extern bStatus_t hr_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * HeartRate_GetParameter - Get a Heart Rate parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t hr_GetParameter( uint8 param, void *value );

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
extern bStatus_t hr_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );

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
extern void hr_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HRSERVICE_H */
