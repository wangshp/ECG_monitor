/*************************************************************************************************
  Filename:       heartrate.c
  Revised:        $Date $
  Revision:       $Revision $
  Description:    This file contains the heart rate sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.
Copyright 2011 Texas Instruments Incorporated. All rights reserved.
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
  PROVIDED ???AS IS??ид WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "eegservice.h"

#include "devinfoservice.h"
#include "battservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "heartrate.h"

#include "TI_ADS1293.h"
#include "TI_CC254x.h"
#include "TI_CC254x_hardware_board.h"
#include "TI_CC254x_spi.h"
#include "TI_ADS1293_register_settings.h"

#include "ads1299.h"
#include "hal_i2c.h"
#include "HR_module.h"

//haptic part
#include "Haptics_2605.h"
#include "DRV2605.h"

//service
#include "hrservice.h"

/*********************************************************************
 * MACROS
 */

// Convert BPM to RR-Interval for data simulation purposes
#define HEARTRATE_BPM_TO_RR(bpm)              ((uint16) 60 * 1024 / (uint16) (bpm))

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             30000

// vishy 1600 to 800
// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             800

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0

// vishy: ios minimum interval is 20ms: change 7 to 16
// How often to perform heart rate periodic event (vishy: change 7 to 20)
#define DEFAULT_ECG_PERIOD                    20

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// vishy: ios minimum interval is 20ms: change 8 to 16 if needed
// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     16

// vishy: ios minimum interval is 20ms: change 8 to 16
// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     16

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6 

// Battery measurement period in ms
#define DEFAULT_BATT_PERIOD                   15000

// Some values used to simulate measurements
#define BPM_DEFAULT                           73
#define BPM_MAX                               80
#define ENERGY_INCREMENT                      10
#define FLAGS_IDX_MAX                         7

#define DEFAULT_HR_PERIOD                    20

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
//#define CH_DATA_SIZE 10     // should be 9+1 for 3 channels
#define CH_DATA_SIZE 9     // 9 bytes when no status is used
//#define PKT_DATA_SIZE 18
//#define NUM_PKTS 6
#define NUM_PKTS 18           // 8 when NUM_BUF = 16, 9 when 18, 12 when 24
//#define NUM_BUF  10       // use this when status byte is read
//#define NUM_BUF  12         // use this when no status byte is read
#define NUM_BUF 36          //  22 data + 2 error for supporting 320samples/sec
//#define PAD_SIZE 8          // 108 bytes sent out as 18 * 6
#define PAD_SIZE 0          // use this when no status byte is read
//#define BUF_SIZE ((CH_DATA_SIZE * NUM_BUF) + PAD_SIZE)  // totally 108 bytes sent out as NUM_PKTS * PKT_DATA_SIZE

//1299 BUFFER
#define CH_DATASIZE 3
#define NUM_CH 8
#define STATUSSIZE 3
#define BUF_SIZE 36   //12*3= 2*6*3 6*2ms *2 =24ms
#define PKT_DATA_SIZE 27

uint8 dataBufX[BUF_SIZE];
uint8 dataBufY[BUF_SIZE];
//uint8 dataBufZ[BUF_SIZE];
uint8 rptr = 0;
uint8 wptr = 0;
uint8 *recv_buf = dataBufX;
uint8 *send_buf = NULL;
uint8 dataReadyFlag = 0;

int counter_BLE = 0;


  //heart rate module
uint8 buffer_w[2] = {0, 0};  //data to write
uint8 buffer_w_2[1];

uint8 checkpoint1 = 0;       //data that read back

uint8 abc1; //test point

uint8 dataBuf_hr[4];
uint8 *recv_buf_hr = dataBuf_hr;
uint8 dataReadyFlag_hr = 0;

//HAPTIC PART
uint8 haptic_start = 1;
uint32 haptic_duration = 10; //10s
uint8 vibrate_period = 2; //200ms
extern uint8 vibration_amplitudeH; //0x10~0x30

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 ecg_TaskID;   // Task ID for internal task/event processing
static uint8 hr_TaskID;

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
  0x1a,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'A',
  'D',
  'S',
  '1',
  '2',
  '9',
  '3',
  ' ',   
  'E',
  'C',
  'G',
  ' ',
  'W',
  'i',
  'r',
  'e',
  'l',
  'e',
  's',
  's',
  ' ',
  'D',
  'e',
  'm',
  'o'
};

static uint8 advertData[] = 
{ 
  // flags
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // service UUIDs
  0x05,
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(ECG_SERV_UUID),
  HI_UINT16(ECG_SERV_UUID),
// vishy  LO_UINT16(BATT_SERVICE_UUID),
//  HI_UINT16(BATT_SERVICE_UUID)
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "ADS1293 ECG Wireless Demo";

// GAP connection handle
static uint16 gapConnHandle;

// Heart rate measurement value stored in this structure
static attHandleValueNoti_t ecgMeas;  
uint8 errCnt=0;

static attHandleValueNoti_t hrMeas;  

// Components of heart rate measurement structure
//static uint8 heartRateBpm = BPM_DEFAULT;
//static uint16 ecgEnergy = 0;
//static uint16 heartRateRrInterval1 = HEARTRATE_BPM_TO_RR(BPM_DEFAULT);
//static uint16 heartRateRrInterval2 = HEARTRATE_BPM_TO_RR(BPM_DEFAULT);


//static uint8 ecgFlagsIdx = 0;

// Advertising user-cancelled state
static bool ecgAdvCancelled = FALSE;

static bool ecg_running = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void ecg_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void ecgGapStateCB( gaprole_States_t newState );
static void ecgPeriodicTask( void );
static void ecgBattPeriodicTask( void );
static void ecg_HandleKeys( uint8 shift, uint8 keys );
static void ecgMeasNotify(void);
static void ecgCB(uint8 event);
static void ecgBattCB(uint8 event);

static void hrPeriodicTask( void );
static void hrMeasNotify(void);
static void hrCB(uint8 event);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t ecgPeripheralCB =
{
  ecgGapStateCB,                  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller
};

// Bond Manager Callbacks
static const gapBondCBs_t ecgBondCB =
{
  NULL,                   // Passcode callback
  NULL                    // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HeartRate_Init
 *
 * @brief   Initialization function for the Heart Rate App Task.
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

void ecg_Init( uint8 task_id )
{
  ecg_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
      
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = FALSE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }  

  // Setup the Heart Rate Characteristic Values
  {
    uint8 ecgNumChans = 3;
    ecg_SetParameter( ECG_NUM_CHANS, sizeof ( uint8 ), &ecgNumChans );
    
    uint8 ecgSampleSetsPerPacket = ECG_SAMPLE_SETS_PER_PACKET;
    ecg_SetParameter( ECG_SAMPLE_SETS, sizeof ( uint8 ), &ecgSampleSetsPerPacket );    
  }
  
  // Setup Battery Characteristic Values
  {
    uint8 critical = DEFAULT_BATT_CRITICAL_LEVEL;
    Batt_SetParameter( BATT_PARAM_CRITICAL_LEVEL, sizeof (uint8 ), &critical );
  }
  
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  ecg_AddService( GATT_ALL_SERVICES );
//heartrate
  hr_AddService( GATT_ALL_SERVICES );
  
  DevInfo_AddService( );
  // vishy Batt_AddService( );
  
  // Register for Heart Rate service callback
  ecg_Register( ecgCB );
//heartrate  
  hr_Register( hrCB );
  
  // Register for Battery service callback;
  Batt_Register ( ecgBattCB );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( ecg_TaskID );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.
  
  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFF; // It was input mode on p0.0 p0.1(For button)
                // all others (P0.0-P0.7) as output  ??CHANGED!!P0.0
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output
  
//  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P0 = 0;
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low 
  
  /* Enable ADC channel for battery measurement */
//  APCFG = 0x80; // AIN7 as analog input
//  P0DIR &= ~0x80; // force P0.7 to be input
  
  //vishy
  P1DIR &= ~0x02; // force P1.1 as input: with Vlithium
       
  /* Configure DRDYB (P1_7) for ADS1299*/
  P1DIR &= ~0x80;     //P1_7 as input                                                         // pin1.7 is input
  PICTL |= 0x04;     // falling edge interrupt
  IRCON2 &= ~0x08;   // clear Port 1 interrupt flag
  P1IFG &= ~0x80;    // clear Port1.7 pin status flag
  
  P1IEN |= 0x80;    // enable P1_7 interrupt
 
  IEN2  |= 0x10;       // TEST ???    mute interrupt      // enable Port1 interrupt
  
  
  
//  EA = 1;           // enable global interrupt  
  delay_init();
  
  //make sure every digital input is low.
  //Allow time for the supply voltages to reach their final value!!!
  ms_delay(200);
  TI_ADS1293_SPISetup();   // Initilaize CC254x SPI Block 

  ads1299_set_up();
  
  HalI2CInit( i2cClock_267KHZ );
  
  Haptics_Init();
  
  //heart rate init.
  buffer_w[0] = INT_STATUS;      //clear power up interrupt
  HalI2CWrite(HR_ID, 1, buffer_w, 0);
  HalI2CRead(HR_ID, 1, &checkpoint1);

  
  buffer_w[0] = LED_PWC;  //0X09
  buffer_w[1] = 0xAA;         //set led current
  HalI2CWrite(HR_ID, 2, buffer_w, 1);

  buffer_w[0] = SPO2_CONFIG; //0X07
  buffer_w[1] = 0x07;         //set 50SPS, LED pulse width 1600us
  HalI2CWrite(HR_ID, 2, buffer_w, 1);
  
  buffer_w[0] = MODE_CFR; //0X06
  buffer_w[1] = 0x02;         //set HR only mode 
  HalI2CWrite(HR_ID, 2, buffer_w, 1); //after set mode, it triggers power ready interrupt.

  buffer_w[0] = MODE_CFR;      
  HalI2CWrite(HR_ID, 1, buffer_w, 0);
  HalI2CRead(HR_ID, 1, &checkpoint1);
  
  buffer_w[0] = INT_EN;  //0X01
  buffer_w[1] = 0xA0;         // enable only HR interrupt 
  HalI2CWrite(HR_ID, 2, buffer_w, 1);

//  buffer_w[0] = MODE_CFR;
//  buffer_w[1] = 0x40;         //reset all registers of HR module, //reset works
//  HalI2CWrite(HR_ID, 2, buffer_w, 1);

  //test point: after set mode=HR, HR_INT is always HIGH.
  //!wait--but this interrupt is not generated by p2.0
  //!!what?--->generated by I2C 
  

  // Setup a delayed profile startup
  osal_set_event( ecg_TaskID, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      HeartRate_ProcessEvent
 *
 * @brief   Heart Rate Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */


uint16 ecg_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  static uint8 one_time_flag = 0;
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( ecg_TaskID )) != NULL )
    {
      ecg_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &ecgPeripheralCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &ecgBondCB );
    
    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & ECG_PERIODIC_EVT )
  {
    HalI2CInit( i2cClock_267KHZ ); //???didn't find out the reason where I2C got disabled???

 /*
    if (!one_time_flag)
    { uint8 i;
      for(i=0; i<PKT_DATA_SIZE;i++)
      {
        // initialize error buffers to 0xFF
        dataBufX[((NUM_BUF-2)*CH_DATA_SIZE)+i] = 0xff;
        dataBufY[((NUM_BUF-2)*CH_DATA_SIZE)+i] = 0xff;
      }
      one_time_flag = 1;
    }
*/
    
    // vishy Restart timer
    if (DEFAULT_ECG_PERIOD) //start periodic heart rate task
    {
      //ok, it is actually related with nitification enable or not!
      osal_start_timerEx( ecg_TaskID, ECG_PERIODIC_EVT, DEFAULT_ECG_PERIOD );
      
      haptic_config_update();
                      //right now every timer + is DEFAULT_ECG_PERIOD = 20ms.
                      //so we could check it every 20ms. but we need to enable ECG once to start timer.
                      //we could improve this part later...
      haptic_process(); //not the problem of time! then what?
      
    }
    // Perform periodic heart rate task
//dataReadyFlag = 1;

    
    
    if (dataReadyFlag)
    {
      ecgPeriodicTask();
    }

    return (events ^ ECG_PERIODIC_EVT);
  }  

  if ( events & BATT_PERIODIC_EVT )
  {
    // Perform periodic battery task
    ecgBattPeriodicTask();
    
    return (events ^ BATT_PERIODIC_EVT);
  }
  
  if ( events & HR_PERIODIC_EVT )
  {
    if (DEFAULT_ECG_PERIOD) //start periodic heart rate task
    {
      //ok, it is actually related with notification enable or not!
      osal_start_timerEx( hr_TaskID, HR_PERIODIC_EVT, DEFAULT_HR_PERIOD );
    }

    // Perform periodic battery task
    hrPeriodicTask();
    
    return (events ^ HR_PERIODIC_EVT);
  }
/*
  if ( events & ADS1293_DRDYB_EVT )  //deal with receive data in process but not interrupt, but need connect it with DRDY interrupt~
  {
    // Read ECG Data
    read_buf = &dataBuf[rptr * CH_DATA_SIZE];
    TI_ADS1293_SPIStreamReadReg(read_buf, CH_DATA_SIZE);                       // read adc output into read_buf
    rptr++;
    if (rptr == NUM_BUF)
    {
      rptr = 0;
      osal_set_event(ecg_TaskID, ECG_PERIODIC_EVT);
    } else if (rptr == NUM_BUF/2)
    {
      osal_set_event(ecg_TaskID, ECG_PERIODIC_EVT);     
    }
  }
*/  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      heartRate_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void ecg_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      ecg_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  }
}

/*********************************************************************
 * @fn      heartRate_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void ecg_HandleKeys( uint8 shift, uint8 keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
/*    
    // Left Key Button - Start/Stop Data Notification (also can be started from server)
    if (ecg_running)
    {
      ecg_running = FALSE;
      osal_stop_timerEx( ecg_TaskID, ECG_PERIODIC_EVT);
      TI_ADS1293_SPIWriteReg(TI_ADS1293_CONFIG_REG, 
                          TI_ADS1293_CONFIG_REG_VALUE & ADS1293_STOP_CONV);    // stop conversion      
    }
    else if (gapProfileState == GAPROLE_CONNECTED)
    {
      osal_start_timerEx( ecg_TaskID, ECG_PERIODIC_EVT, DEFAULT_ECG_PERIOD );
      TI_ADS1293_SPIWriteReg(TI_ADS1293_CONFIG_REG, 
                          TI_ADS1293_CONFIG_REG_VALUE | ADS1293_START_CONV);   // start conversion
      ecg_running = TRUE;
    } 
*/
  }
  
  if ( keys & HAL_KEY_SW_2 )
  {
/*
    // if not in a connection, toggle advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 status;
      
      // Set fast advertising interval for user-initiated connections
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );
      // toggle GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &status );
      status = !status;
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &status );   
      
      // Set state variable
      if (status == FALSE)
      {
        ecgAdvCancelled = TRUE;
      }
    }
*/
  }
}

/*********************************************************************
 * @fn      ecgMeasNotify
 *
 * @brief   Prepare and send a ecg measurement notification
 *
 * @return  none
 */

static void ecgMeasNotify(void)
{
  uint8 i;

  ecgMeas.len = 18;
 
    //----read data byte from spi
    
    //1 status(3 BYTES) + 8 channel * 3 BYTES(24bits)=27 B

    if(dataReadyFlag == 1)
    {
      
      osal_memcpy(&ecgMeas.value[0], &send_buf[counter_BLE * 18], 18);   //maximum size?
       
      if(ecg_MeasNotify( gapConnHandle, &ecgMeas) == SUCCESS)
      {
        counter_BLE++;
        if(counter_BLE == 2)    //2*18=36Bytes  12 samples has been sent, then wait for new data filled
        {
          counter_BLE = 0;
          dataReadyFlag = 0;  
          send_buf = NULL;
        }
      }
    } 
}

/*********************************************************************
 * @fn      hrMeasNotify
 *
 * @brief   Prepare and send a heart rate measurement notification
 *
 * @return  none
 */

static void hrMeasNotify(void)
{
  //every sample has 4 Bytes, including 2 B for IR and 2 B for RED.
   
  uint8 i;

  hrMeas.len = 4;
  
  uint8 *tmp_buf;
  tmp_buf = &recv_buf_hr[0];
  for(int i = 0; i < 4; i++)
  {
    buffer_w[0] = FIFO_DATA;      
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &tmp_buf[i]);                                 // Read data     
  }
 
    //----read data byte from spi
    
    //1 status(3 BYTES) + 8 channel * 3 BYTES(24bits)=27 B
      
      osal_memcpy(&hrMeas.value[0], &tmp_buf[0], 4);   //maximum size?
       
      if(ecg_MeasNotify( gapConnHandle, &hrMeas) == SUCCESS)
      {
      
      }
   
  /**/
}

/*********************************************************************
 * @fn      HeartRateGapStateCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void ecgGapStateCB( gaprole_States_t newState )
{
  // if connected
  if (newState == GAPROLE_CONNECTED)
  {
    // get connection handle
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
  }
  // if disconnected
  else if (gapProfileState == GAPROLE_CONNECTED && 
           newState != GAPROLE_CONNECTED)
  {
    uint8 advState = TRUE;

    // stop periodic measurement
    osal_stop_timerEx( ecg_TaskID, ECG_PERIODIC_EVT );
    TI_ADS1293_SPIWriteReg(TI_ADS1293_CONFIG_REG, 
                          TI_ADS1293_CONFIG_REG_VALUE & ADS1293_STOP_CONV);    // stop conversion     
    ecg_running = FALSE;

    // reset client characteristic configuration descriptors
    ecg_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );
    Batt_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );

    if ( newState == GAPROLE_WAITING_AFTER_TIMEOUT )
    {
      // link loss timeout-- use fast advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );
    }
    else
    {
      // Else use slow advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
    }

    // Enable advertising
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );    
  }    
  // if advertising stopped
  else if ( gapProfileState == GAPROLE_ADVERTISING && 
            newState == GAPROLE_WAITING )
  {
    // if advertising stopped by user
    if ( ecgAdvCancelled )
    {
      ecgAdvCancelled = FALSE;
    }
    // if fast advertising switch to slow
    else if ( GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL )
    {
      uint8 advState = TRUE;
      
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );   
    }  
  }
  // if started
  else if (newState == GAPROLE_STARTED)
  {
    // Set the system ID from the bd addr
    uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);
    
    // shift three bytes up
    systemId[7] = systemId[5];
    systemId[6] = systemId[4];
    systemId[5] = systemId[3];
    
    // set middle bytes to zero
    systemId[4] = 0;
    systemId[3] = 0;
    
    DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
  }
  
  gapProfileState = newState;
}

/*********************************************************************
 * @fn      ecgCB
 *
 * @brief   Callback function for eeg service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void ecgCB(uint8 event)
{
  if (event == ECG_MEAS_NOTI_ENABLED)
  {
    // if connected start periodic measurement
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      osal_start_timerEx( ecg_TaskID, ECG_PERIODIC_EVT, DEFAULT_ECG_PERIOD );
      TI_ADS1293_SPIWriteReg(TI_ADS1293_CONFIG_REG, 
                          TI_ADS1293_CONFIG_REG_VALUE | ADS1293_START_CONV);   // start conversion
      ecg_running = TRUE;
    } 
  }
  else if (event == ECG_MEAS_NOTI_DISABLED)
  {
    // stop periodic measurement
    osal_stop_timerEx( ecg_TaskID, ECG_PERIODIC_EVT );
    TI_ADS1293_SPIWriteReg(TI_ADS1293_CONFIG_REG, 
                          TI_ADS1293_CONFIG_REG_VALUE & ADS1293_STOP_CONV);    // stop conversion    
    ecg_running = FALSE;
  }
  else if (event == ECG_COMMAND_SET)
  {
    // reset energy expended
//    ecgEnergy = 0;
  }
}


/*********************************************************************
 * @fn      heartRateCB
 *
 * @brief   Callback function for heart rate service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void hrCB(uint8 event)
{
  if (event == HR_MEAS_NOTI_ENABLED)
  {
    // if connected start periodic measurement
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      osal_start_timerEx( hr_TaskID, HR_PERIODIC_EVT, DEFAULT_HR_PERIOD );
     // TI_ADS1293_SPIWriteReg(TI_ADS1293_CONFIG_REG, 
     //                     TI_ADS1293_CONFIG_REG_VALUE | ADS1293_START_CONV);   // start conversion
      //ecg_running = TRUE;
    } 
  }
  else if (event == HR_MEAS_NOTI_DISABLED)
  {
    // stop periodic measurement
    osal_stop_timerEx( hr_TaskID, HR_PERIODIC_EVT );
   // TI_ADS1293_SPIWriteReg(TI_ADS1293_CONFIG_REG, 
   //                       TI_ADS1293_CONFIG_REG_VALUE & ADS1293_STOP_CONV);    // stop conversion    
    //ecg_running = FALSE;
  }
  else if (event == HR_COMMAND_SET)
  {
    // reset energy expended
//    ecgEnergy = 0;
  }
}

/*********************************************************************
 * @fn      heartRateBattCB
 *
 * @brief   Callback function for battery service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void ecgBattCB(uint8 event)
{
  if (event == BATT_LEVEL_NOTI_ENABLED)
  {
    // if connected start periodic measurement
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      osal_start_timerEx( ecg_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
    } 
  }
  else if (event == BATT_LEVEL_NOTI_DISABLED)
  {
    // stop periodic measurement
    osal_stop_timerEx( ecg_TaskID, BATT_PERIODIC_EVT );
  }
}

/*********************************************************************
 * @fn      ecgPeriodicTask
 *
 * @brief   Perform a periodic heart rate application task.
 *
 * @param   none
 *
 * @return  none
 */
static void ecgPeriodicTask( void )
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    // send heart rate measurement notification
    ecgMeasNotify();
    // Restart timer (vishy already done)
    //osal_start_timerEx( ecg_TaskID, ECG_PERIODIC_EVT, DEFAULT_ECG_PERIOD );
  }
}

/*********************************************************************
 * @fn      heartRateBattPeriodicTask
 *
 * @brief   Perform a periodic task for battery measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void ecgBattPeriodicTask( void )
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    // perform battery level check
    Batt_MeasLevel( );
    
    // Restart timer
    osal_start_timerEx( ecg_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
  }
}

/*********************************************************************
 * @fn      heartRatePeriodicTask
 *
 * @brief   Perform a periodic heart rate application task.
 *
 * @param   none
 *
 * @return  none
 */
static void hrPeriodicTask( void )
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    hrMeasNotify();
  }
}

int counter_ADS = 0;

/*********************************************************************
*********************************************************************/
//******************************************************************************
// TI_ADS1293_SPI_DRDYB_PIN interrupt service routine
#pragma vector = P1INT_VECTOR
__near_func __interrupt void TI_ADS1293_DRDY_PORTx(void)
{
//  static uint8 rd_count = 0;
//  static uint8 first_time = 1;
  uint8 *tmp_buf;
  uint8 blank_buf;
  EA = 0;
  IRCON2 &= ~0x08;    // clear Port 1 interrupt flag

  

  
  if (P1IFG & 0x80)
  {
    P1IFG &= ~0x80;               // clear interrupt status flag

    // Read ECG Data
  
    //spiWriteByte(_RDATA);
     
    tmp_buf = &recv_buf[0];
    
    //TI_ADS1293_SPIStreamRead((tmp_buf), BUF_SIZE);
    
    /* test RDATA mode */
    CS_ADS = 0; 
    spiWriteByte(_RDATA);    
    us_delay(10);
    
    //read and dismiss the 3 status bytes of ADS.
    for(int i = 0; i<8; i++)
    {
      spiReadByte((&blank_buf), 0xFF);
      spiReadByte((&blank_buf), 0xFF);
      spiReadByte((&blank_buf), 0xFF);
    }
    for(int i = 0; i < 3; i++)
    {
      spiReadByte((tmp_buf+i+counter_ADS), 0xFF);                                             // Read data     
    }
    counter_ADS = counter_ADS + 3;
    CS_ADS = 1;
    
    //no flag contrain, no ADS waiting, keep data continuous.
    if(counter_ADS > 35)  //36bytes will takes 12*2ms = 24ms or 12*4ms= 48ms
    {      
      send_buf = recv_buf;
      if (recv_buf == dataBufX)  //keep sending to another buffer.
        recv_buf = dataBufY;
      else
        recv_buf = dataBufX;
      dataReadyFlag = 1; 
      counter_ADS = 0;

    } 
  }

  EA = 1;
}


//interrupt should not process data!!!!!!!!!!!
//how much time it take to read data? interrupt shouldn't keep too long time.
//interrupt most of time used to set flag and process data in main function.

//observe: even after i comment the ISR, if i enable PORT2's interrupt, it will keep calling ISR.
//try: maybe mask SI after using it in I2C transfer will be useful.

//interrupt period: 16*20=320ms   
//16*4=64bytes--0.03ms*8*64=
/*for HR interrupt p2_0*/
#pragma vector = P2INT_VECTOR
__interrupt void TI_ADS1299_HR_INT(void)
{
//  static uint8 rd_count = 0;
//  static uint8 first_time = 1;
  uint8 *tmp_buf;
  
  EA = 0;
  IRCON2 &= ~0x01;    // clear Port 2 interrupt flag
  
 // I2CCFG = I2CCFG & (~0x08);
    
  //check point: breakpoint set here, log a lot of time, show I2C keep go inside the ISR
  //-->it won't affect HR_INT going low or high, but it will take time to affect BLE?
  if (P2IFG & 0x01)
  {
    if(dataReadyFlag == 0)
    {
      P1IFG &= ~0x01;            // clear p2.0 interrupt status flag
      
      tmp_buf = &recv_buf[0];
      
      //buffer_w_2[0] = INT_EN;
      buffer_w_2[0] = FIFO_DATA;
      HalI2CWrite(HR_ID, 1, buffer_w_2, 0);
      
      //HalI2CRead(HR_ID, 1, tmp_buf);
      
      HalI2CRead(HR_ID, 36, tmp_buf);
      send_buf = recv_buf;
      if (recv_buf == dataBufX)  //keep sending to another buffer.
        recv_buf = dataBufY;
      else
        recv_buf = dataBufX;
      dataReadyFlag = 1; 
      counter_ADS = 0;
    }
  }
  EA = 1;
}

uint8 haptic_config = 0;
void haptic_config_update(void)
{
  ecg_GetParameter(ECG_COMMAND, &haptic_config);
  if(haptic_config == 0x01)
    haptic_start = 1;              
  else if(haptic_config == 0x02)
  {
    haptic_start = 0;
    haptic_stop();
  }
  else if(haptic_config == 0x03)
    haptic_duration++;             //*m
  else if(haptic_config == 0x04)
    haptic_duration--;
  else if(haptic_config == 0x05)
    vibrate_period++;              //*100ms
  else if(haptic_config == 0x06)
    vibrate_period--;
  else if(haptic_config == 0x07)
    vibration_amplitudeH = vibration_amplitudeH + 0x05;
  else if(haptic_config == 0x08)
    vibration_amplitudeH = vibration_amplitudeH - 0x05;
  
  if(vibration_amplitudeH > 0x31)
    vibration_amplitudeH = 0x30;
  if(vibration_amplitudeH < 0x10)
    vibration_amplitudeH = 0x10;
  
  haptic_config = 0;
  ecg_SetParameter(ECG_COMMAND, sizeof (uint8), &haptic_config);
}



//******************************************************************************