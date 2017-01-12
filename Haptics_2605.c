/******************************************************************************
* Haptics.c
*
* Created on: Dec 16, 2011
* Board: DRV2605EVK-CT RevA
*
* Description: This file contains the functions for sending haptics waveforms.
* 		Edit this file when changing actuators.
*
* @TODO - Update this file when changing between actuators types
* 			(ERM, LRA, Piezo)
*
******************************************************************************/
//Last modified by Gautham Ramachandran on 10/21/2014


#include "Haptics_2605.h"
#include "DRV2605.h"
#include "OnBoard.h"
#include "osal.h"
#include "hal_i2c.h"


#define I2C_HAPTICS_ADDRESS 0x5A

#define ST_HAPTICBK_SENSOR_EVT  0x0200



// private variables
static bool LRAmode = FALSE;  //TRUE
static uint8 AutoCal_Comp, AutoCal_BEMF, AutoCal_FB;
static uint8 RatedVoltage, ODClamp;
static uint8 AutoCal_Result;
static uint8 CONTROL1_Values, CONTROL2_Values, CONTROL3_Values;

static uint8 mode = 0;
static bool enabled = false;
static uint16 delay = 10;
static uint8 repetitions = 0;
static uint8 sequenceCount = 0;
static bool infinite = false;
static uint8 waveSequence[8] = {0,0,0,0,0,0,0,0};

static uint8 st_TaskID = 0x00;

uint8 buffer_w2[2];
uint8 test_point1 = 0;


static void Haptics_WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

// Function start


void Haptics_Enable(void){
  HAPTICS_ENABLE_PIN = 1; // Set enable on
  enabled = true;
  Haptics_WaitUs(20000);
}


/**
* Haptics_Init - initialize haptics variables and settings
* This should be run once at device power up
*/
void Haptics_Init(uint8 task_id)
{

  // Initialize DRV2605
  st_TaskID = task_id;
  
  P0SEL &= ~(0x40); // Init Enable_pin to output
  P0DIR |= 0x40;
  Haptics_Enable();
  
  buffer_w2[0] = DRV2605_STATUS;       
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &CONTROL1_Values); //DRV2605_CONTROL1:1001 0011
  //DRV2605_STATUS:1110 0000
  
  //Set Defaults
  buffer_w2[0] = DRV2605_MODE;        // Come out of STANDBY
  buffer_w2[1] = ACTIVE;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_WAVEFORMSEQ1;  //could set all wave register to identifier 1
  buffer_w2[1] = 0x01;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  buffer_w2[0] = DRV2605_WAVEFORMSEQ2;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  buffer_w2[0] = DRV2605_WAVEFORMSEQ3;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  buffer_w2[0] = DRV2605_WAVEFORMSEQ4;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  buffer_w2[0] = DRV2605_WAVEFORMSEQ5;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  buffer_w2[0] = DRV2605_WAVEFORMSEQ6;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  buffer_w2[0] = DRV2605_WAVEFORMSEQ7;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);


  /*
  buffer_w2[0] = DRV2605_MODE;
  buffer_w2[1] = Dev_Reset;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_MODE;          //test if reset work? --work
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &test_point1);
   
  buffer_w2[0] = DRV2605_RATED_VOLTAGE;          //test if reset work?
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &test_point1);
 */
  buffer_w2[0] = DRV2605_ODT;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_SPT;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_SNT;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_BRT;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_LIBRARY;  //choose ROM library
  buffer_w2[1] = RAM;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
    
  buffer_w2[0] = 0x13;
  buffer_w2[1] = 0x64;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1); //A2H Max Peak

  
  // Run auto Calibration
  if(LRAmode)
    Haptics_RunAutoCal_LRA();
  else
    Haptics_RunAutoCal_ERM();
}

void Haptics_RunAutoCal_LRA(void)
{
  buffer_w2[0] = DRV2605_MODE;
  buffer_w2[1] = Auto_Calibration;  
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  // Enable Amplifier
  
  // Run AutoCal LRA
  buffer_w2[0] = DRV2605_RATED_VOLTAGE;
  buffer_w2[1] = RatedVoltage_1p8;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_OD_CLAMP;
  buffer_w2[1] = RatedVoltage_3p0;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_FEEDBACK_CONTROL;
  buffer_w2[1] = LRA_MODE | FBBrakeFactor_2x | LoopResponse_Medium;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  

  
  buffer_w2[0] = DRV2605_AUTOCAL_MEMIF;
  buffer_w2[1] = AutoCalTime_500MS;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_GO;
  buffer_w2[1] = GO;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
}

void Haptics_RunAutoCal_ERM(void)
{
  // Run AutoCal ERM
  buffer_w2[0] = DRV2605_MODE;
  buffer_w2[1] = Auto_Calibration;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  // Enable Amplifier
  buffer_w2[0] = DRV2605_RATED_VOLTAGE;
  buffer_w2[1] = RatedVoltage_1p3;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_OD_CLAMP;//problem: read nothing back
  buffer_w2[1] = RatedVoltage_3p3;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_FEEDBACK_CONTROL;
  buffer_w2[1] = ERM_MODE | FBBrakeFactor_4x | LoopResponse_Fast;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);

  
  buffer_w2[0] = DRV2605_CONTROL1;  //problem: read nothing back!
  buffer_w2[1] = StartupBoost | DriveTime_2p4m; //0X93 1001 0011
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);

  buffer_w2[0] = DRV2605_CONTROL2;
  buffer_w2[1] = BiDir_Input | AutoResGain_Medium | BlankingTime_Short | IDissTime_Short;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_CONTROL3;
  buffer_w2[1] = NGThresh_4PERCENT | ERM_OpenLoop | DataFormat_RTP_Signed | LRADriveMode_Once | InputMode_PWM | LRA_OpenLoop;  
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  //Store Default Selections
  buffer_w2[0] = DRV2605_CONTROL1;       
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &CONTROL1_Values);
  
  buffer_w2[0] = DRV2605_CONTROL2;       
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &CONTROL2_Values);
  
  buffer_w2[0] = DRV2605_CONTROL3;       
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &CONTROL3_Values);
  

  
  buffer_w2[0] = DRV2605_AUTOCAL_MEMIF; //AUTO_CAL_TIME
  buffer_w2[1] = AutoCalTime_500MS;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);

  buffer_w2[0] = DRV2605_LIBRARY;          //test if before auto calibration finished, it is 1?
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &test_point1);
  
/*  */
  buffer_w2[0] = DRV2605_GO;
  buffer_w2[1] = GO;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);

  buffer_w2[0] = DRV2605_STATUS;          //test if before auto calibration finished, it is 1?
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &test_point1);
  
  while(1)
  {
    buffer_w2[0] = DRV2605_GO;           //wait for auto caliration finished.
    HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
    HalI2CRead(HAPTICS_ID, 1, &test_point1);
    
    
    if(test_point1 == 0)
      break;
  }
  
    buffer_w2[0] = DRV2605_AUTOCAL_COMP;          //test if caliration setting output anything.--it works!.
    HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
    HalI2CRead(HAPTICS_ID, 1, &test_point1);  
  
  buffer_w2[0] = DRV2605_STATUS;       //test if after auto calibration finished, it is 0? 
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &test_point1);
}
