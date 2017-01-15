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
uint8 vibration_amplitudeH = 0x20;  // from 00~30 are fine!
uint8 vibration_amplitudeL = 0x00;
uint16 vibration_period = 20000;  //may need set period longer to let interrupt go.

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

  Haptics_WaitUs(20000);
}


/**
* Haptics_Init - initialize haptics variables and settings
* This should be run once at device power up
*/
void Haptics_Init(void)
{

  
  P0SEL &= ~(0x40); // Init Enable_pin to output
  P0DIR |= 0x40;
  Haptics_Enable();
  
 
  
  //Set Defaults
  buffer_w2[0] = DRV2605_MODE;      //reset
  buffer_w2[1] = Dev_Reset;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
 
  
  buffer_w2[0] = DRV2605_MODE;        // Come out of STANDBY
  buffer_w2[1] = ACTIVE;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
 
  
  buffer_w2[0] = DRV2605_OD_CLAMP;
  buffer_w2[1] = RatedVoltage_1p8; //by changing the voltage at here, we could change its strength.
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_FEEDBACK_CONTROL;
  buffer_w2[1] = ERM_MODE | FBBrakeFactor_4x | LoopResponse_Fast;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_CONTROL3;
  buffer_w2[1] = NGThresh_4PERCENT | ERM_OpenLoop | DataFormat_RTP_Signed | LRADriveMode_Once | InputMode_PWM | LRA_OpenLoop;  
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);

  
  buffer_w2[0] = DRV2605_RTP;       
  buffer_w2[1] = vibration_amplitudeL;  //0x01 - 0x7f same direction, different amplitude
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);

  /*
  buffer_w2[0] = DRV2605_MODE;
  buffer_w2[1] = RTP;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
//  vibration_amplitudeH = vibration_amplitudeH + 0x10;
//  period = period + 10000;  
  while(1)
  {
    //0.9~1.6
    
    buffer_w2[0] = DRV2605_RTP;       
    buffer_w2[1] = vibration_amplitudeH;
    HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
    
    Haptics_WaitUs(vibration_period);
    
    buffer_w2[0] = DRV2605_RTP;       
    buffer_w2[1] = vibration_amplitudeL;
    HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
    
    Haptics_WaitUs(vibration_period);
    
    
    
  }
    buffer_w2[0] = DRV2605_RTP;       
    buffer_w2[1] = 0x10;
    HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1); //0.44v
    Haptics_WaitUs(20000);
    
    buffer_w2[0] = DRV2605_RTP;       
    buffer_w2[1] = 0x30;
    HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1); //1.32v //0.71
    Haptics_WaitUs(20000);
   
    buffer_w2[0] = DRV2605_RTP;       //2.20v acceptable.
    buffer_w2[1] = 0x50;
    HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
    Haptics_WaitUs(20000);
    
    buffer_w2[0] = DRV2605_RTP;       //3.0v  //1.66
    buffer_w2[1] = 0x70;     //ONCE I connect this voltage, the motor just stop work! also DRV stop work too.
    HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1); //DRV even couldn't communicate, only after repowered, it comes back
    Haptics_WaitUs(20000);
    
   */
}

extern uint8 haptic_start;
extern uint32 haptic_duration; //10000ms
extern uint8 vibrate_period; //200ms

uint8 haptic_vibrate = 0;
uint32 haptic_timer = 0;

void haptic_process(void)
{
  if(haptic_start == 1)  //set at outside when writing.
  { 
    if(haptic_timer == 0)
    {
    
    buffer_w2[0] = DRV2605_MODE;
    buffer_w2[1] = RTP;
    HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
    }
    
    if(haptic_timer < (haptic_duration / 20))  //DEFAULT_ECG_PERIOD = 20
    {  
      if((haptic_timer % (vibrate_period / 20)) == 0)
      {
        if(haptic_vibrate == 1)
        {
          buffer_w2[0] = DRV2605_RTP;       
          buffer_w2[1] = vibration_amplitudeL;
          HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1); //0.44v
          haptic_vibrate = 0;
        }
        else
        {
          //if()
          buffer_w2[0] = DRV2605_RTP;       
          buffer_w2[1] = vibration_amplitudeH;
          HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1); //0.44v
          haptic_vibrate = 1;
        }
      }
      haptic_timer++; 
    }
    else
    {
      haptic_start = 0;
      haptic_timer = 0;
      
      buffer_w2[0] = DRV2605_RTP;       
      buffer_w2[1] = vibration_amplitudeL;
      HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1); //set to 0.
      
      buffer_w2[0] = DRV2605_MODE;
      buffer_w2[1] = STANDBY;
      HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
    }
            
        
  }
}

/*
void haptic_config_update(void)
{
  ecg_GetParameter(ECG_COMMAND);
}

*/ 