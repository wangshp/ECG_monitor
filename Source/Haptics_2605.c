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

//added from Haptics.c
// Private variables
static uint16_t tickdelay;
static uint8_t 	playEffect = 1;						// If 1 = play, 0 = do not play
static uint8_t 	EffectActive = 0;					// Flag if effect is playing
static uint8_t 	triggerType = TRIGGER_INTERNAL;		// Trigger Setting
static uint8_t 	actuatorType = ACTUATOR_LRA;		// Actuator Setting
static uint8_t  disableAllowed = 1;					// Flag if hardware disable is allowed
static uint8_t  IsERMOpenLoop = ERM_ClosedLoop;		// Flag for ERM Open Loop
static uint8_t  IsLRAOpenLoop = LRA_AutoResonance;	// Flag for ERM Open Loop
static uint8_t  useERMAutoCalValues = 0;			// Flag to use ERM auto-cal values
static uint8_t  useLRAAutoCalValues = 0;			// Flag to use LRA auto-cal values

/* DRV260x Register Variables */
static uint8_t 	ERM_AutoCal_Comp, ERM_AutoCal_BEMF, ERM_AutoCal_FB;
static uint8_t 	LRA_AutoCal_Comp, LRA_AutoCal_BEMF, LRA_AutoCal_FB;
static uint8_t 	ERM_RatedVoltage, ERM_ODClamp, ERM_ODClamp_OL;
static uint8_t 	LRA_RatedVoltage, LRA_ODClamp, LRA_ODClamp_OL;
static uint8_t 	AutoCal_Result;
static uint8_t 	control1, control2, control3;
static uint8_t  ERMDefaultOpenLoopSetting, LRADefaultOpenLoopSetting;
uint16_t RDS_Comp_temp;

/* DRV260xL Register Variables ('L' version only) */
static uint8_t	control5;


//added from BinaryModes
unsigned int defaultActuator = ACTUATOR_LRA;		// Actuator used when no actuator is specified



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
  
  buffer_w2[0] = DRV2605_MODE;
  buffer_w2[1] = Dev_Reset;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
 
  buffer_w2[0] = DRV2605_STATUS;       
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &CONTROL1_Values);
  
  buffer_w2[0] = DRV2605_MODE;        // Come out of STANDBY
  buffer_w2[1] = ACTIVE;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_STATUS;       
  HalI2CWrite(HAPTICS_ID, 1, buffer_w2, 0);
  HalI2CRead(HAPTICS_ID, 1, &CONTROL1_Values);
  //init
    control1 = DEFAULT_CTRL1;
    control2 = DEFAULT_CTRL2;
    control3 = DEFAULT_CTRL3;
        
    ERM_ODClamp_OL = RatedVoltage_3p3;
    ERM_RatedVoltage = RatedVoltage_1p3;
    ERM_AutoCal_FB = ERM_AUTOCAL_FB;
    ERM_AutoCal_Comp = ERM_AUTOCAL_COMP;
    ERM_AutoCal_BEMF = ERM_AUTOCAL_BEMF;

  
  
  buffer_w2[0] = DRV2605_WAVEFORMSEQ1;  //could set all wave register to identifier 1
  buffer_w2[1] = 0x01;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  buffer_w2[0] = DRV2605_WAVEFORMSEQ2;
  buffer_w2[1] = 0x00;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);


 
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
    
  buffer_w2[0] = 0x13;
  buffer_w2[1] = 0x64;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1); //A2H Max Peak

  static signed char amplitude = 0x7F;
  Haptics_SetERMOpenLoopMode(0);						// Set ERM closed-loop mode

  /* Test Click waveform data */
  unsigned char RTP_onofflifetest_data[] = {
                  amplitude, 0xFF,
                  amplitude, 0x90,
                  0x00, 0xC8
  };
  const Waveform RTP_onoff = {MODE_RTP,6,RTP_onofflifetest_data};
  
  Haptics_SetActuator(defaultActuator);
  
  Haptics_SetTrigger(TRIGGER_INTERNAL);		// Select the trigger type

  Haptics_Start(RTP_onoff.inputMode);			// Enable the amplifier, begin PWM

  Haptics_HardwareMode(RTP_onoff.inputMode);	// Configure the Hardware

  Haptics_SendTriggerType();			// Send trigger type to DRV260x
  
  Haptics_SendActuatorSettings();				// Send actuator settings to DRV260x

  // Set ERM closed-loop mode
  buffer_w2[0] = DRV2605_FEEDBACK_CONTROL;
  buffer_w2[1] = ERM_MODE | FBBrakeFactor_4x | LoopResponse_Fast;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_OD_CLAMP;
  buffer_w2[1] = RatedVoltage_3p3;
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  buffer_w2[0] = DRV2605_CONTROL3;
  buffer_w2[1] = NGThresh_4PERCENT | ERM_OpenLoop | DataFormat_RTP_Signed | LRADriveMode_Once | InputMode_PWM | LRA_OpenLoop;  
  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
  
  Haptics_OutputEnableSet(1);
  
  while(1)
  {
          Haptics_OutputWaveform(RTP_onoff);		// Send waveform data to driver
  }
}


void Haptics_SetERMOpenLoopMode(uint8_t openLoop)
{
	IsERMOpenLoop = openLoop;
}


void Haptics_SetActuator(uint8 actuator)
{
	actuatorType = actuator;
}

/**
 * Haptics_SetTrigger - set the trigger DRV260x trigger type, call before Haptics_HardwareMode()
 * @param unsigned char trigger - trigger constant
 */
void Haptics_SetTrigger(uint8_t trigger)
{
	triggerType = trigger;
}

void Haptics_Start(uint8_t inputMode)
{
	if(playEffect)
	{
		Haptics_EnableAmplifier();
	}
}

void Haptics_EnableAmplifier(void)
{
  HAPTICS_ENABLE_PIN = 1; // Set enable on
}

void Haptics_HardwareMode(uint8_t inputMode)
{
	P0SEL &= ~(0x40); // Init Enable_pin to output
        P0DIR |= 0x40;
        HAPTICS_ENABLE_PIN = 1; // Set enable on
        enabled = true;
	
}

/**
 * Haptics_SendTriggerType - set the trigger settings in the DRV260x
 */
void Haptics_SendTriggerType()
{
	/* Set the trigger mode in the DRV260x */
	buffer_w2[0] = DRV2605_MODE;
        buffer_w2[1] = Int_Trig;
        HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
}

/**
 * Haptics_SendActuatorSettings - set the actuator settings in the DRV260x
 */
void Haptics_SendActuatorSettings()
{
	/* DRV2605 Actuator Settings */
	switch(actuatorType)
	{
	case ACTUATOR_ERM: 	// ERM Mode

		if(IsERMOpenLoop)	// Open Loop Mode
		{
                  buffer_w2[0] = DRV2605_OD_CLAMP;
                  buffer_w2[1] = ERM_ODClamp_OL;     // Open-loop Overdrive
                  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
                  
                  buffer_w2[0] = DRV2605_CONTROL3;
                  buffer_w2[1] = control3 | ERM_OpenLoop;   // Set open-loop
                  HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
		}
		else				// Closed Loop Mode
		{
			//I2C_WriteSingleByte(DRV2605_OD_CLAMP, ERM_ODClamp);						// Closed-loop Overdrive
			//I2C_WriteSingleByte(DRV2605_CONTROL3, (control3 & ~(ERM_OpenLoop)));	// Set closed-loop
		}
                
                buffer_w2[0] = DRV2605_RATED_VOLTAGE;
                buffer_w2[1] = ERM_RatedVoltage;     // Open-loop Overdrive
                HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);

                buffer_w2[0] = DRV2605_AUTOCAL_COMP;
                buffer_w2[1] = ERM_AutoCal_Comp;     // Open-loop Overdrive
                HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
                
                buffer_w2[0] = DRV2605_AUTOCAL_BEMF;
                buffer_w2[1] = ERM_AutoCal_BEMF;     // Open-loop Overdrive
                HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
                
                buffer_w2[0] = DRV2605_FEEDBACK_CONTROL;
                buffer_w2[1] = ERM_AutoCal_FB;     // Open-loop Overdrive
                HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
                
		break;
	
	default: break;
	}
}

/**
 * Haptics_SendWaveform - send haptic waveform
 * @param struct Waveform - the waveform output type, length in bytes, and data
 */
void Haptics_OutputWaveform(const Waveform waveform)
{
	uint8_t i, j, k;
	uint8_t ReadVal;

	if(playEffect)
	{

		switch(waveform.inputMode)
		{
		case MODE_RAM_OL_1ms:
			break;
		case MODE_RAM_OL_5ms:
			break;
		case MODE_RAM:			// RAM Mode
			break;
		case MODE_ROM:		// ROM Mode 
                  /*
                  for(i = 0; i < waveform.length; i += 3)
			{
				if(waveform.data[i+2] != 0xFF)		// Read register, Operate, Write register
				{
					ReadVal = I2C_ReadSingleByte(waveform.data[i]);
					ReadVal = waveform.data[i+1] | (ReadVal & ~waveform.data[i+2]); 	// Apply Mask to ReadData
					I2C_WriteSingleByte(waveform.data[i], ReadVal);   					// Send (register, value)
				}
				else		// Write over existing register setting
				{
					I2C_WriteSingleByte(waveform.data[i], waveform.data[(i+1)]);   // Send (register, value)
				}
			}

			Haptics_EnableTrigger();		// Set ROM trigger (Internal, Ext. Edge, Ext. Level)
                    */
			break;

		case MODE_RTP:		// RTP Mode

                        buffer_w2[0] = DRV2605_MODE;
                        buffer_w2[1] = RTP;                 // Set RTP Mode
                        HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);	
                        
			for(k=0; k<waveform.length; k=k+2)			// Go through waveform data array of time-value pairs
			{
                           buffer_w2[0] = DRV2605_RTP;
                           buffer_w2[1] = waveform.data[k];       // Send (register, value)          
                           HalI2CWrite(HAPTICS_ID, 2, buffer_w2, 1);
                            
				
                            for(j=0;j<waveform.data[k+1];++j)		// Count number of 5ms intervals
                            {
                                    int time_count = 10000;				// Time delay before change in amplitude
                                    while(time_count)
                                      time_count--;
                            }
			}

			break;
		default:
              /*
			for(i = 0; i < waveform.length; i += 3)
			{
				if(waveform.data[i+2] != 0xFF)		// Read register, Operate, Write register
				{
					ReadVal = I2C_ReadSingleByte(waveform.data[i]);
					ReadVal = waveform.data[i+1] | (ReadVal & ~waveform.data[i+2]); // Apply Mask to ReadData
					I2C_WriteSingleByte(waveform.data[i], ReadVal);   				// Send (register, value)
				}
				else		// Write over existing register setting
				{
					I2C_WriteSingleByte(waveform.data[i], waveform.data[(i+1)]);   	// Send (register, value)
				}
			}

			Haptics_EnableTrigger();		// Set ROM trigger (Internal, Ext. Edge, Ext. Level)
                        */
			break;
		}
	}
}

/**
 * Haptics_OutputEnableSet - enable/disable play back of all waveforms
 */
void Haptics_OutputEnableSet(uint8_t enable)
{
	playEffect = enable;
}