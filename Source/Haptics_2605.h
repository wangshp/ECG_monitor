/******************************************************************************
 * Haptics.c
 *
 * Created on: April 21, 2012
 * Board: DRV2605EVK-CT
 *
 * Description: This file contains the functions for sending haptics waveforms.
 * 		Edit this file when changing actuators.
 *
 * Modified:
 *
 ******************************************************************************/

#ifndef _HAPTICS_2605_H_
#define _HAPTICS_2605_H_
  
#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_types.h" // Strange import problem with types - quick fix for now

#define AUTOCAL_LRA_EVENT 0x0001
#define DELAYED_WAVEFORM 0x0002

// DRV2605 Trigger Modes
#define INTERNAL_TRIGGER		        0
#define EXTERNAL_EDGE_TRIGGER	                1
#define EXTERNAL_LEVEL_TRIGGER	                2

// DRV2605 Actuator
#define ACTUATOR_ERM			        0
#define ACTUATOR_LRA			        1

// DRV2605 Analog Mode Coupling
#define DC_COUPLED				0
#define AC_COUPLED				1

#define HAPTICS_ENABLE_PIN P0_6 // Change Port init in Haptics_init after

// user
#define HAPTICS_ID 0x5A
//1011 0100 read


// Waveform Structure Type Definition
typedef struct Haptics_Waveform {
	const unsigned char 	inputMode; 		//See Input Modes Above
	const unsigned char	length;			// size of array in bytes
	const unsigned char* 	data;			// pointer to waveform array data (waveform array is in (amplitude, time) pairs
} Waveform;

static void Haptics_WaitUs(uint16 microSecs);

void Haptics_Enable(void);

/**
 * Haptics_Init - initialize haptics variables and settings
 */




// Default Control Register Settings
#define DEFAULT_CTRL1	StartupBoost | DriveTime_2p4m
#define DEFAULT_CTRL2	BiDir_Input| BrakeStabilizer | SampleTime_300us | BlankingTime_Short | IDissTime_Short
#define DEFAULT_CTRL3	NGThresh_4PERCENT | ERM_ClosedLoop | DataFormat_RTP_Signed | LRADriveMode_Once | InputMode_PWM | LRA_AutoResonance

/**
 * Haptics_OutputEnableSet - enable/disable play back of all waveforms
 */

//added
// DRV2605 Input Modes
#define MODE_RAM				0	// RAM Playback Mode
#define MODE_ROM				1	// ROM Playback Mode (Reserved for DRV2605)
#define MODE_PWM				2	// PWM Playback Mode
#define MODE_RTP				3	// RTP Real-Time Playback Mode
#define MODE_ANALOG				4	// Analog Input Mode
#define MODE_A2H				5	// Audio-to-Haptics Mode
#define MODE_DEFAULT			6   // Special RAM mode
#define MODE_RAM_OL_5ms			7	// Open-Loop RAM Mode 5ms Intervals
#define MODE_RAM_OL_1ms			8	// Open-Loop RAM Mode 1ms Intervals


// DRV260x Trigger Modes
#define TRIGGER_INTERNAL		0	// Internal trigger mode
#define TRIGGER_EXTERNAL_EDGE	1	// External trigger, Edge mode
#define TRIGGER_EXTERNAL_LEVEL	2	// External trigger, Level mode

// Default Control Register Settings
#define DEFAULT_CTRL1	StartupBoost | DriveTime_2p4m
#define DEFAULT_CTRL2	BiDir_Input| BrakeStabilizer | SampleTime_300us | BlankingTime_Short | IDissTime_Short
#define DEFAULT_CTRL3	NGThresh_4PERCENT | ERM_ClosedLoop | DataFormat_RTP_Signed | LRADriveMode_Once | InputMode_PWM | LRA_AutoResonance

//CHOOSE RANDOM SETTING
#define ERM_AUTOCAL_COMP		0x0D					// Compensation Coef.
#define ERM_AUTOCAL_BEMF		0x6E					// BEMF Coef.
#define ERM_AUTOCAL_FB			0x36					// Feedback Control

static void Haptics_WaitUs(uint16 microSecs);

void Haptics_Enable(void);

void Haptics_Init(void);

void haptic_process(void);

//void haptic_config_update(void);

#ifdef __cplusplus
}
#endif

#endif /* _HAPTICS_2605_H_ */
