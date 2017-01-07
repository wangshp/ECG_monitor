/******************************************************************************

 @file  KeyFob_Main.c

 @brief This file contains the main and callback functions

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2009-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

#include "ioCC2541.h"

#include "HR_module.h"
#include "hal_i2c.h"

/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/

/* This callback is triggered when a key is pressed */
void MSA_Main_KeyCallback(uint8 keys, uint8 state);

/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */

#define BIT1        0x02
#define BIT0        0x01
#define ACC_X_LSB                   0x02 

#define HAL_I2C TRUE
#define HAL_I2C_MASTER TRUE


uint8 *new_x;
uint8 x[20],i;
uint8 len;
uint8 buffer_w[2];
uint8 checkpoint1;
int main(void)
{

    P1SEL &= ~BIT0;      // GPIO.
    P1DIR |= BIT0;      // Output.
    P1_0 = 0;           // LED1 off.
    
    uint32 a = 10000;
    
    
    
      /* Configure DRDYB (P2_0) as interrupt for HR module */ 
    P2DIR &= ~0x01;     //P2_0 as input                                                         // pin1.7 is input
    PICTL |= 0x08;     // falling edge interrupt
    IRCON2 &= ~0x01;   // clear Port 2 interrupt flag
    P2IFG &= ~0x01;    // clear Port2.0 pin status flag
    
    P2IEN |= 0x01;    // enable P2_0 interrupt
   
    IEN2  |= 0x02;      // enable Port2 interrupt
    
   // EA = 0;   //to make sure interrupt only generated after all configuration
   
    IEN2  &= (~0x02);      // 111 disable Port2 interrupt-->to know if I2C work in this way or not? -->it work
    
    /***test for heart rate---I2C.  */ 
     /* Set up the I2C interface */
    HalI2CInit( i2cClock_165KHZ ); // I2C clock rate
    
    buffer_w[0] = INT_STATUS;      //clear power up interrupt
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1);
    
    buffer_w[0] = SPO2_CONFIG;
    buffer_w[1] = 0x20;         
    HalI2CWrite(HR_ID, 2, buffer_w, 1); 

    buffer_w[0] = SPO2_CONFIG;      
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1);
   
    buffer_w[0] = MODE_CFR;
    buffer_w[1] = 0x02;         //set HR only mode 
    HalI2CWrite(HR_ID, 2, buffer_w, 1); //after set mode, it triggers power ready interrupt.
    
    while(a--){}
    a =  1000;
    
    buffer_w[0] = SPO2_CONFIG;      
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1);
    
    buffer_w[0] = MODE_CFR;      
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1);    
    
    buffer_w[0] = MODE_CFR;      //clear power up interrupt
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1);
    
    
    buffer_w[0] = INT_EN;
    buffer_w[1] = 0x02;         //set HR only mode 
    HalI2CWrite(HR_ID, 2, buffer_w, 1); //after set mode, it triggers power ready interrupt.
    
    buffer_w[0] = INT_EN;      //clear power up interrupt
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1);
    
    
    
    /*configuration:
    buffer_w[0] = MODE_CFR;
    buffer_w[1] = 0x40;         //reset 
    HalI2CWrite(HR_ID, 2, buffer_w, 1);
    */
    buffer_w[0] = MODE_CFR;      //clear power up interrupt
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1); 
    
    buffer_w[0] = LED_PWC;
    buffer_w[1] = 0xFF;         //set 
    HalI2CWrite(HR_ID, 2, buffer_w, 1);
    
    buffer_w[0] = MODE_CFR;
    buffer_w[1] = 0x02;         //set HR only mode 
    HalI2CWrite(HR_ID, 2, buffer_w, 1); //after set mode, it triggers power ready interrupt.
    
    buffer_w[0] = MODE_CFR;      
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1); 
    
    buffer_w[0] = LED_PWC;      
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1);

    while(a--){}
    a =  1000;
    
    buffer_w[0] = LED_PWC;      //clear power up interrupt
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1);  

    
    
    buffer_w[0] = INT_EN;
    buffer_w[1] = 0xA0;         // enable only HR interrupt 
    HalI2CWrite(HR_ID, 2, buffer_w, 1);
    
 
    
    buffer_w[0] = LED_PWC;
    buffer_w[1] = 0xFF;         //set 
    HalI2CWrite(HR_ID, 2, buffer_w, 1);
    
    buffer_w[0] = SPO2_CONFIG;
    buffer_w[1] = 0x07;         //set 50SPS, LED pulse width 1600us
    HalI2CWrite(HR_ID, 2, buffer_w, 1);
    

    buffer_w[0] = INT_STATUS;      //clear power up interrupt
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 1, &checkpoint1);


    
    while(1)
    {
      
        while(a--){}
        a =  10000;
        P1_0 = ~ P1_0; 
        /*
        buffer_w[0] = INT_STATUS;      //clear power up interrupt
        HalI2CWrite(HR_ID, 1, buffer_w, 0);
        HalI2CRead(HR_ID, 1, &checkpoint1);
        
        if(checkpoint1 != 0)
          break;
        */
    }
    
    return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
