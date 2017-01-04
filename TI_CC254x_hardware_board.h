//----------------------------------------------------------------------------
//  Description:  This file contains definitions specific to the hardware board.
//  Specifically, the definitions include hardware connections with the
//  ADS1293 connector port, LEDs, and switches.
//
//  CC254x/ADS1293 Interface Code Library v1.0
//
//   Vishy Natarajan
//   Texas Instruments Inc.
//   February 2012
//   Built with CCE Version: 4.2 and IAR Embedded Workbench Version:  5.3x
//------------------------------------------------------------------------------
// Change Log:
//------------------------------------------------------------------------------
// Version:  1.00
// Comments: Initial Release Version
//------------------------------------------------------------------------------
#ifndef HEADER_TI_CC254x_HARDWARE_BOARD_H

#define HEADER_FILE_TI_CC254x_HARDWARE_BOARD_H

// ADS1293 connected at:
// P1_6 = ALARM
// P1_2 = /CS
// P1_7 = /DRDY

#define CS              P1_2

#define DRDYB           P1_7
#define ALARM           P1_6

#define CS_DISABLED     1
#define CS_ENABLED      0

#define LED             P1_0
#define LED_ENABLED     1
#define LED_DISABLED    0

#endif                                                                         // HEADER_FILE_TI_CC254x_HARDWARE_BOARD_H
