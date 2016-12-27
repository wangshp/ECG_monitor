//------------------------------------------------------------------------------
//  Description:  Header file for TI_CC254x_spi.c
//
//  CC254x/ADS1293 Interface Code Library v1.1
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

#ifndef HEADER_FILE_TI_CC254x_SPI_H
#define HEADER_FILE_TI_CC254x_SPI_H

void TI_ADS1293_SPISetup(void);
void TI_ADS1293_SPIWriteReg(uint8, uint8);
uint8 TI_ADS1293_SPIReadReg(uint8);
void TI_ADS1293_SPIAutoIncWriteReg(uint8, uint8 *, uint8);
void TI_ADS1293_SPIAutoIncReadReg(uint8, uint8 *, uint8);
void TI_ADS1293_SPIStreamReadReg(uint8 *, uint8);
void TI_ADS1293_WriteRegSettings(void);
void ADS_WRITE_command(uint8 command);
void spiReadByte(uint8 *read, uint8 write);
void spiWriteByte(uint8 write);


#endif                                                                         // HEADER_FILE_TI_CC254x_SPI_H
