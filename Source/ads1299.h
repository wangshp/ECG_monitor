#ifndef ADS1299_H
#define ADS1299_H

#include <hal_board_cfg.h>

/* ADS1299 */
#define CS_ADS      P1_2
#define SCLK_ADS    P1_3
#define SDO_ADS     P1_4
#define SDI_ADS     P1_5
#define START_ADS   P1_6
#define DRDY_ADS    P1_7  
#define PWDN_ADS    P0_7
#define RESET_ADS   P0_0

#define START 0x01
#define STOP 0x02
#define RDATA 0x03
#define RREG 0x04
#define WREG 0x05


#define _WAKEUP  0x02 // Wake-up from standby mode
#define _STANDBY 0x04 // Enter Standby mode
#define _RESET   0x06 // Reset the device registers to default
#define _START   0x08 // Start and restart (synchronize) conversions
#define _STOP    0x0A // Stop conversion
#define _RDATAC  0x10 // Enable Read Data Continuous mode (default mode at power-up)
#define _SDATAC  0x11 // Stop Read Data Continuous mode
#define _RDATA   0x12 // Read data by command; supports multiple read back
#define _RREG    0x20 // Read Register
#define _WREG    0x40 // Write to Register

// Register Addresses
#define ID         0x00
#define CONFIG1    0x01
#define CONFIG2    0x02
#define CONFIG3    0x03
#define LOFF       0x04
#define CH1SET     0x05
#define CH2SET     0x06
#define CH3SET     0x07
#define CH4SET     0x08
#define CH5SET     0x09
#define CH6SET     0x0A
#define CH7SET     0x0B
#define CH8SET     0x0C
#define BIAS_SENSP 0x0D
#define BIAS_SENSN 0x0E
#define LOFF_SENSP 0x0F
#define LOFF_SENSN 0x10
#define LOFF_FLIP  0x11
#define LOFF_STATP 0x12
#define LOFF_STATN 0x13
#define GPIO       0x14
#define MISC1      0x15
#define MISC2      0x16
#define CONFIG4    0x17

// Gains
#define ADS1299_PGA_GAIN01 0x00         //(0b00000000)
#define ADS1299_PGA_GAIN02 0x10         //(0b00010000)
#define ADS1299_PGA_GAIN04 0x20         //(0b00100000)
#define ADS1299_PGA_GAIN06 0x30         //(0b00110000)
#define ADS1299_PGA_GAIN08 0x40         //(0b01000000)
#define ADS1299_PGA_GAIN12 0x50         //(0b01010000)
#define ADS1299_PGA_GAIN24 0x60         //(0b01100000)

// Input Modes - Channels

#define ADS1299_INPUT_PWR_DOWN   0x80   //(0b10000000)
#define ADS1299_INPUT_PWR_UP     0x00   //(0b00000000)

#define ADS1299_INPUT_NORMAL     0x00   //(0b00000000)
#define ADS1299_INPUT_SHORTED    0x01   //(0b00000001)
#define ADS1299_INPUT_MEAS_BIAS  0x02   //(0b00000010)
#define ADS1299_INPUT_SUPPLY     0x03   //(0b00000011)
#define ADS1299_INPUT_TEMP       0x04   //(0b00000100)
#define ADS1299_INPUT_TESTSIGNAL 0x50   //(0b00000101)
#define ADS1299_INPUT_SET_BIASP  0x60   //(0b00000110)
#define ADS1299_INPUT_SET_BIASN  0x70   //(0b00000111)

// Test Signal Choices - p41
#define ADS1299_TEST_INT              0x10      //(0b00010000)
#define ADS1299_TESTSIGNAL_AMP_1X     0x00      //(0b00000000)
#define ADS1299_TESTSIGNAL_AMP_2X     0x04      //(0b00000100)
#define ADS1299_TESTSIGNAL_PULSE_SLOW 0x00      //(0b00000000)
#define ADS1299_TESTSIGNAL_PULSE_FAST 0x01      //(0b00000001)
#define ADS1299_TESTSIGNAL_DCSIG      0x03      //(0b00000011)
#define ADS1299_TESTSIGNAL_NOCHANGE   0xFF      //(0b11111111)

//Lead-off Signal Choices
#define LOFF_MAG_6NA       0x00         //(0b00000000)
#define LOFF_MAG_24NA      0x04         //(0b00000100)
#define LOFF_MAG_6UA       0x08         //(0b00001000)
#define LOFF_MAG_24UA      0x0C         //(0b00001100)
#define LOFF_FREQ_DC       0x00         //(0b00000000)
#define LOFF_FREQ_7p8HZ    0x01         //(0b00000001)
#define LOFF_FREQ_31p2HZ   0x02         //(0b00000010)
#define LOFF_FREQ_FS_4     0x03         //(0b00000011)
#define PCHAN (1)
#define NCHAN (2)
#define BOTHCHAN (3)

#define OFF (0)
#define ON (1)


//timer1 delay

#define P2DIR_PRIP0_T1_0_1                (0x02 << 6) // Timer 1 channels 0-1 has priority, then USART 1, then USART 0, then Timer 1 channels 2-3
#define T1STAT_CH0IF                      0x01    // Overflow interrupt flag
#define T1CCTLn_CMP                       (0x07 << 3)     // Compare-mode bit mask.
#define T1CCTLn_CMP_SET_ON_CMP            (0x00 << 3)     // Set output on compare.
#define T1CCTLn_MODE                      0x04          // Compare mode when set, capture mode when cleared
#define T1CTL_MODE                        (0x03)        // Timer 1 mode select. The timer operating mode is selected as follows:
#define T1CTL_DIV                         (0x0C)        // Bit mask, Timer 1 tick speed divider 
#define T1CTL_MODE_FREERUN                (0x01)        // Free-running, repeatedly count from 0x0000 to 0xFFFF.
#define T1CTL_DIV_32                      (0x02 << 2)   // Divide tick frequency by 32
#define T1CTL_DIV_1                     (0x00 << 2)

void ads1299_stop_dataread(void);
void ads1299_write_reg(uint8 ADDR, uint8 VAL);
void ads1299_read_reg(uint8 ADR, uint8 *register_data);

void ads1299_set_up(void);
void TI_ADS1293_SPIStreamRead(uint8 *buffer, uint8 count);

//T1 timer delay
void delay_init(void);
void ms_delay(int msec);
void us_delay(int usec);

//not sure
void ads1299_read_data(uint8 *ads_status, uint8 *ads_data);
#endif