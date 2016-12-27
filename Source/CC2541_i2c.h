
#ifndef HEADER_CC2541_I2C_H

#define HEADER_CC2541_I2C_H

// IEN2 (0x9A) - Interrupt Enable 2 Register
#define IEN2_P2IE                         0x02

// I2CCFG (0x6230) I2C Control
#define I2CCFG_CR2                        0x80    // Clock rate bit 2
#define I2CCFG_ENS1                       0x40    // Enable I2C
#define I2CCFG_STA                        0x20    // Start condition flag
#define I2CCFG_STO                        0x10    // Stop flag
#define I2CCFG_SI                         0x08    // Interrupt flag
#define I2CCFG_AA                         0x04    // Assert acknowledge flag
#define I2CCFG_CR1                        0x02    // Clock rate bit 1
#define I2CCFG_CR0                        0x01    // Clock rate bit 0
#define I2CCFG_CR                         (0x83)  // Clock rate bit mask
#define I2CCFG_CR_DIV_256                 (0x00)  // 123 kHz at 32MHz system clock
#define I2CCFG_CR_DIV_244                 (0x01)  // 144 kHz at 32MHz system clock
#define I2CCFG_CR_DIV_192                 (0x02)  // 165 kHz at 32MHz system clock
#define I2CCFG_CR_DIV_160                 (0x03)  // 197 kHz at 32MHz system clock
#define I2CCFG_CR_DIV_960                 (0x80)  //  33 kHz at 32MHz system clock
#define I2CCFG_CR_DIV_120                 (0x81)  // 267 kHz at 32MHz system clock
#define I2CCFG_CR_DIV_60                  (0x82)  // 533 kHz at 32MHz system clock
  









// Define size of buffer and number of bytes to send.
#define I2C_BUFFER_SIZE 0xFF
#define I2C_SLAVE_ADDRESS 0x53    // 7-bit addressing.
#define I2C_HR_WADD 0xAE
#define I2C_HR_RADD 0xAF

#define I2C_READ_FROM_SLAVE 0x01
#define I2C_WRITE_TO_SLAVE 0x00


#endif                                                        // HEADER_FILE_TI_ADS1293_H

