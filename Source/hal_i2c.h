#ifndef HAL_I2C_H
#define HAL_I2C_H


/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_types.h"
#if (defined HAL_I2C) && (HAL_I2C == TRUE)

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_I2C TRUE
#define HAL_I2C_MASTER TRUE

#if !defined HAL_I2C_MASTER
#define HAL_I2C_MASTER  FALSE
#endif

#if !defined HAL_I2C_SLAVE
#if HAL_I2C_MASTER
#define HAL_I2C_SLAVE   FALSE
#else
#define HAL_I2C_SLAVE   TRUE
#endif
#endif

#if HAL_I2C_MASTER
#define HAL_I2C_POLLED  FALSE  // Master does not use ISR and does not need periodic polling.
#else // if HAL_I2C_SLAVE
#if !defined HAL_I2C_POLLED
#define HAL_I2C_POLLED  FALSE  // Prefer the ISR as a Slave to speed reaction to Master READ req.
#endif
#endif

#if !defined HAL_I2C_BUF_MAX
#define HAL_I2C_BUF_MAX                  255
#endif

#define HAL_I2C_SLAVE_ADDR_DEF           0x41

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

#if (HAL_I2C_BUF_MAX < 256)
typedef uint8  i2cLen_t;
#else
typedef uint16 i2cLen_t;
#endif

#if HAL_I2C_MASTER
typedef enum {
  i2cClock_123KHZ = 0x00,
  i2cClock_144KHZ = 0x01,
  i2cClock_165KHZ = 0x02,
  i2cClock_197KHZ = 0x03,
  i2cClock_33KHZ  = 0x80,
  i2cClock_267KHZ = 0x81,
  i2cClock_533KHZ = 0x82
} i2cClock_t;
#else // if HAL_I2C_SLAVE
/**************************************************************************************************
 * @fn          i2cCallback_t
 *
 * @brief       Slave mode callback to Application to alert of a Master request to read or of
 *              received bytes from a Master write.
 *
 * input parameters
 *
 * @param       cnt - Zero indicates Master read request and slave Tx buffer empty. Non-zero
 *                    indicates the number of bytes received from a Master write,
 *                    this data is in the slave Rx buffer, ready to be read.
 *
 * output parameters
 *
 * None.
 *
 * @return      N/A when cnt is non-zero; otherwise TRUE if I2C should continue to clock stretch,
 *              FALSE to send a zero with last byte indication.
 */
typedef uint8 (*i2cCallback_t)(uint8 cnt);
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Global Functions
 * ------------------------------------------------------------------------------------------------
 */

void HalI2CInit(i2cClock_t clockRate);
i2cLen_t HalI2CRead(uint8 address, i2cLen_t len, uint8 *pBuf);
i2cLen_t HalI2CWrite(uint8 address, i2cLen_t len, uint8 *pBuf, bool stp);

uint8 HalI2CReady2Sleep(void);
void HalI2CEnterSleep(void);
void HalI2CExitSleep(void);

#endif
#endif
/**************************************************************************************************
 */