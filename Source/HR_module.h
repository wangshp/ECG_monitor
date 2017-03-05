#ifndef HR_MODULE
#define HR_MODULE

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <hal_board_cfg.h>

#define HR_ID           0x57    //0101 0111
#define HR_R_ID         0xAF  //1010 1111

#define MODE_CFR        0x06
#define INT_EN          0x01
#define PART_ID         0xFF
#define FIFO_DATA       0x05
#define SPO2_CONFIG     0x07
#define LED_PWC         0x09
#define INT_STATUS      0x00  
#define FIFO_W_POINTER  0x02
#define FIFO_R_POINTER  0x04
#endif
/**************************************************************************************************
 */