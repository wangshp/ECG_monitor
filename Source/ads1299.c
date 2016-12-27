#include <ads1299.h>
#include <hal_types.h>
#include "TI_CC254x_spi.h"


void ads1299_set_up(void)
{ 
  START_ADS = 0;   //TEST R28
  uint8 space[2]; 
 
  
  ms_delay(1);   //internal oscillator
  
  PWDN_ADS = 1;  //P0_7
  RESET_ADS = 1;  //R38
  ms_delay(200);    //??? test point:
  //should check time using oscilloscope VCAP1>1.1v
  
  

/*  ADS_WRITE_command(_RESET);
  //18*(1/(2*10^6))
  ms_delay(1);
*/
  //RESET
  RESET_ADS = 1;
  us_delay(5);
  RESET_ADS = 0;
  us_delay(5);
  RESET_ADS = 1;
  us_delay(20);
  
 
  ADS_WRITE_command(_SDATAC);  //before read
  us_delay(4);
  
  ads1299_read_reg(CH1SET, space);  //0110 0001after reset, test if reset work
  us_delay(20);
  
  /*test 
  uint8 space[27];  //be able to read meaningless data at here.
   while(1)
  {
    
    ads1299_write_reg(CONFIG1, 0x96);
    us_delay(4);
    ads1299_read_reg(CONFIG1, space);
    us_delay(50);
  }
  */
  
  
  
  
  
  /*test
  uint8 space1[4];
  while(1)
  {
  ads1299_read_reg(ID, space1);
  ms_delay(4);
  }
  */
  ads1299_write_reg(CONFIG3, 0xE0);  // 1110 0000
  us_delay(4);
  
  
  ads1299_read_reg(CONFIG3, space); //just to make sure i set every register correctly
  us_delay(4);
  

  ads1299_write_reg(CONFIG1, 0x94); //changed
  us_delay(4);
  ads1299_read_reg(CONFIG1, space);// 1001 0110
  us_delay(4);
  
  /*read noise */
  ads1299_write_reg(CONFIG2, 0xC0); //1100 0000
  us_delay(4);
  ads1299_read_reg(CONFIG2, space);//
  us_delay(4);
  
  ads1299_write_reg(CH1SET, 0x60);
  us_delay(4);
  ads1299_read_reg(CH1SET, space);//0000 0001
  us_delay(4);
  
  ads1299_write_reg(CH2SET, 0x81);
  ms_delay(1);
  ads1299_write_reg(CH3SET, 0x81);
  ms_delay(1);
  ads1299_write_reg(CH4SET, 0x81);
  ms_delay(1);
  ads1299_write_reg(CH5SET, 0x81);
  ms_delay(1);
  ads1299_write_reg(CH6SET, 0x81);
  ms_delay(1);
  ads1299_write_reg(CH7SET, 0x81);
  ms_delay(1);
  ads1299_write_reg(CH8SET, 0x81);
  ms_delay(1);
  
  
  
  /*change CHnSET
  ads1299_write_reg(CONFIG2, 0xD0);
  us_delay(4);
  
  ads1299_write_reg(CH1SET, 0x80);
  ms_delay(1);
  ads1299_write_reg(CH2SET, 0x80);
  ms_delay(1);
  ads1299_write_reg(CH3SET, 0x80);
  ms_delay(1);
  ads1299_write_reg(CH4SET, 0x80);
  ms_delay(1);
  ads1299_write_reg(CH5SET, 0x80);
  ms_delay(1);
  ads1299_write_reg(CH6SET, 0x80);
  ms_delay(1);
  ads1299_write_reg(CH7SET, 0x80);
  ms_delay(1);
  ads1299_write_reg(CH8SET, 0x80);
  ms_delay(1);  
  */
 
  

  


  
  
  /*ACTIVATE TEST SIGNAL   
  
  ads1299_write_reg(CONFIG2, 0xD0);
  us_delay(4);
  ads1299_read_reg(CONFIG2, space);
  us_delay(4);
  
  ads1299_write_reg(CH1SET, 0x05);
  us_delay(4);
  
  ads1299_read_reg(CH1SET, space);
  us_delay(4);
  ads1299_read_reg(CH1SET, space);
  us_delay(4);
  
  ads1299_write_reg(CH2SET, 0x05);
  us_delay(4);
  ads1299_write_reg(CH3SET, 0x05);
  us_delay(4);
  ads1299_write_reg(CH4SET, 0x05);
  us_delay(4);
  ads1299_write_reg(CH5SET, 0x05);
  us_delay(4);
  ads1299_write_reg(CH6SET, 0x05);
  us_delay(4);
  ads1299_write_reg(CH7SET, 0x05);
  us_delay(4);
  ads1299_write_reg(CH8SET, 0x05);
  us_delay(4);
 */
    
  /*test on SPI
  uint8 space[2];
  ADS_WRITE_command(_SDATAC);
  while(1)
  {
  ads1299_write_reg(CH1SET, 0x05);
  us_delay(4);  
  ads1299_read_reg(CH1SET, space);
  ms_delay(4);
  }
  */
  
  /*TEST NORMAL INPUT
  ads1299_write_reg(CONFIG2, 0xC0);
  us_delay(4);
  
  ads1299_write_reg(CH1SET, 0x00);
  us_delay(4);
  ads1299_write_reg(CH2SET, 0x00);
  us_delay(4);
  ads1299_write_reg(CH3SET, 0x00);
  us_delay(4);
  ads1299_write_reg(CH4SET, 0x00);
  us_delay(4);
  ads1299_write_reg(CH5SET, 0x00);
  us_delay(4);
  ads1299_write_reg(CH6SET, 0x00);
  us_delay(4);
  ads1299_write_reg(CH7SET, 0x00);
  us_delay(4);
  ads1299_write_reg(CH8SET, 0x00);
  us_delay(4);
  */
  
  
  START_ADS = 1;
  us_delay(4);
  
//  ADS_WRITE_command(_RDATAC);
  us_delay(4);
  
 /* test  
  uint8 space[27];  //be able to read meaningless data at here.
   while(1)
  {
    TI_ADS1293_SPIStreamRead(space, 20);
    ms_delay(4);
  }
 */
 // ADS_WRITE_command(_SDATAC);  //before read register for test
  // us_delay(5);
}


void ads1299_read_data(uint8 *ads_status, uint8 *ads_data)
{
  //waits for DRDY goes low?  BETTER to use it as interrupt
  //start pin also should be high

 
  uint8 readout[27] = {0}; //if use ISR then need to be two buffer
  
  CS_ADS = 0;
  
  //send RDATA command
  spiWriteByte(_RDATA);
  
  //receive data
  
  for(uint8 i=0; i<27; i++)
  {
    spiReadByte((uint8 *)&readout[i], 0xFF);
  }
  
  for(uint8 i=0; i<3; i++)
  {
    *(ads_status+i) = readout[i];
  }
  
  for(uint8 i=3; i<27; i++)
    *(ads_data+i) = readout[i];
    
  CS_ADS = 1;
  
}

void ads1299_stop_dataread(void)
{
  CS_ADS = 0;
  
  spiWriteByte(_SDATAC);
 
  CS_ADS = 1;
}

void ads1299_write_reg(uint8 ADDR, uint8 VAL)
{
  CS_ADS = 0;  //R23
  
  spiWriteByte(_WREG | ADDR);
  us_delay(5);
  spiWriteByte(0x00);
  us_delay(5);
  spiWriteByte(VAL);
  us_delay(5);
  
  CS_ADS = 1;
}

void ads1299_read_reg(uint8 ADR, uint8 *register_data)
{


  CS_ADS = 0;


  spiWriteByte(_RREG|ADR);   //ON R27 
  us_delay(5);
//  spiWriteByte(0xAA);    //test
//  us_delay(5);
  spiWriteByte(0x00);
  us_delay(5);

  spiReadByte(register_data, 0xFF);   //FF ON R27 
  us_delay(5);
  
  CS_ADS = 1;
  
}

void TI_ADS1293_SPIStreamRead(uint8 *buffer, uint8 count)
{
  uint8 i;
//  static uint32 tst_count = 0;
//  static uint8 toggle = 0;
  
  CS_ADS = 0;                                                                       // /CS enable
  
  
  for(i=0; i < count; i++)
  {
    spiReadByte((buffer+i), 0xFF);                                             // Read data
   // us_delay(2);             //give it time to read data          
  }
  CS_ADS = 1;                                                                   // /CS disable
      
}

void ADS_WRITE_command(uint8 command)
{
  
  CS_ADS = 0;                                                             // /CS enable                                               // Send register address
  
  spiWriteByte(command);                                                         // Send data value  

  CS_ADS = 1;                                                            // /CS disable      

}



//set CH1 compare value = 32
//set clock edge = 1 / 32M
//delay_us = 32 * (1/32)
void delay_init(void)
{
  P0SEL |= 0x04;
  P2DIR |= P2DIR_PRIP0_T1_0_1;
  T1STAT = ~T1STAT_CH0IF;
  
  T1CCTL0 = (T1CCTL0 & ~T1CCTLn_CMP) | T1CCTLn_CMP_SET_ON_CMP | T1CCTLn_MODE;
  T1CC0L = 0x20;
  T1CC0H = 0x00;
  T1CTL = (T1CTL & ~(T1CTL_MODE | T1CTL_DIV)) | T1CTL_MODE_FREERUN | T1CTL_DIV_1;
}

void us_delay(int usec)
{
  int a = 0;
  T1CNTL = 0x01;
  a = 0;
  while(a < usec)
  {
    if(P0_2 == 1)
    {  
      T1CNTL = 0x01;
      P0_2 = 0;
      a++;
    }
  }
  a = 0;
}

void ms_delay(int msec)
{
  int b = 0;
  while(b < msec)
  {
    us_delay(1000);
    b++;
  }
}




