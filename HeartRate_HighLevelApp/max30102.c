/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#include "max30102.h"

#define true 1
#define false 0

//extern i2c_handle_t  i2c_handle;
static int (*_i2c_read)( uint8_t addr, uint16_t count, uint8_t* ptr );
static void (*_i2c_write)( uint8_t addr, uint16_t count, uint8_t* ptr );
static int (*_intPin)( void );

void maxim_max30102_i2c_setup( int (*rd)( uint8_t addr, uint16_t count, uint8_t* ptr ), void (*wr)( uint8_t addr, uint16_t count, uint8_t* ptr ), int (*iPin)(void) )
{
    _i2c_read = rd;
    _i2c_write= wr;
    _intPin = iPin;
}

int maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
   _i2c_write(uch_addr, 1, &uch_data);
   return true;
}

int maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
  _i2c_read(uch_addr, 1, puch_data);

  return true;
}

int maxim_max30102_init()
{
  if(!maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0)) // INTR setting
    return false;
  if(!maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00))
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00))  //FIFO_WR_PTR[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_OVF_COUNTER,0x00))  //OVF_COUNTER[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00))  //FIFO_RD_PTR[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_CONFIG,0x0f))  //sample avg = 1, fifo rollover=false, fifo almost full = 17
    return false;
  if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x03))   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    return false;
  if(!maxim_max30102_write_reg(REG_SPO2_CONFIG,0x27))  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    return false;
  
  if(!maxim_max30102_write_reg(REG_LED1_PA,0x24))   //Choose value for ~ 7mA for LED1
    return false;
  if(!maxim_max30102_write_reg(REG_LED2_PA,0x24))   // Choose value for ~ 7mA for LED2
    return false;
  if(!maxim_max30102_write_reg(REG_PILOT_PA,0x7f))   // Choose value for ~ 25mA for Pilot LED
    return false;
  return true;  
}

int maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
  uint32_t un_temp;
  unsigned char uch_temp;
  *pun_red_led=0;
  *pun_ir_led=0;
  char ach_i2c_data[6];
  
  //read and clear status register
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
  maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
  
  uint8_t addr = REG_FIFO_DATA;
  _i2c_read(addr, 6, (uint8_t*)ach_i2c_data);

  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
  
  
  return true;
}

int maxim_max30102_reset()
{
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x40))
        return false;
    else
        return true;    
}

int maxim_max30102_data_available()
{
    uint8_t reg;

    maxim_max30102_read_reg(REG_INTR_STATUS_1, &reg);
    return (reg & 0b10000000);
}

int maxim_max30102_finger_detected()
{
    uint8_t reg;


    if (!maxim_max30102_write_reg(REG_PROX_INT_THRESH, 0xff))
        return 0;
    if (!maxim_max30102_write_reg(REG_INTR_ENABLE_1, 0b00010000)) // INTR setting
        return 0;

    maxim_max30102_read_reg(REG_INTR_STATUS_1, &reg);

    if (!maxim_max30102_write_reg(REG_INTR_ENABLE_1, 0b11000000)) // INTR setting
        return 0;
    return (reg & 0b00010000);
}

uint8_t maxim_max30102_get_part_id()
{
    uint8_t tread;
    maxim_max30102_read_reg(0xff, &tread);
    return tread;
}

uint8_t maxim_max30102_get_revision()
{
    uint8_t tread;
    maxim_max30102_read_reg(0xfe, &tread);
    return tread;
}

void maxim_max30102_shut_down(int  yes)
{
    uint8_t temp;
    maxim_max30102_read_reg(REG_MODE_CONFIG, &temp);
    if (yes)
        temp |= (1 << 7);
    else
        temp &= ~(1 << 7);
    maxim_max30102_write_reg(REG_MODE_CONFIG,temp);
}

int maxim_max30102_data_avail()
{
    uint8_t wr, rd;
    maxim_max30102_read_reg(REG_FIFO_WR_PTR, &wr);
    maxim_max30102_read_reg(REG_FIFO_RD_PTR, &rd);
    return wr-rd;
}

int max30102_data_available()
{
    uint8_t reg2=0;

    if( !_intPin() ){ //active low
        maxim_max30102_read_reg(REG_INTR_STATUS_1, &reg2);
        }

    return (reg2 & 0b01000000);
}

int max30102_finger_detected()
{
    uint8_t reg1, reg2=0;

    maxim_max30102_read_reg(REG_INTR_ENABLE_1, &reg1);               //get current interrupt enable status
    maxim_max30102_write_reg(REG_PROX_INT_THRESH, 0xff);             //set proximity threshold
    maxim_max30102_write_reg(REG_INTR_ENABLE_1, (reg1|0b00010000));  //enable int for proximity

    if( !_intPin() ){ //active low
        maxim_max30102_read_reg(REG_INTR_STATUS_2, &reg2);
        maxim_max30102_read_reg(REG_INTR_STATUS_1, &reg2);
        }

    maxim_max30102_write_reg(REG_INTR_ENABLE_1, reg1);               //restore interrupts
    return (reg2 & 0b00010000);
}
