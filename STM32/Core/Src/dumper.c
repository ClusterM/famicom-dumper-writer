#include <stdio.h>
#include "main.h"
#include "dumper.h"
#include "comm.h"
#include "crc.h"
#include "led.h"

static volatile uint8_t dummy;

void delay_clock(uint32_t cycles)
{
  if (cycles < 30000)
  {
    // Accurate timer
    TIM4->CNT = 0;
    while (TIM4->CNT < cycles)
    {
    }
  } else
  {
    // Not so accurate timer for large delays
    cycles /= 1000;
    TIM3->CNT = 0;
    while (TIM3->CNT < cycles)
    {
    }
  }
}

void reset(void)
{
  led_white();
  HAL_GPIO_WritePin(SHIFTERS_OE_GPIO_Port, SHIFTERS_OE_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(SHIFTERS_OE_GPIO_Port, SHIFTERS_OE_Pin, GPIO_PIN_RESET);
}

void read_prg_send(uint16_t address, uint16_t length)
{
  led_green();
  comm_start(COMMAND_PRG_READ_RESULT, length);
  comm_send((uint8_t*) &PRG(address), length);
}

void read_prg_crc_send(uint16_t address, uint16_t length)
{
  led_green();
  uint16_t crc = 0;
  while (length > 0)
  {
    crc = calc_crc16(crc, PRG(address));
    length--;
    address++;
  }
  comm_start(COMMAND_PRG_READ_RESULT, 2);
  comm_send_byte(crc & 0xFF);
  comm_send_byte((crc >> 8) & 0xFF);
}

void write_prg(uint16_t address, uint16_t len, uint8_t *data)
{
  led_red();
  while (len > 0)
  {
    PRG(address) = *data;
    address++;
    len--;
    data++;
  }
}

void read_chr_send(uint16_t address, uint16_t length)
{
  led_green();
  comm_start(COMMAND_CHR_READ_RESULT, length);
  comm_send((uint8_t*) &CHR(address), length);
}

void read_chr_crc_send(uint16_t address, uint16_t length)
{
  led_green();
  uint16_t crc = 0;
  while (length > 0)
  {
    crc = calc_crc16(crc, CHR(address));
    length--;
    address++;
  }
  comm_start(COMMAND_CHR_READ_RESULT, 2);
  comm_send_byte(crc & 0xFF);
  comm_send_byte((crc >> 8) & 0xFF);
}

void write_chr(uint16_t address, uint16_t length, uint8_t *data)
{
  led_red();
  while (length > 0)
  {
    CHR(address) = *data;
    address++;
    length--;
    data++;
  }
}


void get_mirroring()
{
  led_green();
  comm_start(COMMAND_MIRRORING_RESULT, 4);
  dummy = CHR(0);
  comm_send_byte(HAL_GPIO_ReadPin(CIRAM_A10_GPIO_Port, CIRAM_A10_Pin));
  dummy = CHR(1 << 10);
  comm_send_byte(HAL_GPIO_ReadPin(CIRAM_A10_GPIO_Port, CIRAM_A10_Pin));
  dummy = CHR(1 << 11);
  comm_send_byte(HAL_GPIO_ReadPin(CIRAM_A10_GPIO_Port, CIRAM_A10_Pin));
  dummy = CHR((1 << 10) | (1 << 11));
  comm_send_byte(HAL_GPIO_ReadPin(CIRAM_A10_GPIO_Port, CIRAM_A10_Pin));
}
