#include "main.h"
#include "dumper.h"
#include "comm.h"
#include "led.h"

static void unrom512_cmd_write(uint32_t address, uint8_t data)
{
  PRG(0xC000) = address >> 14;
  PRG(0x8000 | (address & 0x3FFF)) = data;
}

static uint8_t unrom512_cmd_read(uint32_t address)
{
  PRG(0xC000) = address >> 14;
  return PRG(0x8000 | (address & 0x3FFF));
}

void erase_unrom512()
{
  led_yellow();
  unrom512_cmd_write(0x0000, 0xF0);
  unrom512_cmd_write(0x5555, 0xAA);
  unrom512_cmd_write(0x2AAA, 0x55);
  unrom512_cmd_write(0x5555, 0x80);
  unrom512_cmd_write(0x5555, 0xAA);
  unrom512_cmd_write(0x2AAA, 0x55);
  unrom512_cmd_write(0x5555, 0x10);

  uint32_t start_time = HAL_GetTick();
  // waiting for result
  uint8_t ff_count = 0;
  while (1)
  {
    if (HAL_GetTick() >= start_time + 5000) // 5 seconds timeout
    {
      // timeout
      comm_start(COMMAND_FLASH_ERASE_TIMEOUT, 0);
      break;
    }
    if (PRG(0x8000) != 0xFF)
      ff_count = 0;
    else
      ff_count++;
    if (ff_count >= 2)
    {
      // OK
      comm_start(COMMAND_PRG_WRITE_DONE, 0);
      break;
    }
  }
}

void write_unrom512(uint32_t address, uint16_t len, uint8_t *data)
{
  led_red();
  PRG(0x8000 | 0x0000) = 0xF0;
  while (len > 0)
  {
    unrom512_cmd_write(0x5555, 0xAA);
    unrom512_cmd_write(0x2AAA, 0x55);
    unrom512_cmd_write(0x5555, 0xA0);
    unrom512_cmd_write(address, *data);

    uint32_t start_time = HAL_GetTick();
    // waiting for result
    while (1)
    {
      if (HAL_GetTick() >= start_time + 50) // 50 ms timeout
      {
        // timeout
        comm_start(COMMAND_FLASH_WRITE_TIMEOUT, 0);
        return;
      }
      uint8_t read_1 = unrom512_cmd_read(address);
      uint8_t read_2 = unrom512_cmd_read(address);
      if ((read_1 == read_2) && (read_2 == *data)) break; // OK
    }
    address++;
    data++;
    len--;
  }
  comm_start(COMMAND_PRG_WRITE_DONE, 0);
}
