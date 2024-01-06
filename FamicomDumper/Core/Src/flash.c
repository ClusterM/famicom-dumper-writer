#include "main.h"
#include "dumper.h"
#include "comm.h"
#include "led.h"

static uint16_t flash_buffer_mask = 0xFFC0;

void set_flash_buffer_size(uint16_t value)
{
  // Set maximum number of bytes in multi-byte flash program
  // 0 = disable multi-byte program
  uint8_t bit_value = 0;
  while (value > 1)
  {
    value >>= 1;
    bit_value++;
  }
  flash_buffer_mask = 0xFFFF << bit_value;
}

void erase_flash_sector()
{
  led_yellow();
  PRG(0x8000 | 0x0000) = 0xF0;
  PRG(0x8000 | 0x0AAA) = 0xAA;
  PRG(0x8000 | 0x0555) = 0x55;
  PRG(0x8000 | 0x0AAA) = 0x80;
  PRG(0x8000 | 0x0AAA) = 0xAA;
  PRG(0x8000 | 0x0555) = 0x55;
  PRG(0x8000 | 0x0000) = 0x30;

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

void write_flash(uint16_t address, uint16_t len, uint8_t *data)
{
  led_red();
  PRG(0x8000 | 0x0000) = 0xF0; // reset flash
  while (len > 0)
  {
    uint16_t count = 0;
    uint8_t *d = data;
    uint16_t a = address;
    uint16_t last_address = 0;
    uint8_t last_data = 0;
    if (flash_buffer_mask != 0xFFFF)
    {
      // multi-byte program
      uint16_t address_base = a & flash_buffer_mask;
      while ((len > 0) && ((a & flash_buffer_mask) == address_base))
      {
        if (*d != 0xFF)
          count++;
        a++;
        len--;
        d++;
      }
    } else {
      // single-byte program
      if (*d != 0xFF)
        count = 1;
      a++;
      len--;
      d++;
    }

    if (count)
    {
      if (count > 1)
      {
        // multi-byte
        PRG(0x8000 | 0x0AAA) = 0xAA;
        PRG(0x8000 | 0x0555) = 0x55;
        PRG(0x8000 | 0x0000) = 0x25;
        PRG(0x8000 | 0x0000) = count - 1;
        while (count)
        {
          if (*data != 0xFF)
          {
            PRG(0x8000 | address) = *data;
            last_address = address;
            last_data = *data;
            count--;
          }
          address++;
          data++;
        }
        PRG(0x8000 + 0x0000) = 0x29;
      } else {
        // single-byte
        while (count)
        {
          if (*data != 0xFF)
          {
            PRG(0x8000 | 0x0AAA) = 0xAA;
            PRG(0x8000 | 0x0555) = 0x55;
            PRG(0x8000 | 0x0AAA) = 0xA0;
            PRG(0x8000 | address) = *data;
            last_address = address;
            last_data = *data;
            count--;
          }
          address++;
          data++;
        }
      }

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
        uint8_t read_1 = PRG(0x8000 | last_address);
        uint8_t read_2 = PRG(0x8000 | last_address);
        uint8_t read_3 = PRG(0x8000 | last_address);
        if ((read_1 == read_2) && (read_2 == read_3) && (read_3 == last_data)) break; // OK
      }
    }

    address = a;
    data = d;
  }
  comm_start(COMMAND_PRG_WRITE_DONE, 0);
}

void set_coolboy_gpio_mode(uint8_t coolboy_gpio_mode)
{
  if (coolboy_gpio_mode)
    HAL_GPIO_WritePin(COOLBOY_MODE_GPIO_Port, COOLBOY_MODE_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(COOLBOY_MODE_GPIO_Port, COOLBOY_MODE_Pin, GPIO_PIN_RESET);
}
