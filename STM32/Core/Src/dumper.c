#include <stdio.h>
#include "main.h"
#include "dumper.h"
#include "comm.h"
#include "crc.h"
#include "led.h"

volatile uint8_t dummy;
uint16_t flash_buffer_mask = 0xFFC0;

static void delay_clock(uint32_t cycles)
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
  HAL_GPIO_WritePin(SHIFTERS_OE_GPIO_Port, SHIFTERS_OE_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(SHIFTERS_OE_GPIO_Port, SHIFTERS_OE_Pin, GPIO_PIN_RESET);
}

void set_flash_buffer_size(uint16_t value)
{
  // Set maximum number of bytes in multi-byte program
  uint8_t bit_value = 0;
  while (value > 1)
  {
    value >>= 1;
    bit_value++;
  }
  flash_buffer_mask = 0xFFFF << bit_value;
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
  while (len > 0)
  {
    uint8_t count = 0;
    uint8_t *d = data;
    uint16_t a = address;
    uint16_t last_address;
    uint8_t last_data;
    uint16_t address_base = a & flash_buffer_mask;
    while ((len > 0) && ((a & flash_buffer_mask) == address_base))
    {
      if (*d != 0xFF)
        count++;
      a++;
      len--;
      d++;
    }

    if (count)
    {
      PRG(0x8000 | 0x0000) = 0xF0;
      PRG(0x8000 | 0x0AAA) = 0xAA;
      PRG(0x8000 | 0x0555) = 0x55;
      PRG(0x8000 | 0x0000) = 0x25;
      PRG(0x8000 | 0x0000) = count - 1;

      while (count > 0)
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
        if (((read_1 ^ read_2) & (1 << 6)) && ((read_2 ^ read_3) & (1 << 6)))
        {
          if (read_1 & (1 << 1))
          {
            comm_start(COMMAND_FLASH_WRITE_ERROR, 3);
            comm_send_byte(read_1);
            comm_send_byte(read_2);
            comm_send_byte(read_3);
            return;
          } else if (read_1 & (1 << 5))
          {
            comm_start(COMMAND_FLASH_WRITE_TIMEOUT, 3);
            comm_send_byte(read_1);
            comm_send_byte(read_2);
            comm_send_byte(read_3);
            return;
          }
        } else
        {
          read_1 = PRG(0x8000 | last_address);
          read_2 = PRG(0x8000 | last_address);
          if (read_1 == read_2 && read_2 == last_data)
            break; // OK
        }
      }
    }

    address = a;
    data = d;
  }
  comm_start(COMMAND_PRG_WRITE_DONE, 0);
}

static uint8_t transfer_fds_byte(uint8_t *output, uint8_t input, uint8_t *end_of_head)
{
  uint32_t start_time;
  start_time = HAL_GetTick();
  while (!IRQ_FIRED)
  {
    // waiting for interrupt
    // timeout 5 secs
    if (HAL_GetTick() - start_time >= 5000)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  }
  if (output)
    *output = PRG(FDS_DATA_READ);
  PRG(FDS_DATA_WRITE) = input; // clear interrupt
  uint8_t status = PRG(FDS_DISK_STATUS);
  if (end_of_head)
    *end_of_head |= (status >> 6) & 1;
  start_time = HAL_GetTick();
  while (IRQ_FIRED)
  {
    // is interrupt flag cleared?
    // timeout 5 secs
    if (HAL_GetTick() - start_time >= 5000)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  }
  return 1;
}

static uint8_t read_fds_block_send(uint16_t length, uint8_t send, uint8_t *crc_ok, uint8_t *end_of_head, uint16_t *file_size, uint32_t gap_delay)
{
  uint8_t data;
  uint8_t status;
  uint32_t b;

  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON; // motor on without transfer
  delay_clock(gap_delay);
  if (send)
  {
    led_green();
    comm_start(COMMAND_FDS_READ_RESULT_BLOCK, length + 2);
  }
  // start transfer, enable IRQ
  PRG(FDS_CONTROL) = PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON;
  for (b = 0; b < length; b++)
  {
    if (!transfer_fds_byte(&data, 0, end_of_head))
      return 0;
    if (file_size)
    {
      if (b == 13)
        *file_size |= data;
      else if (b == 14)
        *file_size |= data << 8;
    }
    if (send)
      comm_send_byte(data);
  }
  if (!transfer_fds_byte((uint8_t*) &dummy, 0, end_of_head))
    return 0;
  PRG(FDS_CONTROL) = PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON | FDS_CONTROL_CRC; // enable CRC control
  if (!transfer_fds_byte((uint8_t*) &dummy, 0, end_of_head))
    return 0;
  status = PRG(FDS_DISK_STATUS);
  *crc_ok &= ((status >> 4) & 1) ^ 1;
  *end_of_head |= (status >> 6) & 1;
  if (send)
  {
    comm_send_byte(*crc_ok); // CRC check result
    comm_send_byte(*end_of_head); // end of head meet?
  }
  led_cyan();
  return 1; // success
}

static uint8_t write_fds_block(uint8_t *data, uint16_t length, uint32_t gap_delay)
{
  uint8_t end_of_head = 0;
  uint32_t start_time;
  led_red();
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON; // motor on without transfer
  dummy = PRG(FDS_DRIVE_STATUS); // check if disk is inserted
  PRG(FDS_CONTROL) = FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON; // enable writing without transfer
  delay_clock(gap_delay);
  PRG(FDS_DATA_WRITE) = 0x00; // write $00
  // start transfer, enable IRQ
  PRG(FDS_CONTROL) = FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON;
  transfer_fds_byte(0, 0x80, &end_of_head);  // write $80
  while (length)
  {
    if (end_of_head)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset, stop
      comm_start(COMMAND_FDS_END_OF_HEAD, 0);
      return 0;
    }
    if (!transfer_fds_byte(0, *data, &end_of_head))
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
    data++;
    length--;
  }
  if (!transfer_fds_byte(0, 0xFF, &end_of_head))
  {
    PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset, stop
    comm_start(COMMAND_FDS_TIMEOUT, 0);
    return 0;
  }
  if (end_of_head)
  {
    PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset, stop
    comm_start(COMMAND_FDS_END_OF_HEAD, 0);
    return 0;
  }
  PRG(FDS_CONTROL) = PRG(FDS_CONTROL) = FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON | FDS_CONTROL_CRC;  // enable CRC control
  delay_clock(FDS_WRITE_CRC_DELAY);
  start_time = HAL_GetTick();
  while (1)
  {
    uint8_t status = PRG(FDS_DRIVE_STATUS);
    if (!(status & 2))
      break; // ready
    // timeout 1 sec
    if (HAL_GetTick() - start_time >= 1000)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  }
  led_cyan();
  return 1;
}

void fds_transfer(uint8_t block_read_start, uint8_t block_read_count, uint8_t block_write_count, uint8_t *block_write_ids, uint16_t *write_lengths,
    uint8_t *write_data)
{
  uint8_t crc_ok = 1;
  uint8_t end_of_head = 0;
  uint8_t current_block = 0;
  uint8_t current_writing_block = 0;
  uint32_t start_time;

  led_magenta();
  PRG(FDS_IRQ_CONTROL) = 0x00; // disable timer IRQ
  PRG(FDS_MASTER_IO) = 0x01; // enable disk registers
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset
  uint8_t ram_adapter_connected = 1;
  PRG(FDS_EXT_WRITE) = 0x00; // Ext. connector
  PRG(0x0000) = 0xFF; // To prevent open bus read
  if ((PRG(FDS_EXT_READ) & 0x7F) != 0x00)
    ram_adapter_connected = 0;
  PRG(FDS_EXT_WRITE) = 0xFF; // Ext. connector
  PRG(0x0000) = 0x00; // To prevent open bus read
  if ((PRG(FDS_EXT_READ) & 0x7F) != 0x7F)
    ram_adapter_connected = 0;
  if (!ram_adapter_connected)
  {
    comm_start(COMMAND_FDS_NOT_CONNECTED, 0);
    return;
  }
  if (PRG(FDS_DRIVE_STATUS) & 1)
  {
    comm_start(COMMAND_FDS_DISK_NOT_INSERTED, 0);
    return;
  }
  delay_clock(916500); // ~916500 cycles
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON; // monor on, unreset
  delay_clock(268500); // ~268500 cycles
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON; // monor on, unreset
  // waiting until drive is rewinded
  start_time = HAL_GetTick();
  do
  {
    // timeout 15 secs
    if (HAL_GetTick() - start_time >= 15000)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return;
    }
  } while (PRG(FDS_DRIVE_STATUS) & 2);

  led_cyan();

  // disk info block
  if (block_write_count && (current_block == block_write_ids[current_writing_block]))
  {
    // gap delay while writing = ~28300 bits = (~28300 / 8)bits * ~165cycles = ~583687.5
    uint16_t write_length = write_lengths[current_writing_block];
    if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BEFORE_FIRST_BLOCK))
      return;
    write_data += write_length;
    current_writing_block++;
    block_write_count--;
  } else
  {
    // gap delay while reading = ~486974 cycles
    if (!read_fds_block_send(56, (current_block >= block_read_start) && block_read_count, &crc_ok, &end_of_head, 0, FDS_READ_GAP_BEFORE_FIRST_BLOCK))
      return;
  }
  if (block_read_count)
    block_read_count--;
  current_block++;

  if (crc_ok && !end_of_head && (block_read_count || block_write_count))
  {
    // file amount block
    if (block_write_count && (current_block == block_write_ids[current_writing_block]))
    {
      uint16_t write_length = write_lengths[current_writing_block];
      if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BETWEEN_BLOCKS))
        return;
      write_data += write_length;
      current_writing_block++;
      block_write_count--;
    } else
    {
      if (!read_fds_block_send(2, (current_block >= block_read_start) && block_read_count, &crc_ok, &end_of_head, 0, FDS_READ_GAP_BETWEEN_BLOCKS))
        return;
    }
    if (block_read_count)
      block_read_count--;
    current_block++;
  }

  while (crc_ok && !end_of_head && (block_read_count || block_write_count))
  {
    // file header block
    uint16_t file_size = 0; // size of the next file
    if (block_write_count && (current_block == block_write_ids[current_writing_block]))
    {
      uint16_t write_length = write_lengths[current_writing_block];
      if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BETWEEN_BLOCKS))
        return;
      write_data += write_length;
      current_writing_block++;
      block_write_count--;
    } else
    {
      if (!read_fds_block_send(16, (current_block >= block_read_start) && block_read_count, &crc_ok, &end_of_head, &file_size, FDS_READ_GAP_BETWEEN_BLOCKS))
        return;
    }
    if (block_read_count)
      block_read_count--;
    current_block++;

    if (crc_ok && !end_of_head && (block_read_count || block_write_count))
    {
      // file data block
      if (block_write_count && (current_block == block_write_ids[current_writing_block]))
      {
        uint16_t write_length = write_lengths[current_writing_block];
        if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BETWEEN_BLOCKS))
          return;
        write_data += write_length;
        current_writing_block++;
        block_write_count--;
      } else
      {
        if (!read_fds_block_send(file_size + 1, (current_block >= block_read_start) && block_read_count, &crc_ok, &end_of_head, 0, FDS_READ_GAP_BETWEEN_BLOCKS))
          return;
      }
      if (block_read_count)
        block_read_count--;
      current_block++;
    }
  }

  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_RESET; // reset, stop

  HAL_Delay(50);
  if (current_writing_block && !block_write_count && !block_read_count)
  {
    comm_start(COMMAND_FDS_WRITE_DONE, 0);
    return;
  }
  comm_start(COMMAND_FDS_READ_RESULT_END, 0);
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
