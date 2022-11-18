#include "main.h"
#include "dumper.h"
#include "comm.h"
#include "led.h"

static volatile uint8_t dummy;

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
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  }
  if (output)
    *output = PRG(FDS_DATA_READ);
  else
    dummy = PRG(FDS_DATA_READ);
  PRG(FDS_DATA_WRITE) = input; // clear interrupt
  uint8_t disk_status = PRG(FDS_DISK_STATUS);
  if (end_of_head)
    *end_of_head |= (disk_status >> 6) & 1;
  start_time = HAL_GetTick();
  while (IRQ_FIRED)
  {
    // is interrupt flag cleared?
    // timeout 5 secs
    if (HAL_GetTick() - start_time >= 5000)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  }
  return 1;
}

static uint8_t read_fds_block_send(uint16_t length, uint8_t send, uint16_t *file_size, uint32_t gap_delay)
{
  uint8_t data;
  uint8_t disk_status;
  uint32_t b;
  uint8_t crc_ok = 1;
  uint8_t end_of_head = 0;

  delay_clock(gap_delay);
  if (send)
  {
    led_green();
    comm_start(COMMAND_FDS_READ_RESULT_BLOCK, length + 2);
  }
  // start transfer, enable IRQ
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON;
  for (b = 0; b < length; b++)
  {
    if (!transfer_fds_byte(&data, 0, &end_of_head))
      return 0;
    // parse file size if need
    if (file_size)
    {
      if (b == 13)
        *file_size = data;
      else if (b == 14)
        *file_size |= data << 8;
    }
    if (send)
    {
      if (!comm_send_byte(data))
        return 0;
    }
  }
  if (!transfer_fds_byte(0, 0, &end_of_head))
    return 0;
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON | FDS_CONTROL_CRC; // enable CRC control
  if (!transfer_fds_byte(0, 0, &end_of_head))
    return 0;
  disk_status = PRG(FDS_DISK_STATUS);
  crc_ok &= ((disk_status >> 4) & 1) ^ 1;
  end_of_head |= (disk_status >> 6) & 1;
  if (send)
  {
    if (!comm_send_byte(crc_ok)) // CRC check result
      return 0;
    if (!comm_send_byte(end_of_head)) // end of head meet?
      return 0;
  }
  if (!crc_ok || end_of_head)
  {
    // invalid data or end of disk, abort transfer
    PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
    HAL_Delay(50);
    if (send)
      comm_start(COMMAND_FDS_READ_RESULT_END, 0);
    else {
      if (!crc_ok)
        comm_start(COMMAND_FDS_BLOCK_CRC_ERROR, 0);
      else
        comm_start(COMMAND_FDS_END_OF_HEAD, 0);
    }
    return 0;
  }
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON; // motor on without transfer

  led_cyan();
  return 1; // success
}

static uint8_t write_fds_block(uint8_t *data, uint16_t length, uint32_t gap_delay)
{
  uint8_t end_of_head = 0;
  uint32_t start_time;
  uint16_t pos = 0;
  led_red();
  PRG(FDS_CONTROL) = FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON; // enable writing without transfer
  delay_clock(gap_delay);
  PRG(FDS_DATA_WRITE) = 0x00; // write $00
  // start transfer, enable IRQ
  PRG(FDS_CONTROL) = FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON;
  transfer_fds_byte(0, 0x80, &end_of_head); // write $80
  while (length)
  {
    if (end_of_head)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
      comm_start(COMMAND_FDS_END_OF_HEAD, 0);
      return 0;
    }
    if (!transfer_fds_byte(0, *data, &end_of_head))
      return 0;
    data++;
    length--;
    pos++;
    // avoid copy protection, lol
    if ((pos % FDS_COPY_PROTECTION_RESET_INTERVAL) == 0)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON;
      PRG(FDS_CONTROL) = FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON;
    }
  }
  if (!transfer_fds_byte(0, 0xFF, &end_of_head))
    return 0;
  if (end_of_head)
  {
    PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
    comm_start(COMMAND_FDS_END_OF_HEAD, 0);
    return 0;
  }
  PRG(FDS_CONTROL) = FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON | FDS_CONTROL_CRC; // enable CRC control
  delay_clock(FDS_WRITE_CRC_DELAY);
  start_time = HAL_GetTick();
  while (1)
  {
    uint8_t drive_status = PRG(FDS_DRIVE_STATUS);
    if (!(drive_status & FDS_DRIVE_STATUS_DISK_NOT_READY))
      break; // ready
    // timeout 1 sec
    if (HAL_GetTick() - start_time >= 1000)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  }
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON; // motor on without transfer

  led_cyan();
  return 1; // success
}

static uint8_t fds_transfer_real(uint8_t block_read_start, uint8_t block_read_count, uint8_t block_write_count, uint8_t *block_write_ids, uint16_t *write_lengths, uint8_t *write_data)
{
  uint8_t current_block = 0;
  uint32_t start_time;
  uint8_t drive_status;
  uint8_t blocks_writed = 0;

  led_magenta();
  PRG(FDS_IRQ_CONTROL) = 0x00; // disable timer IRQ
  PRG(FDS_MASTER_IO) = 0x01; // enable disk registers
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset
  drive_status = PRG(FDS_DRIVE_STATUS);
  // is disk inserted?
  if (drive_status & FDS_DRIVE_STATUS_DISK_NOT_INSERTED)
  {
    comm_start(COMMAND_FDS_DISK_NOT_INSERTED, 0);
    return 0;
  }
  // is it write protected while we are writing?
  if (block_write_count && (drive_status & FDS_DRIVE_STATUS_DISK_WRITE_PROTECTED))
  {
    comm_start(COMMAND_FDS_DISK_WRITE_PROTECTED, 0);
    return 0;
  }
  // battery test
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON; // monor on, unreset
  PRG(FDS_EXT_WRITE) = 0xFF;
  HAL_Delay(100);
  if (!(PRG(FDS_EXT_READ) & 0x80))
  {
    // battery low
    PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
    comm_start(COMMAND_FDS_BATTERY_LOW, 0);
    return 0;
  }
  // waiting until drive is rewinded
  start_time = HAL_GetTick();
  do
  {
    // timeout 15 secs
    if (HAL_GetTick() - start_time >= 15000)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  } while (!(PRG(FDS_DRIVE_STATUS) & FDS_DRIVE_STATUS_DISK_NOT_READY));
  start_time = HAL_GetTick();
  do
  {
    // timeout 15 secs
    if (HAL_GetTick() - start_time >= 15000)
    {
      PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  } while (PRG(FDS_DRIVE_STATUS) & FDS_DRIVE_STATUS_DISK_NOT_READY);

  led_cyan();

  // disk info block
  if (block_write_count && (current_block == block_write_ids[blocks_writed]))
  {
    // gap delay while writing = ~28300 bits = (~28300 / 8)bits * ~165cycles = ~583687.5
    uint16_t write_length = write_lengths[blocks_writed];
    if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BEFORE_FIRST_BLOCK))
      return 0;
    write_data += write_length;
    blocks_writed++;
    block_write_count--;
  } else
  {
    // gap delay while reading = ~486974 cycles
    if (!read_fds_block_send(56, (current_block >= block_read_start) && block_read_count, 0, FDS_READ_GAP_BEFORE_FIRST_BLOCK))
      return 0;
  }
  if ((current_block >= block_read_start) && block_read_count)
    block_read_count--;
  current_block++;

  if (block_read_count || block_write_count)
  {
    // file amount block
    if (block_write_count && (current_block == block_write_ids[blocks_writed]))
    {
      uint16_t write_length = write_lengths[blocks_writed];
      if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BETWEEN_BLOCKS))
        return 0;
      write_data += write_length;
      blocks_writed++;
      block_write_count--;
    } else
    {
      if (!read_fds_block_send(2, (current_block >= block_read_start) && block_read_count, 0, FDS_READ_GAP_BETWEEN_BLOCKS))
        return 0;
    }
    if ((current_block >= block_read_start) && block_read_count)
      block_read_count--;
    current_block++;
  }

  while (block_read_count || block_write_count)
  {
    // file header block
    uint16_t file_size = 0; // size of the next file
    if (block_write_count && (current_block == block_write_ids[blocks_writed]))
    {
      uint16_t write_length = write_lengths[blocks_writed];
      if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BETWEEN_BLOCKS))
        return 0;
      write_data += write_length;
      blocks_writed++;
      block_write_count--;
    } else
    {
      if (!read_fds_block_send(16, (current_block >= block_read_start) && block_read_count, &file_size, FDS_READ_GAP_BETWEEN_BLOCKS))
        return 0;
    }
    if ((current_block >= block_read_start) && block_read_count)
      block_read_count--;
    current_block++;

    if (block_read_count || block_write_count)
    {
      // file data block
      if (block_write_count && (current_block == block_write_ids[blocks_writed]))
      {
        uint16_t write_length = write_lengths[blocks_writed];
        if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BETWEEN_BLOCKS))
          return 0;
        write_data += write_length;
        blocks_writed++;
        block_write_count--;
      } else
      {
        if (!read_fds_block_send(file_size + 1, (current_block >= block_read_start) && block_read_count, 0, FDS_READ_GAP_BETWEEN_BLOCKS))
          return 0;
      }
      if ((current_block >= block_read_start) && block_read_count)
        block_read_count--;
      current_block++;
    }
  }

  return 1;
}

void fds_transfer(uint8_t block_read_start, uint8_t block_read_count, uint8_t block_write_count, uint8_t *block_write_ids, uint16_t *write_lengths, uint8_t *write_data)
{
  uint8_t ok = fds_transfer_real(block_read_start, block_read_count, block_write_count, block_write_ids, write_lengths, write_data);
  PRG(FDS_CONTROL) = FDS_CONTROL_READ | FDS_CONTROL_MOTOR_OFF; // reset, stop
  if (ok)
  {
    HAL_Delay(50);
    if (block_write_count && !block_read_count)
      comm_start(COMMAND_FDS_WRITE_DONE, 0);
    else
      comm_start(COMMAND_FDS_READ_RESULT_END, 0);
  }
}
