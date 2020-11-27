#include "main.h"
#include "dumper.h"
#include "comm.h"
#include "crc.h"
#include "led.h"

volatile uint8_t dummy;
volatile uint16_t page_mask = 0xFFC0;

static uint8_t read_prg_once(uint16_t address)
{
  while (TIM2->CNT > 10)
  {
  }
  while (TIM2->CNT < 10)
  {
  }
  uint8_t result = PRG(address);
  dummy = PRG(0);
  return result;
}

void reset(void)
{
  HAL_GPIO_WritePin(SHIFTERS_OE_GPIO_Port, SHIFTERS_OE_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(SHIFTERS_OE_GPIO_Port, SHIFTERS_OE_Pin, GPIO_PIN_RESET);
}

void read_prg_send(uint16_t address, uint16_t length)
{
  led_green();
  comm_start(COMMAND_PRG_READ_RESULT, length);
  if (length == 1)
    comm_send_byte(read_prg_once(address));
  else
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
    if (HAL_GetTick() >= start_time + 3000) // 3 seconds timeout
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
    uint16_t address_base = a & page_mask;
    while ((len > 0) && ((a & page_mask) == address_base))
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
        if (HAL_GetTick() >= start_time + 1000) // 1 second timeout
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

void read_fds_send(uint8_t start_block, uint8_t block_count)
{
  uint8_t status;
  uint8_t end_of_head = 0;
  uint8_t current_block = 0;
  uint16_t b;

  led_magenta();
  PRG(0x4022) = 0x00; // disable IRQ
  PRG(0x4023) = 0x00; // disable registers
  PRG(0x4023) = 0x01; // enable disk registers
  PRG(0x4025) = 0x2E; // reset
  // waiting for disk
  while (1)
  {
    status = PRG(0x4032);
    if (!(status & 1))
      break; // disk inserted
  }
  PRG(0x4025) = 0x2E; // reset
  HAL_Delay(800); // 916522 cycles
  PRG(0x4025) = 0x2F; // start motor
  PRG(0x4025) = 0x2D; // unreset

  HAL_Delay(250); // 268531 cycles
  PRG(0x4025) = 0x2E; // reset
  PRG(0x4025) = 0x2F; // start motor
  PRG(0x4025) = 0x2D; // unreset

  // waiting until drive is rewinded
  while (1)
  {
    status = PRG(0x4032);
    if (!(status & 2))
      break; // ready
  }

  HAL_Delay(FDS_PAUSE_BEFORE_FIRST_BLOCK); // 486974 cycles

  led_green();
  if (start_block == 0)
    comm_start(COMMAND_FDS_READ_RESULT_BLOCK, 58);
  PRG(0x4025) = 0x6D; // start transfer
  PRG(0x4025) = 0xED; // enable IRQ
  for (b = 0; b < 56; b++)
  {
    while (!IRQ_FIRED)
    {
      // waiting for interrupt
    }
    uint8_t data = PRG(0x4031);
    //PRG(0x4024) = 0xFF; // clear interrupt
    // status read also clears interrupt
    status = PRG(0x4030);
    end_of_head |= (status >> 6) & 1;
    if (start_block == 0)
      comm_send_byte(data);
    while (IRQ_FIRED)
    {
      // is interrupt flag cleared?
    }
  }
  PRG(0x4025) = 0xED; // enable CRC control
  while (!IRQ_FIRED)
    ; // waiting for interrupt
  PRG(0x4031);
  PRG(0x4024) = 0xFF; // clear interrupt
  while (IRQ_FIRED)
    ; // is interrupt flag cleared?
  status = PRG(0x4030);
  end_of_head |= (status >> 6) & 1;
  if (start_block == 0)
  {
    comm_send_byte(((status >> 4) & 1) ^ 1); // CRC check result
    comm_send_byte(end_of_head); // end of head meet?
  }
  current_block++;

  // reading file amount block
  if (!end_of_head && ((start_block + block_count > current_block) || (block_count = 0)))
  {
    PRG(0x4025) = 0x2D; // motor on without transfer
    // waiting until drive is ready
    while (1)
    {
      status = PRG(0x4032);
      if (!(status & 2))
        break; // ready
    }
    HAL_Delay(FDS_PAUSE_BETWEEN_BLOCKS); // 9026 cycles
    if ((current_block >= start_block) && ((current_block < start_block + block_count) || (block_count == 0)))
      comm_start(COMMAND_FDS_READ_RESULT_BLOCK, 4);
    PRG(0x4025) = 0x6D; // start transfer
    PRG(0x4025) = 0xED; // enable IRQ
    for (b = 0; b < 2; b++)
    {
      while (!IRQ_FIRED)
        ; // waiting for interrupt
      uint8_t data = PRG(0x4031);
      //PRG(0x4024) = 0xFF; // clear interrupt
      // status read also clears interrupt
      status = PRG(0x4030);
      end_of_head |= (status >> 6) & 1;
      if ((current_block >= start_block) && ((current_block < start_block + block_count) || (block_count == 0)))
        comm_send_byte(data);
      while (IRQ_FIRED)
      {
        // is interrupt flag cleared?
      }
    }
    PRG(0x4025) = 0xED; // enable CRC control
    while (!IRQ_FIRED)
    {
      // waiting for interrupt
    }
    PRG(0x4031);
    PRG(0x4024) = 0xFF; // clear interrupt
    while (IRQ_FIRED)
      ; // is interrupt flag cleared?
    status = PRG(0x4030);
    end_of_head |= (status >> 6) & 1;
    if ((current_block >= start_block) && ((current_block < start_block + block_count) || (block_count == 0)))
    {
      comm_send_byte(((status >> 4) & 1) ^ 1); // CRC check result
      comm_send_byte(end_of_head); // end of head meet?
    }
    current_block++;
  }

  while (!end_of_head && ((start_block + block_count > current_block) || (block_count = 0)))
  {
    // reading file header block
    uint16_t file_size = 0; // size of the next file

    PRG(0x4025) = 0x2D; // motor on without transfer
    // waiting until drive is ready
    while (1)
    {
      status = PRG(0x4032);
      if (!(status & 2))
        break; // ready
    }
    HAL_Delay(FDS_PAUSE_BETWEEN_BLOCKS); // 9026 cycles
    if ((current_block >= start_block) && ((current_block < start_block + block_count) || (block_count == 0)))
      comm_start(COMMAND_FDS_READ_RESULT_BLOCK, 18);
    PRG(0x4025) = 0x6D; // start transfer
    PRG(0x4025) = 0xED; // enable IRQ
    for (b = 0; b < 16; b++)
    {
      while (!IRQ_FIRED)
        ; // waiting for interrupt
      uint8_t data = PRG(0x4031);
      //PRG(0x4024) = 0xFF; // clear interrupt
      // status read also clears interrupt
      status = PRG(0x4030);
      end_of_head |= (status >> 6) & 1;
      if ((current_block >= start_block) && ((current_block < start_block + block_count) || (block_count == 0)))
        comm_send_byte(data);
      if (b == 13)
        file_size |= data;
      else if (b == 14)
        file_size |= data << 8;
      while (IRQ_FIRED)
        ; // is interrupt flag cleared?
    }
    PRG(0x4025) = 0xED; // enable CRC control
    while (!IRQ_FIRED)
    {
      // waiting for interrupt
    }
    PRG(0x4031);
    PRG(0x4024) = 0xFF; // clear interrupt
    while (IRQ_FIRED)
    {
      // is interrupt flag cleared?
    }
    status = PRG(0x4030);
    end_of_head |= (status >> 6) & 1;
    if ((current_block >= start_block) && ((current_block < start_block + block_count) || (block_count == 0)))
    {
      comm_send_byte(((status >> 4) & 1) ^ 1); // CRC check result
      comm_send_byte(end_of_head); // end of head meet?
    }
    current_block++;

    // reading file data
    if (!end_of_head && ((start_block + block_count > current_block) || (block_count = 0)))
    {
      PRG(0x4025) = 0x2D; // motor on without transfer
      // waiting until drive is ready
      while (1)
      {
        status = PRG(0x4032);
        if (!(status & 2))
          break; // ready
      }
      HAL_Delay(FDS_PAUSE_BETWEEN_BLOCKS); // 9026 cycles
      if ((current_block >= start_block) && ((current_block < start_block + block_count) || (block_count == 0)))
        comm_start(COMMAND_FDS_READ_RESULT_BLOCK, file_size + 3);
      PRG(0x4025) = 0x6D; // start transfer
      PRG(0x4025) = 0xED; // enable IRQ
      for (b = 0; b < file_size + 1; b++)
      {
        while (!IRQ_FIRED)
          ; // waiting for interrupt
        uint8_t data = PRG(0x4031);
        //PRG(0x4024) = 0xFF; // clear interrupt
        // status read also clears interrupt
        status = PRG(0x4030);
        end_of_head |= (status >> 6) & 1;
        if ((current_block >= start_block) && ((current_block < start_block + block_count) || (block_count == 0)))
          comm_send_byte(data);
        while (IRQ_FIRED)
          ; // is interrupt flag cleared?
      }
      PRG(0x4025) = 0xED; // enable CRC control
      while (!IRQ_FIRED)
      {
        // waiting for interrupt
      }
      PRG(0x4031);
      PRG(0x4024) = 0xFF; // clear interrupt
      while (IRQ_FIRED)
      {
        // is interrupt flag cleared?
      }
      status = PRG(0x4030);
      end_of_head |= (status >> 6) & 1;
      if ((current_block >= start_block) && ((current_block < start_block + block_count) || (block_count == 0)))
      {
        comm_send_byte(((status >> 4) & 1) ^ 1); // CRC check result
        comm_send_byte(end_of_head); // end of head meet?
      }
      current_block++;
    }
  }

  PRG(0x4025) = 0x26; // reset, stop

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
