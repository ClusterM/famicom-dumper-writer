#include "comm.h"
#include "crc.h"
#include "usbd_cdc_if.h"

uint8_t send_buffer[SEND_BUFFER_SIZE];
volatile uint16_t send_buffer_pos = 0;
static uint8_t comm_send_crc;     // CRC of outgoing packet with header
static uint16_t comm_send_length; // size of outgoing data
static uint16_t comm_send_pos;    // how many data sent by app

volatile uint8_t recv_buffer[RECV_BUFFER_SIZE];
static uint16_t comm_recv_pos; // how many bytes of packet received
static uint8_t comm_recv_crc;
static uint8_t comm_recv_error;
volatile uint8_t comm_recv_command;
volatile uint16_t comm_recv_length;
volatile uint8_t comm_recv_done;

extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
extern volatile uint8_t dma_done;

static uint8_t comm_flush(void)
{
  uint32_t start_time = HAL_GetTick();
  uint8_t res;
  do
  {
    if (HAL_GetTick() >= start_time + SEND_TIMEOUT) // timeout
    {
      comm_init();
      return 0;
    }
    res = CDC_Transmit_FS((uint8_t*) send_buffer, send_buffer_pos);
  } while (res != USBD_OK);
  send_buffer_pos = 0;
  return 1;
}

static void comm_send_and_calc(uint8_t data)
{
  comm_send_crc = calc_crc8(comm_send_crc, data);
  send_buffer[send_buffer_pos++] = data;
}

static uint8_t check_send_buffer()
{
  uint8_t ok = 1;
  if (comm_send_pos >= comm_send_length)
  {
    send_buffer[send_buffer_pos++] = comm_send_crc;
    ok &= comm_flush();
  } else if (send_buffer_pos >= sizeof(send_buffer))
  {
    ok &= comm_flush();
  }
  return ok;
}

uint8_t comm_start(uint8_t command, uint16_t length)
{
  comm_send_crc = 0;
  send_buffer_pos = 0;
  comm_send_pos = 0;
  comm_send_and_calc('F');
  comm_send_and_calc(command);
  comm_send_and_calc(length & 0xff);
  comm_send_and_calc((length >> 8) & 0xff);
  comm_send_length = length;
  return check_send_buffer();
}

uint8_t comm_send_byte(uint8_t data)
{
  comm_send_and_calc(data);
  comm_send_pos++;
  return check_send_buffer();
}

uint8_t comm_send(uint8_t *address, uint16_t length)
{
  while (length)
  {
    if (!comm_send_byte(*address))
      return 0;
    address++;
    length--;
  }
  return 1;
}

void comm_proceed(uint8_t data)
{
  if (comm_recv_error && data != 'F')
    return;
  comm_recv_error = 0;
  if (!comm_recv_pos)
  {
    comm_recv_crc = 0;
    comm_recv_done = 0;
  }
  comm_recv_crc = calc_crc8(comm_recv_crc, data);
  switch (comm_recv_pos)
  {
  case 0:
    {
      if (data != 'F')
      {
        comm_recv_error = 1;
        comm_start(COMMAND_ERROR_INVALID, 0);
      }
    }
    break;
  case 1:
    {
      comm_recv_command = data;
    }
    break;
  case 2:
    {
      comm_recv_length = data;
    }
    break;
  case 3:
    {
      comm_recv_length |= (uint16_t) data << 8;
    }
    break;
  default:
    {
      uint16_t pos = comm_recv_pos - 4;
      if (pos >= sizeof(recv_buffer))
      {
        comm_recv_pos = 0;
        comm_recv_error = 1;
        comm_start(COMMAND_ERROR_OVERFLOW, 0);
        return;
      } else if (pos < comm_recv_length)
      {
        recv_buffer[pos] = data;
      } else if (pos == comm_recv_length)
      {
        if (!comm_recv_crc)
        {
          comm_recv_done = 1;
        } else
        {
          comm_recv_error = 1;
          comm_start(COMMAND_ERROR_CRC, 0);
        }
        comm_recv_pos = 0;
        return;
      }
    }
    break;
  }
  comm_recv_pos++;
}

void comm_init()
{
  send_buffer_pos = 0;
  comm_send_crc = 0;
  comm_send_length = 0;
  comm_send_pos = 0;
  comm_recv_pos = 0;
  comm_recv_crc = 0;
  comm_recv_error = 0;
  comm_recv_command = 0;
  comm_recv_length = 0;
  comm_recv_done = 0;
}
