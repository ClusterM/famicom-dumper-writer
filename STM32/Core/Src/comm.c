#include "comm.h"
#include "crc.h"
#include "usbd_cdc_if.h"

static uint8_t comm_send_crc;
static unsigned int comm_send_length;
static unsigned int comm_send_pos;

static int comm_recv_pos;
static uint8_t comm_recv_crc;
static uint8_t comm_recv_error;

volatile uint8_t comm_recv_command;
volatile unsigned int comm_recv_length;
volatile uint8_t recv_buffer[RECV_BUFFER + 16];
volatile uint8_t send_buffer[SEND_BUFFER + 16];
volatile uint16_t comm_out_pos = 0;
volatile uint8_t comm_recv_done;

extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
extern volatile uint8_t dma_done;

static void comm_flush(void)
{
  uint8_t res;
  do
  {
    res = CDC_Transmit_FS((uint8_t*) send_buffer, comm_out_pos);
  } while (res != USBD_OK);
}

static void comm_send_and_calc(uint8_t data)
{
  comm_send_crc = calc_crc8(comm_send_crc, data);
  send_buffer[comm_out_pos++] = data;
}

void comm_start(uint8_t command, uint16_t length)
{
  comm_send_crc = 0;
  comm_out_pos = 0;
  comm_send_pos = 0;
  comm_send_and_calc('F');
  comm_send_and_calc(command);
  comm_send_and_calc(length & 0xff);
  comm_send_and_calc((length >> 8) & 0xff);
  comm_send_length = length;

  if (!comm_send_length)
  {
    send_buffer[comm_out_pos++] = comm_send_crc;
    comm_flush();
  }
}

void comm_send_byte(uint8_t data)
{
  comm_send_and_calc(data);
  comm_send_pos++;

  if (comm_send_pos == comm_send_length)
  {
    send_buffer[comm_out_pos++] = comm_send_crc;
    comm_flush();
  }
}

void comm_send(uint8_t *address, uint16_t length)
{
  dma_done = 0;
  HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel1, (uint32_t) address, (uint32_t) &send_buffer[comm_out_pos], length);
  while (!dma_done)
  {
  }
  comm_send_pos += length;
  while (length)
  {
    comm_send_crc = calc_crc8(comm_send_crc, send_buffer[comm_out_pos++]);
    length--;
  }
  if (comm_send_pos >= comm_send_length)
  {
    send_buffer[comm_out_pos++] = comm_send_crc;
    comm_flush();
  }
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
  comm_recv_crc= calc_crc8(comm_recv_crc, data);
  unsigned int l = comm_recv_pos - 4;
  switch (comm_recv_pos)
  {
  case 0:
    if (data != 'F')
    {
      comm_recv_error = 1;
      comm_start(COMMAND_ERROR_INVALID, 0);
    }
    break;
  case 1:
    comm_recv_command = data;
    break;
  case 2:
    comm_recv_length = data;
    break;
  case 3:
    comm_recv_length |= (uint16_t) data << 8;
    break;
  default:
    if (l >= sizeof(recv_buffer))
    {
      comm_recv_pos = 0;
      comm_recv_error = 1;
      comm_start(COMMAND_ERROR_OVERFLOW, 0);
      return;
    } else if (l < comm_recv_length)
    {
      recv_buffer[l] = data;
    } else if (l == comm_recv_length)
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
    break;
  }
  comm_recv_pos++;
}

void comm_init()
{
  comm_recv_pos = 0;
  comm_recv_done = 0;
  comm_recv_error = 0;
}
