#ifndef _COMM_H_
#define _COMM_H_

#include <inttypes.h>

#define RECV_BUFFER_SIZE 1024 * 50
#define SEND_BUFFER_SIZE 64
#define SEND_TIMEOUT 1000

#define COMMAND_PRG_STARTED 0
#define COMMAND_CHR_STARTED 1
#define COMMAND_ERROR_INVALID 2
#define COMMAND_ERROR_CRC 3
#define COMMAND_ERROR_OVERFLOW 4
#define COMMAND_PRG_INIT 5
#define COMMAND_CHR_INIT 6
#define COMMAND_PRG_READ_REQUEST 7
#define COMMAND_PRG_READ_RESULT 8
#define COMMAND_PRG_WRITE_REQUEST 9
#define COMMAND_PRG_WRITE_DONE 10
#define COMMAND_CHR_READ_REQUEST 11
#define COMMAND_CHR_READ_RESULT 12
#define COMMAND_CHR_WRITE_REQUEST 13
#define COMMAND_CHR_WRITE_DONE 14
#define COMMAND_MIRRORING_REQUEST 17
#define COMMAND_MIRRORING_RESULT 18
#define COMMAND_RESET 19
#define COMMAND_RESET_ACK 20
#define COMMAND_COOLBOY_READ_REQUEST 34
#define COMMAND_COOLBOY_ERASE_SECTOR_REQUEST 35
#define COMMAND_COOLBOY_WRITE_REQUEST 36
#define COMMAND_FLASH_ERASE_SECTOR_REQUEST 37
#define COMMAND_FLASH_WRITE_REQUEST 38
#define COMMAND_PRG_CRC_READ_REQUEST 39
#define COMMAND_CHR_CRC_READ_REQUEST 40
#define COMMAND_FLASH_WRITE_ERROR 41
#define COMMAND_FLASH_WRITE_TIMEOUT 42
#define COMMAND_FLASH_ERASE_ERROR 43
#define COMMAND_FLASH_ERASE_TIMEOUT 44
#define COMMAND_FDS_READ_REQUEST 45
#define COMMAND_FDS_READ_RESULT_BLOCK 46
#define COMMAND_FDS_READ_RESULT_END 47
#define COMMAND_FDS_TIMEOUT 48
#define COMMAND_FDS_NOT_CONNECTED 49
#define COMMAND_FDS_BATTERY_LOW 50
#define COMMAND_FDS_DISK_NOT_INSERTED 51
#define COMMAND_FDS_END_OF_HEAD 52
#define COMMAND_FDS_WRITE_REQUEST 53
#define COMMAND_FDS_WRITE_DONE 54
#define COMMAND_SET_FLASH_BUFFER_SIZE 55
#define COMMAND_SET_VALUE_DONE 56
#define COMMAND_FDS_DISK_WRITE_PROTECTED 57
#define COMMAND_FDS_BLOCK_CRC_ERROR 58
#define COMMAND_SET_COOLBOY_GPIO_MODE 59
#define COMMAND_UNROM512_ERASE_REQUEST 60
#define COMMAND_UNROM512_WRITE_REQUEST 61

#define COMMAND_DEBUG 0xFF

void comm_init();
uint8_t comm_start(uint8_t command, uint16_t length);
uint8_t comm_send_byte(uint8_t data);
uint8_t comm_send(uint8_t* address, uint16_t length);
void comm_proceed(uint8_t data);

extern volatile uint8_t comm_recv_command;
extern volatile uint16_t comm_recv_length;
extern volatile uint8_t recv_buffer[RECV_BUFFER_SIZE];
extern volatile uint8_t comm_recv_done;

#endif
