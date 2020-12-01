#ifndef _COMM_H_
#define _COMM_H_

#include <inttypes.h>

#define RECV_BUFFER_SIZE 1024 * 55
#define SEND_BUFFER_SIZE 1000
#define SEND_TIMEOUT 5000

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
//#define COMMAND_PHI2_INIT 15
//#define COMMAND_PHI2_INIT_DONE 16
#define COMMAND_MIRRORING_REQUEST 17
#define COMMAND_MIRRORING_RESULT 18
#define COMMAND_RESET 19
#define COMMAND_RESET_ACK 20
//#define COMMAND_PRG_EPROM_WRITE_REQUEST 21
//#define COMMAND_CHR_EPROM_WRITE_REQUEST 22
//#define COMMAND_EPROM_PREPARE 23
//#define COMMAND_PRG_FLASH_ERASE_REQUEST 24
//#define COMMAND_PRG_FLASH_WRITE_REQUEST 25
//#define COMMAND_CHR_FLASH_ERASE_REQUEST 26
//#define COMMAND_CHR_FLASH_WRITE_REQUEST 27
//#define COMMAND_JTAG_SETUP 28
//#define COMMAND_JTAG_SHUTDOWN 29
//#define COMMAND_JTAG_EXECUTE 30
//#define COMMAND_JTAG_RESULT 31
//#define COMMAND_TEST_SET 32
//#define COMMAND_TEST_RESULT 33
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

#define COMMAND_BOOTLOADER 0xFE
#define COMMAND_DEBUG 0xFF


void comm_init();
void comm_start(uint8_t command, uint16_t length);
void comm_send_byte(uint8_t data);
void comm_send(uint8_t* address, uint16_t length);
void comm_proceed(uint8_t data);

extern volatile uint8_t comm_recv_command;
extern volatile uint16_t comm_recv_length;
extern volatile uint8_t recv_buffer[RECV_BUFFER_SIZE];
extern volatile uint8_t comm_recv_done;

#endif
