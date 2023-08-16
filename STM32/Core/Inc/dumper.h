#ifndef _DUMPER_H_
#define _DUMPER_H_

#define PROTOCOL_VERSION 5
#define FIRMWARE_VERSION_MAJOR 3
#define FIRMWARE_VERSION_MINOR 4
#define FIRMWARE_VERSION_SUFFIX 0
#define HARDWARE_VERSION_ADDRESS 0x08070000 - 2048
#define HARDWARE_VERSION ((volatile uint8_t*)(HARDWARE_VERSION_ADDRESS))

#define FDS_IRQ_CONTROL 0x4022
#define FDS_MASTER_IO 0x4023
#define FDS_DATA_WRITE 0x4024
#define FDS_CONTROL 0x4025
#define FDS_EXT_WRITE 0x4026
#define FDS_DISK_STATUS 0x4030
#define FDS_DATA_READ 0x4031
#define FDS_DRIVE_STATUS 0x4032
#define FDS_EXT_READ 0x4033

#define FDS_CONTROL_MOTOR_ON        0b00100001
#define FDS_CONTROL_MOTOR_OFF       0b00100010
#define FDS_CONTROL_READ            0b00100100
#define FDS_CONTROL_WRITE           0b00100000
#define FDS_CONTROL_CRC             0b00110000
#define FDS_CONTROL_TRANSFER_ON     0b01100000
#define FDS_CONTROL_IRQ_ON          0b10100000

#define FDS_DRIVE_STATUS_DISK_NOT_INSERTED 0b00000001
#define FDS_DRIVE_STATUS_DISK_NOT_READY 0b00000010
#define FDS_DRIVE_STATUS_DISK_WRITE_PROTECTED 0b00000100

#define FDS_READ_GAP_BEFORE_FIRST_BLOCK 486974
#define FDS_WRITE_GAP_BEFORE_FIRST_BLOCK 580000
#define FDS_READ_GAP_BETWEEN_BLOCKS 9026
#define FDS_WRITE_GAP_BETWEEN_BLOCKS 17917
#define FDS_WRITE_CRC_DELAY 897

#define FDS_COPY_PROTECTION_RESET_INTERVAL 0x4000

#define PRG(address) (*(volatile uint8_t*) ((address) + 0x60000000))
#define CHR(address) (*(volatile uint8_t*) ((address) + 0x64000000))
#define IRQ_FIRED (!HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin))

void delay_clock(uint32_t cycles);
void reset(void);
void read_prg_send(uint16_t address, uint16_t len);
void read_prg_crc_send(uint16_t address, uint16_t len);
void write_prg(uint16_t address, uint16_t len, uint8_t *data);
void read_chr_send(uint16_t address, uint16_t len);
void read_chr_crc_send(uint16_t address, uint16_t len);
void write_chr(uint16_t address, uint16_t len, uint8_t *data);
void get_mirroring();

#endif
