#ifndef _DUMPER_H_
#define _DUMPER_H_

#define PROTOCOL_VERSION 3

#define FDS_PAUSE_BEFORE_FIRST_BLOCK 300
#define FDS_PAUSE_BETWEEN_BLOCKS 5

#define PRG(address) (*(volatile uint8_t*) ((address) + 0x60000000))
#define CHR(address) (*(volatile uint8_t*) ((address) + 0x64000000))
#define IRQ_FIRED (!HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin))

void reset(void);
void read_prg_send(uint16_t address, uint16_t len);
void read_prg_crc_send(uint16_t address, uint16_t len);
void write_prg(uint16_t address, uint16_t len, uint8_t *data);
void read_chr_send(uint16_t address, uint16_t len);
void read_chr_crc_send(uint16_t address, uint16_t len);
void write_chr(uint16_t address, uint16_t len, uint8_t *data);
void erase_flash_sector();
void write_flash(uint16_t address, uint16_t len, uint8_t *data);
void fds_transfer(uint8_t block_read_start, uint8_t block_read_count, uint8_t block_write_count, uint8_t* block_write_ids, uint16_t *write_lengths,
    uint8_t *write_data);
void get_mirroring();

#endif
