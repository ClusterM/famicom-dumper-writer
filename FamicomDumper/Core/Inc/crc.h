#ifndef _CRC_H_
#define _CRC_H_

extern const uint16_t crc8_table[];
extern const uint16_t crc16_table[];

uint8_t calc_crc8(uint8_t old_crc, uint8_t inbyte);
uint16_t calc_crc16(uint16_t old_crc, uint8_t inbyte);

#endif
