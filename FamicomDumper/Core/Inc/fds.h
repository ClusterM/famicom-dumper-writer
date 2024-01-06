#ifndef INC_FDS_H_
#define INC_FDS_H_

void fds_transfer(uint8_t block_read_start, uint8_t block_read_count, uint8_t block_write_count, uint8_t *block_write_ids, uint16_t *write_lengths, uint8_t *write_data);

#endif /* INC_FDS_H_ */
