#ifndef INC_FLASH_H_
#define INC_FLASH_H_

void set_flash_buffer_size(uint16_t value);
void erase_flash_sector();
void write_flash(uint16_t address, uint16_t len, uint8_t *data);
void set_coolboy_gpio_mode(uint8_t coolboy_gpio_mode);

#endif /* INC_FLASH_H_ */
