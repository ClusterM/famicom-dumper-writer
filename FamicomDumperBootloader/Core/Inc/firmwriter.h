#ifndef INC_FIRMWRITER_H_
#define INC_FIRMWRITER_H_

#include "fatfs.h"

void write_hardware_version(void);
void write_firmware(FILINFO *bin_file);
int find_bin_file(FILINFO *bin_file);

#endif /* INC_FIRMWRITER_H_ */
