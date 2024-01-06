#ifndef INC_CPLDWRITER_H_
#define INC_CPLDWRITER_H_

#include "fatfs.h"

struct udata_s {
  FIL *fil;
  uint8_t buff[SVF_BUFFER_SIZE];
  unsigned int pos;
  unsigned int size;
  int freq;
};

void write_cpld(FILINFO *svf_file);
int find_svf_file(FILINFO *svf_file);

#endif /* INC_CPLDWRITER_H_ */
