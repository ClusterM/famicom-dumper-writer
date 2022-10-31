#include "bootloader.h"
#include "main.h"
#include "cpldwriter.h"
#include "libxsvf.h"
#include "fatfs.h"
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

static int svf_setup(struct libxsvf_host *h)
{
  HAL_GPIO_WritePin(TCK_GPIO_Port, TCK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TDI_GPIO_Port, TDI_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TMS_GPIO_Port, TMS_Pin, GPIO_PIN_SET);

  return 0;
}

static int svf_shutdown(struct libxsvf_host *h)
{
  return 0;
}

static void svf_udelay(struct libxsvf_host *h, long usecs, int tms, long num_tck)
{
  if (usecs > 50000) usecs = 50000; // limit
  TIM1->CNT = 0;
  if (num_tck)
  {
    HAL_GPIO_WritePin(TMS_GPIO_Port, TMS_Pin, tms ? GPIO_PIN_SET : GPIO_PIN_RESET);
    while (num_tck-- > 0)
    {
      HAL_GPIO_WritePin(TCK_GPIO_Port, TCK_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TCK_GPIO_Port, TCK_Pin, GPIO_PIN_SET);
    }
    while (TIM1->CNT < usecs) ;
  }
}

static int svf_getbyte(struct libxsvf_host *h)
{
  FRESULT fr;
  struct udata_s *u = (struct udata_s *)h->user_data;

  if (u->pos >= u->size)
  {
    // need to read data
    fr = f_read(u->fil, &u->buff, SVF_BUFFER_SIZE, &u->size);
    if (fr) return -1;
    if (!u->size) return -1;
    u->pos = 0;
  }
  return u->buff[u->pos++];
}

static int svf_pulse_tck(struct libxsvf_host *h, int tms, int tdi, int tdo, int rmask, int sync)
{
  HAL_GPIO_WritePin(TMS_GPIO_Port, TMS_Pin, tms ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (tdi >= 0) HAL_GPIO_WritePin(TDI_GPIO_Port, TDI_Pin, tdi ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TCK_GPIO_Port, TCK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TCK_GPIO_Port, TCK_Pin, GPIO_PIN_SET);
  GPIO_PinState line_tdo = HAL_GPIO_ReadPin(TDO_GPIO_Port, TDO_Pin);
  return tdo < 0 || line_tdo == tdo ? line_tdo : -1;
}

int svf_set_frequency(struct libxsvf_host *h, int v)
{
  struct udata_s *u = (struct udata_s *)h->user_data;
  u->freq = v;
  return 0;
}

static void svf_report_error(struct libxsvf_host *h, const char *file, int line, const char *message)
{
  error();
}

static void *svf_realloc(struct libxsvf_host *h, void *ptr, int size, enum libxsvf_mem which)
{
  return realloc(ptr, size);
}

void write_cpld(FILINFO *svf_file)
{
  FATFS FatFs;
  FIL fil;
  FRESULT fr;
  struct udata_s u;
  u.fil = &fil;
  u.pos = 0;
  u.size = 0;
  struct libxsvf_host h = {
    .setup = svf_setup,
    .shutdown = svf_shutdown,
    .udelay = svf_udelay,
    .getbyte = svf_getbyte,
    .sync = 0,
    .pulse_tck = svf_pulse_tck,
    .pulse_sck = 0,
    .set_trst = 0,
    .set_frequency = svf_set_frequency,
    .report_tapstate = 0,
    .report_device = 0,
    .report_status = 0,
    .report_error = svf_report_error,
    .realloc = svf_realloc,
    .user_data = &u
  };

 // Mount FAT file system
  fr = f_mount(&FatFs, "", 1);
  if (fr)
    error();
  // Open firmware file
  fr = f_open(u.fil, svf_file->fname, FA_READ);
  if (fr)
    error();

  if (libxsvf_play(&h, LIBXSVF_MODE_SVF) < 0) {
    error();
  }

  // Close file
  fr = f_close(u.fil);
  if (fr)
    error();
  // Delete file
  fr = f_unlink(svf_file->fname);
  // Unmount
  fr = f_mount(NULL, "", 1);
  if (fr)
    error();
}

int find_svf_file(FILINFO *svf_file)
{
  // Find first .svf file if any
  int len, r = 0;
  FATFS fs;
  FRESULT res;
  DIR dir;
  FILINFO fno;

  f_mount(&fs, "", 1);
  res = f_opendir(&dir, "/");
  if (res == FR_OK)
  {
    while (1)
    {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0)
        break;
      if (!(fno.fattrib & AM_DIR) && (fno.fsize != 0))
      {
        len = strlen(fno.fname);
        if ((len > 4) && (fno.fname[len - 4] == '.') && (fno.fname[len - 3] == 'S') && (fno.fname[len - 2] == 'V') && (fno.fname[len - 1] == 'F'))
        {
          memcpy(svf_file, &fno, sizeof(FILINFO));
          r = 1;
          break;
        }
      }
    }
  }
  f_closedir(&dir);
  f_mount(NULL, "", 1);

  return r;
}
