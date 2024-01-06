#include "bootloader.h"
#include "main.h"
#include "firmwriter.h"
#include "fatfs.h"
#include <inttypes.h>
#include <string.h>

void write_hardware_version(void)
{
  uint32_t value;
  uint32_t current_value;
  HAL_StatusTypeDef hres;
  FLASH_EraseInitTypeDef erase_init_struct;
  uint32_t sector_error = 0;
  uint8_t HARDWARE_VERSION[] = { HARDWARE_VERSION_MAJOR & 0xFF, (HARDWARE_VERSION_MAJOR >> 8) && 0xFF, HARDWARE_VERSION_MINOR, HARDWARE_VERSION_SUFFIX };

  // Write hardware version
  value = *(uint32_t*)HARDWARE_VERSION;
  current_value = *(volatile uint32_t*)(HARDWARE_VERSION_ADDRESS);
  if (value != current_value)
  {
    // Flash can be locked by FATFS
    hres = HAL_FLASH_Unlock();
    if (hres != HAL_OK)
      error();
    erase_init_struct.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init_struct.PageAddress = HARDWARE_VERSION_ADDRESS;
    erase_init_struct.NbPages = 1;
    hres = HAL_FLASHEx_Erase(&erase_init_struct, &sector_error);
    if (hres != HAL_OK)
      error();
    current_value = *(volatile uint32_t*)(HARDWARE_VERSION_ADDRESS);
    hres = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, HARDWARE_VERSION_ADDRESS, value);
    if (hres != HAL_OK)
      error();
    current_value = *(volatile uint32_t*)(HARDWARE_VERSION_ADDRESS);
    if (value != current_value)
      error();
    HAL_FLASH_Lock();
  }
}

void write_firmware(FILINFO *bin_file)
{
  int i;
  uint8_t buff[MSD_BLOCK_SIZE];
  FATFS FatFs;
  FIL fil;
  FRESULT fr;
  UINT br = 0;
  uint32_t address = APP_ADDRESS;
  FLASH_EraseInitTypeDef erase_init_struct;
  uint32_t sector_error = 0;
  HAL_StatusTypeDef hres;
  uint32_t value;

  // Unlock flash
  hres = HAL_FLASH_Unlock();
  if (hres != HAL_OK)
    error();

  // Mount FAT file system
  fr = f_mount(&FatFs, "", 1);
  if (fr)
    error();
  // Open firmware file
  fr = f_open(&fil, bin_file->fname, FA_READ);
  if (fr)
    error();
  while (1)
  {
    // Read data
    fr = f_read(&fil, buff, MSD_BLOCK_SIZE, &br);
    if (fr)
      error();
    // Break if end of file
    if (!br)
      break;
    if ((address % MSD_BLOCK_SIZE) != 0)
      error();
    // Erase flash page
    erase_init_struct.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init_struct.PageAddress = address;
    erase_init_struct.NbPages = 1;
    hres = HAL_FLASHEx_Erase(&erase_init_struct, &sector_error);
    if (hres != HAL_OK)
      error();
    // Write data
    for (i = 0; i < br; i += 4)
    {
      value = *(volatile uint32_t*) &buff[i];
      hres = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, value);
      if (hres != HAL_OK)
        error();
      address += 4;
    }
  }
  // Close file
  fr = f_close(&fil);
  if (fr)
    error();
  // Delete file
  fr = f_unlink(bin_file->fname);
  // Unmount
  fr = f_mount(NULL, "", 1);
  if (fr)
    error();

  // Lock flash
  HAL_FLASH_Lock();
}

int find_bin_file(FILINFO *bin_file)
{
  // Find first .bin file if any
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
        if ((len > 4) && (fno.fname[len - 4] == '.') && (fno.fname[len - 3] == 'B') && (fno.fname[len - 2] == 'I') && (fno.fname[len - 1] == 'N'))
        {
          memcpy(bin_file, &fno, sizeof(FILINFO));
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

