#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

// Version of the PCB
#define HARDWARE_VERSION_MAJOR 3
#define HARDWARE_VERSION_MINOR 2
#define HARDWARE_VERSION_SUFFIX 0

#define HARDWARE_VERSION_ADDRESS 0x08070000 - 2048
#define MSD_ADDRESS 0x08020000
#define APP_ADDRESS 0x08040000
#define WRITE_TO_FLASH_TIME 1500

#define MSD_BLOCK_SIZE 2048
#ifndef DEBUG
#define MSD_BLOCK_COUNT 192 * 1024 / MSD_BLOCK_SIZE
#else
#define MSD_BLOCK_COUNT 128 * 1024 / MSD_BLOCK_SIZE
#endif

#define SVF_BUFFER_SIZE 2048

#endif
