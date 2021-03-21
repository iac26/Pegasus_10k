/*  Title       : Flash
 *  Filename    : flash.h
 *  Author      : iacopo sprenger
 *  Date        : 21.03.2021
 *  Version     : 0.2
 *  Description : NOR FLASH driver
 */

#ifndef FLASH_H
#define FLASH_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

/**********************
 *  CONSTANTS
 **********************/

#define SECTOR_SIZE 1 << 16
#define PAGE_SIZE 256
#define READ_MAX_BUFFER 2048

// State commands
#define READ_STATUS_REGISTER 0x05
#define READ_FLAG_STATUS_REGISTER 0x70


// Write latch commands
#define WRITE_ENABLE_LATCH 0x06
#define WRITE_DISABLE_LATCH 0x04


// Read commands
#define READ_SINGLE 0x03
#define FREAD_SINGLE 0x0B
#define FREAD_DUAL 0xBB


// Write commands
#define WRITE_SINGLE 0x02
#define FWRITE_DUAL 0xA2
#define FWRITE_DUAL_EXT 0xD2


// Erase commands
#define ERASE_SUBSECTOR 0x20 // 4KB subsector
#define ERASE_SECTOR 0xD8
#define ERASE_ALL 0x60


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/



/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif



#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* FLASH_H */

/* END */
