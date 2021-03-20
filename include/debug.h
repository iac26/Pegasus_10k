/*  Title       : Debug
 *  Filename    : debug.h
 *  Author      : iacopo sprenger
 *  Date        : 20.01.2021
 *  Version     : 0.1
 *  Description : debug interface code
 */

#ifndef DEBUG_H
#define DEBUG_H

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <msv2.h>
#include <serial.h>

/**********************
 *  CONSTANTS
 **********************/


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

typedef struct DEBUG_INST{
	uint32_t id;
	MSV2_INST_t msv2;
	SERIAL_INST_t ser;
}DEBUG_INST_t;


/**********************
 *  VARIABLES
 **********************/



/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif


SERIAL_RET_t debug_decode_fcn(void * inst, uint8_t data);

void debug_init(DEBUG_INST_t * debug);

void debug_start(void);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* DEBUG_H */

/* END */
