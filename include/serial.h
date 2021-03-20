/*  Title       : Serial
 *  Filename    : serial.h
 *  Author      : iacopo sprenger
 *  Date        : 31.01.2021
 *  Version     : 0.1
 *  Description : serial communication driver
 */

#ifndef SERIAL_H
#define SERIAL_H



/**********************
 *  INCLUDES
 **********************/

#include <hal.h>
#include <ch.h>

#include <stdint.h>
#include <util.h>

/**********************
 *  CONSTANTS
 **********************/

#define SERIAL_MAX_INST	(16)
#define SERIAL_FIFO_LEN	(1024)




/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

typedef enum SERIAL_RET {
	SERIAL_DONE,
	SERIAL_PROGRESS,
	SERIAL_ERROR
}SERIAL_RET_t;

typedef struct SERIAL_INST {
	uint32_t id;
	UARTDriver * uart;
	event_listener_t event;
	void * inst;
	SERIAL_RET_t (*decode_fcn)(void *, uint8_t);
	UTIL_BUFFER_U8_t bfr;
	uint8_t buffer[SERIAL_FIFO_LEN];
	uint8_t dma_buffer;
}SERIAL_INST_t;

/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif


void serial_init(SERIAL_INST_t * ser, UARTDriver * uart, void * inst, SERIAL_RET_t (*decode_fcn)(void *, uint8_t));

void serial_global_init(void);

void serial_send(SERIAL_INST_t * ser, uint8_t * data, uint16_t length);


/*
 * START SERIAL THREAD
 * Careful, all the serial devices must be registered before starting the thread!!!
 */
void serial_start(tprio_t prio);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* SERIAL_H */

/* END */
