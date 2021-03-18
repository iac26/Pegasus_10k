/*  Title		: Serial
 *  Filename	: serial.c
 *	Author		: iacopo sprenger
 *	Date		: 31.01.2021
 *	Version		: 0.1
 *	Description	: serial communication driver
 */

/**********************
 *	INCLUDES
 **********************/

#include <serial.h>
#include <util.h>

/**********************
 *	CONSTANTS
 **********************/


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/


/*
 * One thread for serial communication with a semaphore to sync between the ISR and the BH
 */

/**********************
 *	VARIABLES
 **********************/

static SERIAL_INST_t * serial_devices[SERIAL_MAX_INST];
static uint16_t serial_devices_count = 0;


/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/


void serial_init(SERIAL_INST_t * ser, SerialDriver * uart, void * inst, SERIAL_RET_t (*decode_fcn)(void *, uint8_t)) {
	ser->id = serial_devices_count;
	ser->uart = uart;
	ser->inst = inst;
	ser->decode_fcn = decode_fcn;
	util_buffer_u8_init(&ser->bfr, ser->buffer, SERIAL_FIFO_LEN);
	if(serial_devices_count < SERIAL_MAX_INST) {
		sdStart(uart, NULL);
		serial_devices[serial_devices_count] = ser;
	}
	serial_devices_count++;
}

void serial_send(SERIAL_INST_t * ser, uint8_t * data, uint16_t length) {
	sdAsynchronousWrite(ser->uart, data, length);
}

static THD_WORKING_AREA(serial_thread_wa, 1024);
static THD_FUNCTION(serial_thread, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	for(uint16_t i = 0; i < serial_devices_count; i++) {
		chEvtRegisterMaskWithFlags(&serial_devices[i]->uart->event, &serial_devices[i]->event, EVENT_MASK(i),
				CHN_INPUT_AVAILABLE|SD_PARITY_ERROR|SD_FRAMING_ERROR|SD_OVERRUN_ERROR|SD_NOISE_ERROR|SD_BREAK_DETECTED);
	}

	for(;;) {
		eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
		for(uint16_t i = 0; i < serial_devices_count; i++) {
			if(evt & EVENT_MASK(i)) {
				eventflags_t flags = chEvtGetAndClearFlags(&serial_devices[i]->event);
				if(flags & CHN_INPUT_AVAILABLE) {
					uint16_t bytes_read = sdAsynchronousRead(serial_devices[i]->uart, serial_devices[i]->buffer, SERIAL_FIFO_LEN);
					uint16_t j = 0;
					while(bytes_read--) {
						serial_devices[i]->decode_fcn(serial_devices[i]->inst, serial_devices[i]->buffer[j++]);
					}
				}
			}
		}
	}
}

void start_serial(void) {
	chThdCreateStatic(serial_thread_wa, sizeof(serial_thread_wa), NORMALPRIO, serial_thread, NULL);
}

/* END */


