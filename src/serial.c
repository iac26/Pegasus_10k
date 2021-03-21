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

#include <hal.h>
#include <ch.h>

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

static binary_semaphore_t serial_rx_sem;

static thread_reference_t serial_rx = NULL;




/**********************
 *	PROTOTYPES
 **********************/

void rxchar(UARTDriver *uartp, uint16_t c);

/**********************
 *	DECLARATIONS
 **********************/

const UARTConfig uart_conf = {
		NULL,
		NULL,
		NULL,
		rxchar,
		NULL,
		NULL,
		115200,
		0U,
		0U,
		0U
};

/*
 * TODO: use hardware FIFO
 */

void rxchar(UARTDriver *uartp, uint16_t c) {
	for(uint16_t i = 0; i < serial_devices_count; i++) {
		if(serial_devices[i]->uart == uartp) {
			util_buffer_u8_add(&serial_devices[i]->bfr, c);
			chSysLockFromISR();
			chThdResumeI(&serial_rx, MSG_OK);
			chSysUnlockFromISR();
			break;
		}
	}
}

void serial_global_init(void) {
	chBSemObjectInit(&serial_rx_sem, TRUE);
}


void serial_init(SERIAL_INST_t * ser, UARTDriver * uart, void * inst, SERIAL_RET_t (*decode_fcn)(void *, uint8_t)) {
	ser->id = serial_devices_count;
	ser->uart = uart;
	ser->inst = inst;
	ser->decode_fcn = decode_fcn;
	util_buffer_u8_init(&ser->bfr, ser->buffer, SERIAL_FIFO_LEN);
	if(serial_devices_count < SERIAL_MAX_INST) {
		serial_devices[serial_devices_count] = ser;
	}
	serial_devices_count++;
}

void serial_send(SERIAL_INST_t * ser, uint8_t * data, uint16_t length) {
	uartStartSend(ser->uart, length, data);
}

static THD_WORKING_AREA(serial_thread_wa, 1024);
static THD_FUNCTION(serial_thread, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	serial_global_init();

	for(uint8_t i = 0; i < serial_devices_count; i++) {
		uartStart(serial_devices[i]->uart, &uart_conf);
	}

	for(;;) {
		chSysLock();
		msg_t msg = chThdSuspendS(&serial_rx);
		chSysUnlock();
		if( msg == MSG_OK ) {
			for(uint16_t i = 0; i < serial_devices_count; i++) {
				while(!util_buffer_u8_isempty(&serial_devices[i]->bfr)) {
					serial_devices[i]->decode_fcn(serial_devices[i]->inst, util_buffer_u8_get(&serial_devices[i]->bfr));
				}
			}
		}
	}
}

void serial_start(tprio_t prio) {
	chThdCreateStatic(serial_thread_wa, sizeof(serial_thread_wa), prio, serial_thread, NULL);
}

/* END */


