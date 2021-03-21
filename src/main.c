/*  Title		: PEGASUS 10K Software
 *  Filename	: main.c
 *	Author		: iacopo sprenger
 *	Date		: 19.03.2021
 *	Version		: 0.1
 *	Description	: Pegasus 10k software running on ChibiOS
 */

/**********************
 *	INCLUDES
 **********************/

#include <ch.h>
#include <hal.h>


#include <led.h>
#include <sensor.h>
#include <serial.h>
#include <control.h>
#include <debug.h>

/**********************
 *	CONSTANTS
 **********************/

#define CONTROL_PRIO	NORMALPRIO+3
#define SERIAL_PRIO		NORMALPRIO+2
#define SENSOR_PRIO		NORMALPRIO+1

/**********************
 *	MAIN
 **********************/

int main(void) {

    /* Initialize ChibiOS HAL and core */
    halInit();
    chSysInit();

    /* Init everything*/
    led_init();

    sensor_start(SENSOR_PRIO);

    control_start(CONTROL_PRIO);

    debug_start();

    serial_start(SERIAL_PRIO);

    thread_reference_t main_thread_ref;

    while (TRUE) {
    	chSysLock();
		chThdSuspendS(&main_thread_ref);
		chSysUnlock();
    }
    return 0;
}
