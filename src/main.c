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

    /* Init RGB LED */
    led_init();
    led_set_color(LED_COLOR_TEAL);

    /* Start threads */


    sensor_start(SENSOR_PRIO);

    control_start(CONTROL_PRIO);

    debug_start();

    serial_start(SERIAL_PRIO);


    while (TRUE) {
        chThdSleepMilliseconds(500);
    }
    return 0;
}
