/***
 * Skeleton code for ChibiOS project
 *
 * Copyright 2011 Kalle Vahlman, <zuh@iki.fi>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */
/* ChibiOS includes */
#include <ch.h>
#include <hal.h>

/* Project includes */

#include <led.h>


/* Setup a working area with a 32 byte stack for our LED4 flashing thread */
static THD_WORKING_AREA(led4, 128);
static THD_FUNCTION(ledThread, arg) {
	(void)arg;

    /* Threads are active until they return, we don't want to */
    while (TRUE) {
        /* Toggle LED4 on/off */
        palTogglePad(GPIOB, 7);
        /* With 1s intervals */
        chThdSleepMilliseconds(100);
    }
}

int main(void) {

    /* Initialize ChibiOS HAL and core */
    halInit();
    chSysInit();


    /* Start a thread dedicated to flashing LED4 */
    chThdCreateStatic(led4, sizeof(led4), NORMALPRIO, ledThread, NULL);

    led_init();
    led_set_color(LED_COLOR_BLUE);

    /* This is our main loop */
    while (TRUE) {
        /* Toggle LED3 on/off */
    	palTogglePad(GPIOA, 15);
        /* With 500ms intervals */
        chThdSleepMilliseconds(500);
    }
    return 0;
}
