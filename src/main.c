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



int main(void) {

    /* Initialize ChibiOS HAL and core */
    halInit();
    chSysInit();

    /* Init RGB LED */
    led_init();
    led_set_color(LED_COLOR_TEAL);


    while (TRUE) {
        /* Toggle LED3 on/off */
    	palTogglePad(GPIOA, 15);
        /* With 500ms intervals */
        chThdSleepMilliseconds(50);
    }
    return 0;
}
