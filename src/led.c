/*  Title		: Led
 *  Filename	: led.c
 *	Author		: iacopo sprenger
 *	Date		: 20.01.2021
 *	Version		: 0.2
 *	Description	: rgb led control ported for chibiOS
 */

/**********************
 *	INCLUDES
 **********************/

#include <hal.h>

#include <led.h>

/**********************
 *	CONFIGURATION
 **********************/

#define LED_TIM				PWMD8

#define LED_CHANNEL_RED			0
#define LED_CHANNEL_GREEN		1
#define LED_CHANNEL_BLUE		2

/**********************
 *	CONSTANTS
 **********************/

#define LED_MAX			(0xfff)



/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/



/**********************
 *	VARIABLES
 **********************/



/**********************
 *	PROTOTYPES
 **********************/



/**********************
 *	DECLARATIONS
 **********************/

void led_init(void) {


	static const PWMConfig led_pwm_config = {
		100000,   /* 100Khz PWM clock frequency */
		256,     /* PWM period of 256 ticks */
		NULL,     /* No callback at the end of a period. */
		{
			{PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, NULL}, /* First channel used as an active PWM with no callback. */
			{PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, NULL}, /* Second channel used as an active PWM with no callback. */
			{PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, NULL}, /* Third channel used as an active PWM with no callback. */
			{PWM_OUTPUT_DISABLED, NULL} /* Fourth channel not used */
		},
		0,        /* Zero, as the doc says. */
		0,        /* Zero, as the doc says. */
		0		  /* Zero, as the doc says. */
	};

	pwmStart(&LED_TIM, &led_pwm_config);
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
	pwmEnableChannel(&LED_TIM, LED_CHANNEL_RED, r);
	pwmEnableChannel(&LED_TIM, LED_CHANNEL_GREEN, g);
	pwmEnableChannel(&LED_TIM, LED_CHANNEL_BLUE, b);
}


/* END */


