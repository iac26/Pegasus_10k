/*  Title		: Control
 *  Filename	: control.c
 *	Author		: iacopo sprenger
 *	Date		: 20.01.2021
 *	Version		: 0.1
 *	Description	: main program control
 */

/**********************
 *	INCLUDES
 **********************/

#include <hal.h>
#include <ch.h>

#include <control.h>
#include <epos4.h>
#include <led.h>
#include <sensor.h>

/**********************
 *	CONFIGURATION
 **********************/

#define CONTROL_HEART_BEAT	50 /* ms */

#define VENTING_PIN		7

#define VENTING_PORT	GPIOB

#define THRUST_CONTROL_ENABLE 0

/**********************
 *	CONSTANTS
 **********************/


#define TARGET_REACHED_DELAY_CYCLES	(50)

#define SCHED_ALLOWED_WIDTH	(5)

#define CONTROL_SAVE_DELAY	(5000)

/**********************
 *	MACROS
 **********************/

#define DEG2INC(deg)	((int32_t)(((float)-(deg))*4*1024*66/1/360))

/**********************
 *	TYPEDEFS
 **********************/



/**********************
 *	VARIABLES
 **********************/

static CONTROL_INST_t control;

static CONTROL_SCHED_t sched_allowed[][SCHED_ALLOWED_WIDTH] = {
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_MOVE, CONTROL_SCHED_CALIBRATE, CONTROL_SCHED_ARM, CONTROL_SCHED_NOTHING}, 		//IDLE
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING}, 	//CALIBRATION
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_IGNITE, CONTROL_SCHED_DISARM, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING}, 	//ARMED
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING}, 	//COUNTDOWN
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING}, 	//IGNITION
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING}, 	//THRUST
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING}, 	//SHUTDOWN
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING}, 	//GLIDE
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_RECOVER, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING}, 	//ABORT
		{CONTROL_SCHED_ABORT, CONTROL_SCHED_RECOVER, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING} 	//ERROR
};


/**********************
 *	PROTOTYPES
 **********************/
static void init_control(CONTROL_INST_t * control);
static void control_update(CONTROL_INST_t * control);

// Enter state functions
static void init_idle(CONTROL_INST_t * control);
static void init_calibration(CONTROL_INST_t * control);
static void init_armed(CONTROL_INST_t * control);
static void init_countdown(CONTROL_INST_t * control);
static void init_ignition(CONTROL_INST_t * control);
static void init_thrust(CONTROL_INST_t * control);
static void init_shutdown(CONTROL_INST_t * control);
static void init_glide(CONTROL_INST_t * control);
static void init_abort(CONTROL_INST_t * control);
static void init_error(CONTROL_INST_t * control);

// Main state functions
static void idle(CONTROL_INST_t * control);
static void calibration(CONTROL_INST_t * control);
static void armed(CONTROL_INST_t * control);
static void countdown(CONTROL_INST_t * control);
static void ignition(CONTROL_INST_t * control);
static void thrust(CONTROL_INST_t * control);
static void shutdown(CONTROL_INST_t * control);
static void glide(CONTROL_INST_t * control);
static void _abort(CONTROL_INST_t * control);
static void error(CONTROL_INST_t * control);

//scheduling

static uint8_t control_sched_should_run(CONTROL_INST_t * control, CONTROL_SCHED_t num);
static void control_sched_done(CONTROL_INST_t * control, CONTROL_SCHED_t num);
static void control_sched_set(CONTROL_INST_t * control, CONTROL_SCHED_t num);

/**********************
 *	DECLARATIONS
 **********************/

static THD_WORKING_AREA(control_thread_wa, 1024);
static THD_FUNCTION(control_thread, arg) {

	(void) arg;

	chRegSetThreadName(__FUNCTION__);

	systime_t thread_time = chVTGetSystemTime();

	init_control(&control);


	static EPOS4_INST_t pp_epos4;
	static EPOS4_INST_t ab_epos4;

	epos4_global_init();

	epos4_init(&pp_epos4, 1);
	//epos4_init_bridged(&ab_epos4, &pp_epos4, 2);
	//Bridged func not yet ready

	control.pp_epos4 = &pp_epos4;
	control.ab_epos4 = &ab_epos4;


	for(;;) {


		control_update(&control);
		//read status
		//read motor pos
		//read error register
		//??
		//For PP and AB

		switch(control.state) {
		case CS_IDLE:
			idle(&control);
			break;
		case CS_CALIBRATION:
			calibration(&control);
			break;
		case CS_ARMED:
			armed(&control);
			break;
		case CS_COUNTDOWN:
			countdown(&control);
			break;
		case CS_IGNITION:
			ignition(&control);
			break;
		case CS_THRUST:
			thrust(&control);
			break;
		case CS_SHUTDOWN:
			shutdown(&control);
			break;
		case CS_GLIDE:
			glide(&control);
			break;
		case CS_ABORT:
			_abort(&control);
			break;
		case CS_ERROR:
			error(&control);
			break;
		default:
			control.state = CS_ERROR;
			break;
		}
		thread_time = chThdSleepUntilWindowed(thread_time, thread_time+TIME_MS2I(CONTROL_HEART_BEAT));
	}
}


static void control_update(CONTROL_INST_t * control) {

	//TIMING TEST
	//HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);

	control->last_time = control->time;
	control->time = TIME_I2MS(chVTGetSystemTime());
	control->iter++;

	if(control->counter_active) {
		control->counter -= (control->time - control->last_time);
	}

	//control->msg = can_readBuffer();

	if(control->msg.id == DATA_ID_OPERATION){
		if(control->msg.data == DATA_MAGIC_1) {
			control_ignite();
		}
	}
	if(control->msg.id == DATA_ID_ARMING){
		if(control->msg.data == DATA_MAGIC_1) {
			control_arm();
		}
		if(control->msg.data == DATA_MAGIC_2) {
			control_disarm();
		}
	}
	if(control->msg.id == DATA_ID_VENTING){
		if(control->msg.data == DATA_MAGIC_1) {
			control_open_vent();
		}
		if(control->msg.data == DATA_MAGIC_2) {
			control_close_vent();
		}
	}
	if(control->msg.id == DATA_ID_CALIBRATION){
		if(control->msg.data == DATA_MAGIC_1) {
			control_calibrate();
		}
		if(control->msg.data == DATA_MAGIC_2) {
			control_recover();
		}
	}
	if(control->msg.id == DATA_ID_ABORT){
		if(control->msg.data == DATA_MAGIC_3) {
			control_abort();
		}
	}

	//do something depending on what msg was received


	//read motors parameters
	epos4_sync(control->pp_epos4);

	control->venting = palReadPad(VENTING_PORT, VENTING_PIN);

	if(control->pp_epos4->error && control->state != CS_ERROR) {
		init_error(control);
	}

	//init error if there is an issue with a motor

	if(control_sched_should_run(control, CONTROL_SCHED_ABORT)) {
		init_abort(control);
		control_sched_done(control, CONTROL_SCHED_ABORT);
	}
}

static void init_control(CONTROL_INST_t * control) {
	control->sched = CONTROL_SCHED_NOTHING;
	control->counter_active = 0;
	control->state = CS_IDLE;
	led_set_color(LED_COLOR_GREEN);

	control->pp_params.acc = 25000;
	control->pp_params.dec = 25000;
	control->pp_params.speed = 8000;
	control->pp_params.countdown_wait = 2000;
	control->pp_params.half_wait = 2500;
	control->pp_params.full_wait = 20000;
	control->pp_params.half_angle = DEG2INC(27);
	control->pp_params.full_angle = DEG2INC(90);


	//CAN_Config(CAN_ID_PROPULSION_BOARD);
}

static void init_idle(CONTROL_INST_t * control) {
	control->state = CS_IDLE;
	led_set_color(LED_COLOR_GREEN);
	control->counter_active = 0;
	control->mov_started = 0;
	control->pp_close_mov_started = 0;
	control->pp_abort_mov_started = 0;
	control->pp_motor_prepped = 0;
	/*
	storage_disable();
	*/
}

static void idle(CONTROL_INST_t * control) {

	if(control->mov_started) {
		control->mov_started++;
		if(control->mov_started > TARGET_REACHED_DELAY_CYCLES) {
			uint8_t terminated = 0;
			epos4_ppm_terminate(control->pp_epos4, &terminated);
			if(terminated) {
				control->mov_started = 0;
				control_sched_done(control, CONTROL_SCHED_MOVE);
			}
		}
	}

	//if a move is scheduled, perform it
	if(control_sched_should_run(control, CONTROL_SCHED_MOVE) && !control->mov_started) {
		EPOS4_PPM_CONFIG_t ppm_config;
		ppm_config.profile_acceleration = control->pp_params.acc;
		ppm_config.profile_deceleration = control->pp_params.dec;
		ppm_config.profile_velocity = control->pp_params.speed;
		epos4_ppm_config(control->pp_epos4, ppm_config);
		epos4_ppm_prep(control->pp_epos4);
		epos4_ppm_move(control->pp_epos4, control->mov_type, control->mov_target);
		control->mov_started = 1;
	}

	if(control_sched_should_run(control, CONTROL_SCHED_CALIBRATE)) {
		init_calibration(control);
		control_sched_done(control, CONTROL_SCHED_CALIBRATE);
	}

	if(control_sched_should_run(control, CONTROL_SCHED_ARM)) {
		init_armed(control);
		control_sched_done(control, CONTROL_SCHED_ARM);
	}


}

static void init_calibration(CONTROL_INST_t * control) {
	control->state = CS_CALIBRATION;
	led_set_color(LED_COLOR_BLUE);
	sensor_calib();
	EPOS4_HOM_CONFIG_t config;
	config.method = EPOS4_ACTUAL_POSITION;
	config.home_offset = 0;
	epos4_hom_config(control->pp_epos4, config);
	epos4_hom_move(control->pp_epos4);
}

static void calibration(CONTROL_INST_t * control) {
	//Wait for the calibration ack to come from the sensors
	//perform motor homing --> current position as home
	//perform sensor calibration

	uint8_t terminated = 0;
	epos4_hom_terminate(control->pp_epos4, &terminated);

	if(sensor_calib_done() && terminated) {
		init_idle(control);
	}
}

static void init_armed(CONTROL_INST_t * control) {
	control->state = CS_ARMED;
	led_set_color(LED_COLOR_YELLOW);
}

static void armed(CONTROL_INST_t * control) {

	if(!control->pp_motor_prepped) {
		EPOS4_PPM_CONFIG_t ppm_config;
		ppm_config.profile_acceleration = control->pp_params.acc;
		ppm_config.profile_deceleration = control->pp_params.dec;
		ppm_config.profile_velocity = control->pp_params.speed;
		epos4_ppm_config(control->pp_epos4, ppm_config);
		if(epos4_ppm_prep(control->pp_epos4) == EPOS4_SUCCESS) {
			control->pp_motor_prepped = 1;
		} else {
			//disabled for testing at home
			init_error(control);  //error state in case of motor failure
		}
	}

	if(control_sched_should_run(control, CONTROL_SCHED_IGNITE)) {
		init_countdown(control);
		control_sched_done(control, CONTROL_SCHED_IGNITE);
	}

	if(control_sched_should_run(control, CONTROL_SCHED_DISARM)) {
		epos4_ppm_unprep(control->pp_epos4);
		control->pp_motor_prepped = 0;
		init_idle(control);
		control_sched_done(control, CONTROL_SCHED_DISARM);
	}
}

static void init_countdown(CONTROL_INST_t * control) {
	led_set_color(LED_COLOR_ORANGE);
	control->state = CS_COUNTDOWN;
	control->counter = control->pp_params.countdown_wait-CONTROL_HEART_BEAT;
	control->counter_active = 1;
	/*
	storage_restart();
	storage_enable();
	*/
}

static void countdown(CONTROL_INST_t * control) {
	if(control->counter <= 0) {
		control->counter_active = 0;
		init_ignition(control);
	}
}

static void init_ignition(CONTROL_INST_t * control) {
	control->state = CS_IGNITION;
	led_set_color(LED_COLOR_LILA);
	control->counter = control->pp_params.half_wait-CONTROL_HEART_BEAT;
	control->counter_active = 1;
	epos4_ppm_move(control->pp_epos4, EPOS4_ABSOLUTE_IMMEDIATE, control->pp_params.half_angle);
}

static void ignition(CONTROL_INST_t * control) {
	if(control->counter <= 0) {
		control->counter_active = 0;
		init_thrust(control);
	}

}

static void init_thrust(CONTROL_INST_t * control) {
	control->state = CS_THRUST;
	led_set_color(LED_COLOR_TEAL);
	control->counter = control->pp_params.full_wait-CONTROL_HEART_BEAT;
	control->counter_active = 1;
	epos4_ppm_move(control->pp_epos4, EPOS4_ABSOLUTE_IMMEDIATE, control->pp_params.full_angle);
#if THRUST_CONTROL_ENABLE == 1
	//TC start
#endif
}

static void thrust(CONTROL_INST_t * control) {
#if THRUST_CONTROL_ENABLE == 1
	//THRUST CONTROL HERE

	epos4_ppm_move(control->pp_epos4, EPOS4_ABSOLUTE_IMMEDIATE, DEG2INC(90));
#endif

	if(control->counter <= 0) {
		control->counter_active = 0;
		init_shutdown(control);
	}
	//detect flameout (pressure lower than flameout threshold)
	//-> init shutdown
}

static void init_shutdown(CONTROL_INST_t * control) {
	control->state = CS_SHUTDOWN;
	epos4_ppm_move(control->pp_epos4, EPOS4_ABSOLUTE_IMMEDIATE, 0);
	control->pp_close_mov_started = 1;
}

static void shutdown(CONTROL_INST_t * control) {
	if(control->pp_close_mov_started) {
		control->pp_close_mov_started++;
		if(control->pp_close_mov_started > TARGET_REACHED_DELAY_CYCLES) {
			uint8_t terminated = 0;
			epos4_ppm_terminate(control->pp_epos4, &terminated);
			if(terminated) {
				control->pp_close_mov_started = 0;
				control->pp_motor_prepped = 0;
				init_glide(control);
			}
		}
	}
}

static void init_glide(CONTROL_INST_t * control) {
	control->state = CS_GLIDE;
}

static void glide(CONTROL_INST_t * control) {
	//AB algorithm controls the airbrakes motor

	//expect a stop signal to go to idle
	init_idle(control);
}

static void init_abort(CONTROL_INST_t * control) {
	led_set_color(LED_COLOR_PINK);
	control->state = CS_ABORT;
	epos4_ppm_move(control->pp_epos4, EPOS4_ABSOLUTE_IMMEDIATE, 0);
	control->pp_abort_mov_started = 1;
	control->counter_active=0;
	//storage_disable();
}

static void _abort(CONTROL_INST_t * control) {
	//close main valve
	//close airbrakes
	//wait for user release
	if(control->pp_abort_mov_started) {
		control->pp_abort_mov_started++;
		if(control->pp_abort_mov_started > TARGET_REACHED_DELAY_CYCLES) {
			uint8_t terminated = 0;
			epos4_ppm_terminate(control->pp_epos4, &terminated);
			if(terminated) {
				control->pp_abort_mov_started = 0;
				control->pp_motor_prepped = 0;
			}
		}
	}

	if(control_sched_should_run(control, CONTROL_SCHED_RECOVER)) {
		init_idle(control);
		control_sched_done(control, CONTROL_SCHED_RECOVER);
	}
}

static void init_error(CONTROL_INST_t * control) {
	led_set_color(LED_COLOR_RED);
	control->state = CS_ERROR;
	control->counter_active = 0;
	//storage_disable();
}

static void error(CONTROL_INST_t * control) {
	//wait for user release
	if(control_sched_should_run(control, CONTROL_SCHED_RECOVER)) {
		epos4_recover(control->pp_epos4);
		init_idle(control);
		control_sched_done(control, CONTROL_SCHED_RECOVER);
	}
}

CONTROL_STATE_t control_get_state() {
	return control.state;
}

CONTROL_PP_PARAMS_t control_get_pp_params() {
	return control.pp_params;
}

void control_set_pp_params(CONTROL_PP_PARAMS_t params) {
	control.pp_params = params;
}

void control_move(EPOS4_MOV_t mov_type, int32_t target) {
	control_sched_set(&control, CONTROL_SCHED_MOVE);
	control.mov_type = mov_type;
	control.mov_target = target;
	control.mov_started = 0;
}

void control_calibrate() {
	control_sched_set(&control, CONTROL_SCHED_CALIBRATE);
}

void control_arm() {
	control_sched_set(&control, CONTROL_SCHED_ARM);
}

void control_disarm() {
	control_sched_set(&control, CONTROL_SCHED_DISARM);
}

void control_ignite() {
	control_sched_set(&control, CONTROL_SCHED_IGNITE);
}

void control_abort() {
	control_sched_set(&control, CONTROL_SCHED_ABORT);
}

void control_recover() {
	control_sched_set(&control, CONTROL_SCHED_RECOVER);
}

CONTROL_STATUS_t control_get_status() {
	CONTROL_STATUS_t status;
	status.state = control.state;
	status.pp_error = control.pp_epos4->error;
	status.pp_psu_voltage = control.pp_epos4->psu_voltage;
	status.pp_position = control.pp_epos4->position;
	status.pp_status = control.pp_epos4->status;
	status.counter = control.counter;
	status.time = control.time;
	status.venting = control.venting;
	return status;
}

uint8_t control_open_vent() {
	palSetPad(VENTING_PORT, VENTING_PIN);
	return palReadPad(VENTING_PORT, VENTING_PIN);
}

uint8_t control_close_vent() {
	palClearPad(VENTING_PORT, VENTING_PIN);
	return palReadPad(VENTING_PORT, VENTING_PIN);
}

static uint8_t control_sched_should_run(CONTROL_INST_t * control, CONTROL_SCHED_t num) {
	return control->sched == num;
}

static void control_sched_done(CONTROL_INST_t * control, CONTROL_SCHED_t num) {
	if(control->sched == num) {
		control->sched = CONTROL_SCHED_NOTHING;
	} else {
		init_error(control);
	}
}

static void control_sched_set(CONTROL_INST_t * control, CONTROL_SCHED_t num) {
	if(control->sched == CONTROL_SCHED_NOTHING) {
		for(uint8_t i = 0; i < SCHED_ALLOWED_WIDTH; i++) {
			if(sched_allowed[control->state][i] == num) {
				control->sched = num;
				return;
			}
		}
	}
}


void control_start(tprio_t prio) {
	chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa), prio, control_thread, NULL);
}



/* END */


