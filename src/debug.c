/*  Title		: Debug
 *  Filename	: debug.c
 *	Author		: iacopo sprenger
 *	Date		: 20.01.2021
 *	Version		: 0.1
 *	Description	: debug interface code
 *	What needs to be available:
 *	Access the internal state
 *	Trigger ignition and flight sequence
 *	Trigger calibration
 *	Read sensor data
 *	Read stored data
 *	Configure ignition sequence parameters
 */

/**********************
 *	INCLUDES
 **********************/

#include <debug.h>
#include <control.h>
#include <sensor.h>


/**********************
 *	CONSTANTS
 **********************/

#define DEBUG_UART UARTD3


#define PP_PARAMS_LEN (32)
#define PP_MOVE_LEN (6)
#define STATUS_LEN (20)
#define SENSOR_LEN (24)
#define VENTING_LEN	(2)
#define DOWNLOAD_LEN	(4)

#define SENSOR_BFR	(5)
#define SENSOR_BFR_LEN SENSOR_BFR*SENSOR_LEN


#define ERROR_LO	(0xce)
#define ERROR_HI	(0xec)

#define CRC_ERROR_LO	(0xbe)
#define CRC_ERROR_HI	(0xeb)

#define OK_LO	(0xc5)
#define OK_HI	(0x5c)

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

//debug routines
static void debug_read_state(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_set_pp_params(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_get_pp_params(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_pp_move(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_calibrate(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_arm(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_disarm(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_ignite(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_abort(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_recover(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_get_sensor(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_get_status(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
static void debug_venting(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);
//static void debug_download(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len);

/**********************
 *	DEBUG FCN ARRAY
 **********************/
static void (*debug_fcn[]) (uint8_t *, uint16_t, uint8_t *, uint16_t *) = {
		debug_read_state,		//0x00
		debug_set_pp_params,	//0x01
		debug_get_pp_params, 	//0x02
		debug_pp_move,			//0x03
		debug_calibrate,		//0x04
		debug_arm,				//0x05
		debug_disarm,			//0x06
		debug_ignite,			//0x07
		debug_abort,			//0x08
		debug_recover,			//0x09
		debug_get_sensor,		//0x0A
		debug_get_status,		//0x0B
		debug_venting,			//0x0C
		//debug_download			//0x0D
};

static uint16_t debug_fcn_max = sizeof(debug_fcn) / sizeof(void *);

/**********************
 *	DECLARATIONS
 **********************/


//Requires an instance of type debug
SERIAL_RET_t debug_decode_fcn(void * inst, uint8_t data) {
	static uint8_t send_data[MSV2_MAX_DATA_LEN];
	static uint16_t length = 0;
	static uint16_t bin_length = 0;
	DEBUG_INST_t * debug = (DEBUG_INST_t *) inst;
	MSV2_ERROR_t tmp = msv2_decode_fragment(&debug->msv2, data);

	if(tmp == MSV2_SUCCESS) {
		if(debug->msv2.rx.opcode < debug_fcn_max) {

			debug_fcn[debug->msv2.rx.opcode](debug->msv2.rx.data, debug->msv2.rx.length, send_data, &length);
			//length is in words
			bin_length = msv2_create_frame(&debug->msv2, debug->msv2.rx.opcode, length/2, send_data);
			serial_send(&debug->ser, msv2_tx_data(&debug->msv2), bin_length);
		} else {
			send_data[0] = CRC_ERROR_LO;
			send_data[1] = CRC_ERROR_HI;
			length = 2;
			bin_length = msv2_create_frame(&debug->msv2, debug->msv2.rx.opcode, length/2, send_data);
			serial_send(&debug->ser, msv2_tx_data(&debug->msv2), bin_length);
		}
	}

	return tmp;
}

void debug_init(DEBUG_INST_t * debug) {
	static uint32_t id_counter = 0;
	msv2_init(&debug->msv2);
	serial_init(&debug->ser, &DEBUG_UART, debug, debug_decode_fcn);
	debug->id = id_counter++;
}

static void debug_read_state(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	CONTROL_STATE_t state = control_get_state();
	resp[0] = state;
	resp[1] = 0x00;
	*resp_len = 2;
}

static void debug_set_pp_params(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	if(data_len == PP_PARAMS_LEN){
		CONTROL_PP_PARAMS_t params;
		params.acc = util_decode_u32(data);
		params.dec = util_decode_u32(data+4);
		params.speed = util_decode_u32(data+8);
		params.countdown_wait = util_decode_u32(data+12);
		params.half_wait = util_decode_u32(data+16);
		params.full_wait = util_decode_u32(data+20);
		params.half_angle = util_decode_i32(data+24);
		params.full_angle = util_decode_i32(data+28);
		control_set_pp_params(params);
		resp[0] = OK_LO;
		resp[1] = OK_HI;
		*resp_len = 2;
	} else {
		resp[0] = ERROR_LO;
		resp[1] = ERROR_HI;
		*resp_len = 2;
	}
}

static void debug_get_pp_params(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	CONTROL_PP_PARAMS_t params = control_get_pp_params();
	util_encode_u32(resp, params.acc);
	util_encode_u32(resp+4, params.dec);
	util_encode_u32(resp+8, params.speed);
	util_encode_u32(resp+12, params.countdown_wait);
	util_encode_u32(resp+16, params.half_wait);
	util_encode_u32(resp+20, params.full_wait);
	util_encode_i32(resp+24, params.half_angle);
	util_encode_i32(resp+28, params.full_angle);
	*resp_len = PP_PARAMS_LEN;
}

static void debug_pp_move(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	if(data_len == PP_MOVE_LEN) {
		int32_t target = util_decode_i32(data);
		uint16_t mode = util_decode_u16(data+4);
		control_move(mode, target);
		resp[0] = OK_LO;
		resp[1] = OK_HI;
		*resp_len = 2;
	} else {
		resp[0] = ERROR_LO;
		resp[1] = ERROR_HI;
		*resp_len = 2;
	}
}

static void debug_calibrate(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	control_calibrate();
	resp[0] = OK_LO;
	resp[1] = OK_HI;
	*resp_len = 2;
}

static void debug_arm(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	control_arm();
	resp[0] = OK_LO;
	resp[1] = OK_HI;
	*resp_len = 2;
}

static void debug_disarm(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	control_disarm();
	resp[0] = OK_LO;
	resp[1] = OK_HI;
	*resp_len = 2;
}

static void debug_ignite(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	control_ignite();
	resp[0] = OK_LO;
	resp[1] = OK_HI;;
	*resp_len = 2;
}

static void debug_abort(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	control_abort();
	resp[0] = OK_LO;
	resp[1] = OK_HI;
	*resp_len = 2;
}

static void debug_recover(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	control_recover();
	resp[0] = OK_LO;
	resp[1] = OK_HI;
	*resp_len = 2;
}

static void debug_get_sensor(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	SENSOR_DATA_t sensor = sensor_get_last();
	util_encode_i32(resp, sensor.pressure_1);
	util_encode_i32(resp+4, sensor.pressure_2);
	util_encode_i32(resp+8, sensor.temperature[0]);
	util_encode_i32(resp+12, sensor.temperature[1]);
	util_encode_i32(resp+16, sensor.temperature[2]);
	util_encode_u32(resp+20, sensor.time);
	*resp_len = SENSOR_LEN;
}

static void debug_get_status(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	(void) data;
	(void) data_len;
	CONTROL_STATUS_t status = control_get_status();
	util_encode_u16(resp, status.state);
	util_encode_u16(resp+2, status.pp_psu_voltage);
	util_encode_u16(resp+4, status.pp_error);
	util_encode_u16(resp+6, status.pp_status);
	util_encode_i32(resp+8, status.pp_position);
	util_encode_i32(resp+12, status.counter);
	uint32_t memory = 0;//storage_get_used();
	util_encode_u32(resp+16, memory);
	*resp_len = STATUS_LEN;
}

static void debug_venting(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	if(data_len == VENTING_LEN) {
		uint16_t action = util_decode_u16(data);
		uint16_t status = 0;
		if(action) {
			status = control_open_vent();
		} else {
			status = control_close_vent();
		}
		util_encode_u16(resp, status);
		*resp_len = 2;
	} else {
		resp[0] = ERROR_LO;
		resp[1] = ERROR_HI;
		*resp_len = 2;
	}
}
/*
static void debug_download(uint8_t * data, uint16_t data_len, uint8_t * resp, uint16_t * resp_len) {
	//downloads 5 samples at a certain location
	if(data_len == DOWNLOAD_LEN) {
		uint32_t location = util_decode_u32(data);
		for(uint8_t i = 0; i < 5; i++) {
			storage_get_sample(location+i, resp+i*32);
		}
		*resp_len = 32*5;
	}
}
*/


void debug_start(void) {
	static DEBUG_INST_t debug;
	debug_init(&debug);
}



/* END */


