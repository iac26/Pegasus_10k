/*  Title		: Maxon EPOS4 driver
 *  Filename	: epos4.c
 *	Author		: iacopo sprenger
 *	Date		: 25.01.2021
 *	Version		: 0.1
 *	Description	: maxon epos4 driver all the epos4 boards must be attached to the same serial port.
 *				  Access to multiple boards can only be done by gateway.
 *				  NOT THREAD SAFE
 */

/**********************
 *	INCLUDES
 **********************/

#include <epos4.h>
#include <epos4_def.h>
#include <msv2.h>

#include <ch.h>
#include <hal.h>


/**********************
 *	CONFIGURATION
 **********************/


#define EPOS4_UART	UARTD6




/**********************
 *	CONSTANTS
 **********************/

//CONTROL SEQ PARAMS
#define DLE	0x90
#define STX	0x02
#define READ_OBJECT	0x60
#define READ_OBJECT_LEN 2
#define WRITE_OBJECT 0x68
#define WRITE_OBJECT_LEN 4
#define NODE_ID 0x01
#define DATA_SIZE 4

#define COMM_TIMEOUT 10
#define DRIV_TIMEOUT 200
#define LONG_TIME 0xffff

#define MAX_FRAME_LEN	64


//MOTOR_SPECIFIC SETTINGS
#define MAX_POS			0	//checked
#define MIN_POS			0 	//checked
#define MAX_PROFILE_VEL	8000 //check

#define QUICK_STOP_DEC	50000 //check
#define MAX_ACC			100000 //check

//MOTOR_CSTE TO BE CHANGED --> GENERIC DRIVER
#define MOTOR_TYPE	10			//sin com bldc motor

#define MOTOR_MAX_SPEED	10000	//check
#define MOTOR_NOM_CUR	7320	//checked
#define MOTOR_MAX_CURRENT	10000	//checked
#define MOTOR_THERMAL	327 //checked
#define MOTOR_TORQUE    12419 //checked
#define EL_RESISTANCE	139 //checked
#define EL_INDUCTANCE	93	//checked

#define GEAR_MAX_SPEED	8000 //check
#define GEAR_NUM		66  //check
#define GEAR_DEN		1 	//check

#define NUM_POLE_PAIRS	4 //checked

#define ENC_NB_PULSES	1024 //check
#define ENC_TYPE		0x0001 //check

#define HALL_TYPE		0x0000 //check
#define HALL_PATTERN	0x0005 //check

//controllers
#define CURRENT_P		140106 //check
#define CURRENT_I		203253 //check

#define POSITION_P		1703746 //check
#define POSITION_I		8397288 //check
#define POSITION_D		25239 //check
#define POSITION_FFV	3568 //check
#define POSITION_FFA	162 //check

#define FOLLOW_WINDOW	1000000


/**********************
 *	MACROS
 **********************/




/**********************
 *	TYPEDEFS
 **********************/






/**********************
 *	VARIABLES
 **********************/

//semaphore (allows )

static binary_semaphore_t epos4_rx_sem;

static binary_semaphore_t epos4_busy_sem;




/**********************
 *	PROTOTYPES
 **********************/


//What is needed:
/*
 * EPOS4 state machine interaction:
 * 	startup, enable, recover from fault, change operating mode, etc...
 * 	Send motion orders absolute, relative
 * 	Perform homing
 *
 */

/**********************
 *	DECLARATIONS
 **********************/


void epos4_global_init(void) {
	//create rx mutex
	chBSemObjectInit(&epos4_rx_sem, TRUE);
	chBSemObjectInit(&epos4_busy_sem, FALSE);
	//this semaphore will be epos specific
}

void epos4_init(EPOS4_INST_t * epos4, uint8_t id) {
	static uint32_t id_counter = 0;
	epos4->id = id_counter++;

	msv2_init(&epos4->msv2);

	epos4->can_id = id;

	serial_init(&epos4->ser, &EPOS4_UART, epos4, epos4_decode_fcn);

}

void epos4_init_bridged(EPOS4_INST_t * epos4, EPOS4_INST_t * parent, uint8_t id) {
	(void)epos4;
	(void)parent;
	(void)id;
	// add a "clause" to the serial decode of the bridge that it checks
	// which board is communicating then take that into account
	// when providing "answer" to the requesting driver
}


//mutex for only one access at the same time per serial port
EPOS4_ERROR_t epos4_readobject(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, uint8_t * data, uint32_t * err) {
	if (MSG_OK == chBSemWaitTimeout(&epos4_busy_sem, TIME_MS2I(DRIV_TIMEOUT))) {
		static uint8_t send_data[READ_OBJECT_LEN*2];
		static uint16_t length = 0;
		send_data[0] = epos4->id; //node ID
		send_data[1] = index & 0xff;
		send_data[2] = index >> 8;
		send_data[3] = subindex;
		length = msv2_create_frame(&epos4->msv2, READ_OBJECT, READ_OBJECT_LEN, send_data);
		serial_send(&epos4->ser, msv2_tx_data(&epos4->msv2), length);
		if(MSG_OK == chBSemWaitTimeout(&epos4_rx_sem, TIME_MS2I(COMM_TIMEOUT))) {
			uint8_t * recieved_data = msv2_rx_data(&epos4->msv2);
			*err = recieved_data[0] | (recieved_data[1]<<8) | (recieved_data[2]<<16) | (recieved_data[3]<<24);
			for(uint8_t i = 0; i < DATA_SIZE; i++){
				data[i] = recieved_data[4+i];
			}
			if(*err == 0) {
				chBSemSignal(&epos4_busy_sem);
				return EPOS4_SUCCESS;
			} else {
				chBSemSignal(&epos4_busy_sem);
				return EPOS4_REMOTE_ERROR;
			}
		} else {
			chBSemSignal(&epos4_busy_sem);
			return EPOS4_TIMEOUT;
		}
	} else {
		return EPOS4_BUSY;
	}
}

EPOS4_ERROR_t epos4_writeobject(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, uint8_t * data, uint32_t * err) {
	if (MSG_OK == chBSemWaitTimeout(&epos4_busy_sem, TIME_MS2I(DRIV_TIMEOUT))) {
		static uint8_t send_data[WRITE_OBJECT_LEN*2];
		static uint16_t length = 0;
		send_data[0] = epos4->id; //node ID
		send_data[1] = index & 0xff;
		send_data[2] = index >> 8;
		send_data[3] = subindex;
		for(uint8_t i = 0; i < DATA_SIZE; i++){
			send_data[4+i] = data[i];
		}
		length = msv2_create_frame(&epos4->msv2, WRITE_OBJECT, WRITE_OBJECT_LEN, send_data);
		serial_send(&epos4->ser, msv2_tx_data(&epos4->msv2), length);
		if(MSG_OK == chBSemWaitTimeout(&epos4_rx_sem, TIME_MS2I(COMM_TIMEOUT))) {
			uint8_t * recieved_data = msv2_rx_data(&epos4->msv2);
			*err = recieved_data[0] | (recieved_data[1]<<8) | (recieved_data[2]<<16) | (recieved_data[3]<<24);
			if(*err == 0) {
				chBSemSignal(&epos4_busy_sem);
				return EPOS4_SUCCESS;
			} else {
				chBSemSignal(&epos4_busy_sem);
				return EPOS4_REMOTE_ERROR;
			}
		} else {
			chBSemSignal(&epos4_busy_sem);
			return EPOS4_TIMEOUT;
		}
	} else {
		return EPOS4_BUSY;
	}
}

SERIAL_RET_t epos4_decode_fcn(void * inst, uint8_t data) {
	EPOS4_INST_t * epos4 = (EPOS4_INST_t * ) inst;
	MSV2_ERROR_t tmp = msv2_decode_fragment(&epos4->msv2, data);
	//this should release the semaphore corresponding the the right epos board if bridged
	if(tmp == MSV2_SUCCESS || tmp == MSV2_WRONG_CRC) {
		chBSemSignal(&epos4_rx_sem); // one frame has been received!
	}
	return tmp;
}


EPOS4_ERROR_t epos4_write_u8(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, uint8_t data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	util_encode_u8(bin_data, data);
	return epos4_writeobject(epos4, index, subindex, bin_data, err);
}

EPOS4_ERROR_t epos4_write_u16(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, uint16_t data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	util_encode_u16(bin_data, data);
	return epos4_writeobject(epos4, index, subindex, bin_data, err);
}

EPOS4_ERROR_t epos4_write_u32(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, uint32_t data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	util_encode_u32(bin_data, data);
	return epos4_writeobject(epos4, index, subindex, bin_data, err);
}

EPOS4_ERROR_t epos4_write_i8(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, int8_t data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	util_encode_i8(bin_data, data);
	return epos4_writeobject(epos4, index, subindex, bin_data, err);
}

EPOS4_ERROR_t epos4_write_i16(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, int16_t data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	util_encode_i16(bin_data, data);
	return epos4_writeobject(epos4, index, subindex, bin_data, err);
}

EPOS4_ERROR_t epos4_write_i32(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, int32_t data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	util_encode_i32(bin_data, data);
	return epos4_writeobject(epos4, index, subindex, bin_data, err);
}

EPOS4_ERROR_t epos4_read_u8(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, uint8_t * data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	EPOS4_ERROR_t tmp = epos4_readobject(epos4, index, subindex, bin_data, err);
	*data = util_decode_u8(bin_data);
	return tmp;
}

EPOS4_ERROR_t epos4_read_u16(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, uint16_t * data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	EPOS4_ERROR_t tmp = epos4_readobject(epos4, index, subindex, bin_data, err);
	*data = util_decode_u16(bin_data);
	return tmp;
}

EPOS4_ERROR_t epos4_read_u32(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, uint32_t * data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	EPOS4_ERROR_t tmp = epos4_readobject(epos4, index, subindex, bin_data, err);
	*data = util_decode_u32(bin_data);
	return tmp;
}

EPOS4_ERROR_t epos4_read_i8(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, int8_t * data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	EPOS4_ERROR_t tmp = epos4_readobject(epos4, index, subindex, bin_data, err);
	*data = util_decode_i8(bin_data);
	return tmp;
}

EPOS4_ERROR_t epos4_read_i16(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, int16_t * data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	EPOS4_ERROR_t tmp = epos4_readobject(epos4, index, subindex, bin_data, err);
	*data = util_decode_i16(bin_data);
	return tmp;
}

EPOS4_ERROR_t epos4_read_i32(EPOS4_INST_t * epos4, uint16_t index, uint8_t subindex, int32_t * data, uint32_t * err) {
	uint8_t bin_data[DATA_SIZE];
	EPOS4_ERROR_t tmp = epos4_readobject(epos4, index, subindex, bin_data, err);
	*data = util_decode_i32(bin_data);
	return tmp;
}

EPOS4_ERROR_t epos4_sync(EPOS4_INST_t * epos4) {
	uint32_t err;
	EPOS4_ERROR_t error = 0;

	error |= epos4_read_statusword(epos4, &epos4->status, &err);

	error |= epos4_read_u16(epos4, EPOS4_ERROR_WORD, &epos4->error, &err);

	error |= epos4_read_i32(epos4, EPOS4_ACTUAL_POSITION, &epos4->position, &err);

	error |= epos4_read_u16(epos4, EPOS4_PSU_VOLTAGE, &epos4->psu_voltage, &err);

	return error;
}


EPOS4_ERROR_t epos4_config(EPOS4_INST_t * epos4) {
	uint32_t err;
	EPOS4_ERROR_t error = 0;

	error |= epos4_write_u16(epos4, EPOS4_MOTOR_TYPE, MOTOR_TYPE, &err);

	error |= epos4_write_u32(epos4, EPOS4_MOTOR_NOMINAL_CURRENT, MOTOR_NOM_CUR, &err);

	error |= epos4_write_u32(epos4, EPOS4_MOTOR_CURRENT_LIMIT, MOTOR_MAX_CURRENT, &err);

	error |= epos4_write_u8(epos4, EPOS4_MOTOR_POLE_PAIRS, NUM_POLE_PAIRS, &err);

	error |= epos4_write_u16(epos4, EPOS4_MOTOR_THERMAL_CST, MOTOR_THERMAL, &err);

	error |= epos4_write_u32(epos4, EPOS4_MOTOR_TORQUE_CST, MOTOR_TORQUE, &err);

	error |= epos4_write_u32(epos4, EPOS4_MOTOR_MAX_SPEED, MOTOR_MAX_SPEED, &err);

	error |= epos4_write_u32(epos4, EPOS4_GEAR_MAX_INPUT_SPEED, GEAR_MAX_SPEED, &err);

	error |= epos4_write_u32(epos4, EPOS4_GEAR_NUMERATOR, GEAR_NUM, &err);

	error |= epos4_write_u32(epos4, EPOS4_GEAR_DENOMINATOR, GEAR_DEN, &err);

	error |= epos4_write_u32(epos4, EPOS4_MAX_PROFILE_VELOCITY, MAX_PROFILE_VEL, &err);

	error |= epos4_write_u32(epos4, EPOS4_MAX_ACCELERATION, MAX_ACC, &err);

	error |= epos4_write_i32(epos4, EPOS4_SOFTWARE_MIN_POSITION, MAX_POS, &err);

	error |= epos4_write_i32(epos4, EPOS4_SOFTWARE_MAX_POSITION, MIN_POS, &err);

	error |= epos4_write_u32(epos4, EPOS4_ELECTRICAL_RESISTANCE, EL_RESISTANCE, &err);

	error |= epos4_write_u16(epos4, EPOS4_ELECTRICAL_INDUCTANCE, EL_INDUCTANCE, &err);

	error |= epos4_write_u32(epos4, EPOS4_ENC1_NB_PULSES, ENC_NB_PULSES, &err);

	error |= epos4_write_u16(epos4, EPOS4_ENC1_TYPE, ENC_TYPE, &err);

	error |= epos4_write_u16(epos4, EPOS4_HALL_TYPE, HALL_TYPE, &err);

	error |= epos4_write_u32(epos4, EPOS4_CUR_CTRL_P, CURRENT_P, &err);

	error |= epos4_write_u32(epos4, EPOS4_CUR_CTRL_I, CURRENT_I, &err);

	error |= epos4_write_u32(epos4, EPOS4_PPM_CTRL_P, POSITION_P, &err);

	error |= epos4_write_u32(epos4, EPOS4_PPM_CTRL_I, POSITION_I, &err);

	error |= epos4_write_u32(epos4, EPOS4_PPM_CTRL_D, POSITION_D, &err);

	error |= epos4_write_u32(epos4, EPOS4_PPM_CTRL_FFV, POSITION_FFV, &err);

	error |= epos4_write_u32(epos4, EPOS4_PPM_CTRL_FFA, POSITION_FFA, &err);

	error |= epos4_write_u32(epos4, EPOS4_FOLLOW_ERROR_WINDOW, FOLLOW_WINDOW, &err);

	return error;
}

EPOS4_ERROR_t epos4_ppm_prep(EPOS4_INST_t * epos4) {
	EPOS4_ERROR_t error = 0;
	uint32_t err;

	error |= epos4_write_i8(epos4, EPOS4_MODE_OF_OPERATION, EPOS4_MODE_PPM, &err);

	error |= epos4_control_shutdown(epos4, &err);

	error |= epos4_control_soenable(epos4, &err);

	return error;
}

EPOS4_ERROR_t epos4_ppm_move(EPOS4_INST_t * epos4, EPOS4_MOV_t type, int32_t target) {
	EPOS4_ERROR_t error = 0;
	uint32_t err = 0;

	uint16_t status;

	error |= epos4_read_statusword(epos4, &status, &err);

	if(!EPOS4_SW_ENABLED(status)) {
		return EPOS4_ERROR;
	}

	error |= epos4_write_i32(epos4, EPOS4_TARGET_POSITION, target, &err);

	switch(type) {
	case EPOS4_ABSOLUTE:
		error |= epos4_control_ppm_start_abs(epos4, &err);
		break;
	case EPOS4_ABSOLUTE_IMMEDIATE:
		error |= epos4_control_ppm_start_abs_imm(epos4, &err);
		break;
	case EPOS4_RELATIVE:
		error |= epos4_control_ppm_start_rel(epos4, &err);
		break;
	case EPOS4_RELATIVE_IMMEDIATE:
		error |= epos4_control_ppm_start_rel_imm(epos4, &err);
		break;
	}

	error |= epos4_control_new_pos(epos4, &err);
	return error;
}

EPOS4_ERROR_t epos4_ppm_config(EPOS4_INST_t * epos4, EPOS4_PPM_CONFIG_t config) {
	EPOS4_ERROR_t error = 0;
	uint32_t err;

	error |= epos4_write_u32(epos4, EPOS4_PROFILE_VELOCITY, config.profile_velocity, &err);

	error |= epos4_write_u32(epos4, EPOS4_PROFILE_ACCELERATION, config.profile_acceleration, &err);

	error |= epos4_write_u32(epos4, EPOS4_PROFILE_DECELERATION, config.profile_deceleration, &err);

	return error;
}

EPOS4_ERROR_t epos4_ppm_terminate(EPOS4_INST_t * epos4, uint8_t * terminated) {
	EPOS4_ERROR_t error = 0;
	uint32_t err;
	*terminated = 0;

	uint16_t status;

	error |= epos4_read_statusword(epos4, &status, &err);


	if(EPOS4_SW_TARGET_REACHED(status)) {
		epos4_control_disable(epos4, &err);
		*terminated = 1;
	}

	return error;
}

EPOS4_ERROR_t epos4_ppm_unprep(EPOS4_INST_t * epos4) {
	EPOS4_ERROR_t error = 0;
	uint32_t err;
	error |= epos4_control_disable(epos4, &err);

	return error;
}

EPOS4_ERROR_t epos4_hom_config(EPOS4_INST_t * epos4, EPOS4_HOM_CONFIG_t config) {
	EPOS4_ERROR_t error = 0;
	uint32_t err;

	error |= epos4_write_i8(epos4, EPOS4_MODE_OF_OPERATION, EPOS4_MODE_HOMING, &err);

	error |= epos4_write_i32(epos4, EPOS4_HOME_OFFSET, config.home_offset, &err);

	error |= epos4_write_i8(epos4, EPOS4_HOMING_METHOD, config.method, &err);

	return error;
}

EPOS4_ERROR_t epos4_hom_move(EPOS4_INST_t * epos4) {
	EPOS4_ERROR_t error = 0;
	uint32_t err;

	error |= epos4_control_shutdown(epos4, &err);

	error |= epos4_control_soenable(epos4, &err);

	error |= epos4_control_hom_start(epos4, &err);

	return error;
}

EPOS4_ERROR_t epos4_hom_terminate(EPOS4_INST_t * epos4, uint8_t * terminated) {
	EPOS4_ERROR_t error = 0;
	uint32_t err;
	*terminated = 0;

	uint16_t status;

	error |= epos4_read_statusword(epos4, &status, &err);


	if(!EPOS4_SW_HOMING_ATTAINED(status)) {
		epos4_control_disable(epos4, &err);
		*terminated = 1;
	}

	return error;
}

EPOS4_ERROR_t epos4_recover(EPOS4_INST_t * epos4) {
	EPOS4_ERROR_t error = 0;
	uint32_t err;

	error |= epos4_control_fault_rst(epos4, &err);


	return error;
}


/* END */


