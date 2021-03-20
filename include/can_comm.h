/*  Title       : CAN communication
 *  Filename    : can.h
 *  Author      : iacopo sprenger
 *  Date        : 03.03.2021
 *  Version     : 0.1
 *  Description : can communication
 */

#ifndef CAN_COMM_H
#define CAN_COMM_H



/**********************
 *  INCLUDES
 **********************/

#include <string.h>

/**********************
 *  CONSTANTS
 **********************/

// Define all the data ID's
#define DATA_ID_PRESSURE 0 // hPa
#define DATA_ID_ACCELERATION_X 1 // milli-g
#define DATA_ID_ACCELERATION_Y 2 // milli-g
#define DATA_ID_ACCELERATION_Z 3 // milli-g
#define DATA_ID_GYRO_X 4 // mrps
#define DATA_ID_GYRO_Y 5 // mrps
#define DATA_ID_GYRO_Z 6 // mrps

#define DATA_ID_GPS_HDOP      7 // mm
#define DATA_ID_GPS_LAT       8 // udeg
#define DATA_ID_GPS_LONG      9 // udeg
#define DATA_ID_GPS_ALTITUDE 10 // cm
#define DATA_ID_GPS_SATS     11 // #

#define DATA_ID_TEMPERATURE 12 // cDegC
#define DATA_ID_CALIB_PRESSURE 13 // Pa

#define DATA_ID_AB_STATE   16 // enum
#define DATA_ID_AB_INC     17 // [-]
#define DATA_ID_AB_AIRSPEED 18 // mm/s
#define DATA_ID_AB_ALT     19 // m

#define DATA_ID_KALMAN_STATE 38 // enum
#define DATA_ID_KALMAN_X     40 // m
#define DATA_ID_KALMAN_Y     41 // m
#define DATA_ID_KALMAN_Z     42 // m
#define DATA_ID_KALMAN_VX    43 // mm/s
#define DATA_ID_KALMAN_VY    44 // mm/s
#define DATA_ID_KALMAN_VZ    45 // mm/s
#define DATA_ID_KALMAN_YAW   46 // mrad
#define DATA_ID_KALMAN_PITCH 47 // mrad
#define DATA_ID_KALMAN_ROLL  48 // mrad

#define DATA_ID_STATE 50 // enum

//TODO which DATA_ID is the correct one for the ignition?

//GSE Rx Data
#define DATA_ID_IGNITION 51
#define DATA_ID_ORDER 52
#define DATA_ID_GST_CODE 53

//GSE Tx Data
#define DATA_ID_GSE_CODE 54
#define DATA_ID_FILL_VALVE_STATE 56
#define DATA_ID_PURGE_VALVE_STATE 57
#define DATA_ID_HOSE_DISCONNECT_STATE 58
#define DATA_ID_MAIN_IGNITION_STATE 59
#define DATA_ID_SEC_IGNITION_STATE 60



//Propulsion Data
#define DATA_COMMAND_CHECK_VALUE	0xC0FFEE

#define DATA_ID_OPERATION	    80
#define DATA_ID_ARMING			81
#define DATA_ID_VENTING			82
#define DATA_ID_CALIBRATION		83
#define DATA_ID_ABORT			84

#define DATA_ID_PRESS_1			85
#define DATA_ID_PRESS_2			86
#define DATA_ID_TEMP_1			87
#define DATA_ID_TEMP_2			88
#define DATA_ID_TEMP_3			89
#define DATA_ID_STATUS			90
#define DATA_ID_MOT_POS			91


// Define all the board ID's (lower means higher priority for CAN protocol)
#define CAN_ID_MAIN_BOARD 0
#define CAN_ID_GPS_BOARD 1
#define CAN_ID_TELEMETRY_BOARD 2
#define CAN_ID_AIBRAKE_BOARD 3
#define CAN_ID_DEBUG_BOARD 6
#define CAN_ID_DEFAULT 7
#define CAN_ID_PROPULSION_BOARD 5
#define CAN_ID_VALVE_BOARD 8
#define CAN_ID_CODE_BOARD 10
#define CAN_ID_SENSOR_TELEMETRY_BOARD 9

#define MAX_BOARD_ID 9 // used to implement redundant info in CAN_handling
#define MAX_BOARD_NUMBER (MAX_BOARD_ID+1)

#define DATA_MAGIC_1		0xC0FEBABE
#define DATA_MAGIC_2		0xBEEFBEEF
#define DATA_MAGIC_3		0xDEADDEAD

/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

typedef struct
{
    uint32_t data;
    uint8_t id;
    uint32_t timestamp;
    uint32_t id_CAN;
} CAN_msg;


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif



void CAN_Config(uint32_t id);
void can_setFrame(uint32_t data, uint8_t data_id, uint32_t timestamp);

uint32_t can_msgPending(void);
CAN_msg can_readBuffer(void);

void can_send_thread(void * arg);





#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* CAN_COMM_H */

/* END */
