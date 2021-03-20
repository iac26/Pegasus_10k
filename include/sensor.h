/*  Title       : Sensor
 *  Filename    : sensor.h
 *  Author      : iacopo sprenger
 *  Date        : 28.01.2021
 *  Version     : 0.1
 *  Description : Sensor acquisition and processing
 */

#ifndef SENSOR_H
#define SENSOR_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

/**********************
 *  CONSTANTS
 **********************/


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

typedef struct SENSOR_DATA{
	int32_t pressure_1;
	int32_t pressure_2;
	int32_t temperature[3];
	uint8_t temperature_valid[3];
	uint32_t time;
}SENSOR_DATA_t;


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

void sensor_calib(void);

uint8_t sensor_calib_done(void);

SENSOR_DATA_t sensor_get_last(void);
SENSOR_DATA_t sensor_get_last_bfr(uint8_t n);

uint8_t sensor_new_data_storage(void);
uint8_t sensor_new_data_can(void);

void sensor_start(tprio_t prio);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* SENSOR_H */

/* END */
