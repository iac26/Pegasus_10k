/*  Title		: Sensor
 *  Filename	: sensor.c
 *	Author		: iacopo sprenger
 *	Date		: 28.01.2021
 *	Version		: 0.1
 *	Description	: Sensor acquisition and processing
 */

/**********************
 *	INCLUDES
 **********************/

#include <ch.h>
#include <hal.h>

#include <sensor.h>
#include <lut.h>
#include <util.h>

/**********************
 *	CONFIGURATION
 **********************/

#define SENSOR_TIMER	GPTD3
#define SENSOR_ADC		ADCD1

#define SENSOR_PORT_TEMP_1	GPIOA
#define SENSOR_PORT_TEMP_2	GPIOA
#define SENSOR_PORT_TEMP_3	GPIOA
#define SENSOR_PORT_PRES_1	GPIOC
#define SENSOR_PORT_PRES_2	GPIOA

#define SENSOR_PIN_TEMP_1	0
#define SENSOR_PIN_TEMP_2	1
#define SENSOR_PIN_TEMP_3	3
#define SENSOR_PIN_PRES_1	0
#define SENSOR_PIN_PRES_2	2

#define SENSOR_CH_TEMP_1	ADC_CHANNEL_IN0
#define SENSOR_CH_TEMP_2	ADC_CHANNEL_IN1
#define SENSOR_CH_TEMP_3	ADC_CHANNEL_IN3
#define SENSOR_CH_PRES_1	ADC_CHANNEL_IN10
#define SENSOR_CH_PRES_2	ADC_CHANNEL_IN2

#define SENSOR_HEART_BEAT 1  /* ms */

#define ADC_HEART_BEAT 0.05  /* ms */

/**********************
 *	CONSTANTS
 **********************/



#define ADC_GRP_BUF_DEPTH 1
#define ADC_GRP_NUM_CHANNELS 5



#define DT_THRESH		500 /* 0.1deg */

#define SAMPLING_TIME 	20 //ms

#define NB_SAMPLES		32

#define SAMPLE_BUFFER_LEN	(256)

#define FILTER_BUFFER_LEN	(16)

#define SEND_RATE	20

#define CALIBRATION_CYCLES		(64)


#define ADC_TIMER_FREQ			(10000000)
//this needs to be done correctly
#define MS_2_SENSOR_TIMER(ms)	ADC_TIMER_FREQ*(ms)/1000

/*
 * KULITE CALIBRATION DATA
 */
#define R1_VAL	1799
#define R2_VAL	4700

#define R3_VAL	1798
#define R4_VAL	4700

#define KULITE_322_CAL			64341	//uV/bar
#define KULITE_322_DECODE(val)	((int64_t)(val) * (R3_VAL+R4_VAL) * 3300 * 1000000 / KULITE_322_CAL / 4096 / R4_VAL)

#define KULITE_323_CAL			64271	//uV/bar
#define KULITE_323_DECODE(val)	((int64_t)(val) * (R1_VAL+R2_VAL) * 3300 * 1000000 / KULITE_323_CAL / 4096 / R2_VAL)


#define FILTER_LEN	5



/**********************
 *	MACROS
 **********************/

/*
 * we generate the following functions:
 * static inline util_buffer_SENSOR_init(UTIL_BUFFER_SENSOR_t * bfr, SENSOR_DATA_t * buffer, uint16_t bfr_len)
 * static inline util_buffer_SENSOR_add(UTIL_BUFFER_SENSOR_t * bfr, SENSOR_DATA_t d)
 * static inline SENSOR_DATA_t util_buffer_SENSOR_get(UTIL_BUFFER_SENSOR_t * bfr)
 * static inline SENSOR_DATA_t util_buffer_SENSOR_access(UTIL_BUFFER_SENSOR_t * bfr, uint16_t ix)
 * static inline uint8_t util_buffer_SENSOR_isempty(UTIL_BUFFER_SENSOR_t * bfr)
 */
UTIL_GENERATE_BUFFER(SENSOR_DATA_t, SENSOR)



/**********************
 *	TYPEDEFS
 **********************/

typedef enum SENSOR_TYPE{ //Same order as adc in ioc
	PRESSURE_1, 		// PC0 | A0 S2  m2  tank
	PRESSURE_2, 		// PA2 | A1 S2  m1  cc
	TEMPERATURE_1, 		// PA0 | D0 S1
	TEMPERATURE_2, 		// PA1 | D1 S1
	TEMPERATURE_3,		// PA3 | A0 S1
	NB_SENSOR
}SENSOR_TYPE_t;



/**********************
 *	VARIABLES
 **********************/

static adcsample_t adc_buffer[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP_NUM_CHANNELS * ADC_GRP_BUF_DEPTH)];

static UTIL_BUFFER_SENSOR_t sample_bfr;
static SENSOR_DATA_t sample_buffer[SAMPLE_BUFFER_LEN];
static UTIL_BUFFER_SENSOR_t filter_bfr;
static SENSOR_DATA_t filter_buffer[FILTER_BUFFER_LEN] = {0};

static SENSOR_DATA_t last_data = {0};

static SENSOR_DATA_t offset = {0};

static uint8_t calib = 0;
static uint16_t calib_counter = 0;

static uint8_t new_data_storage;
static uint8_t new_data_can;

/**********************
 *	PROTOTYPES
 **********************/

static void sensor_init(void);
void adc_callback(ADCDriver *adcp);
static void adc_error_callback(ADCDriver *adcp, adcerror_t err);



/**********************
 *	DECLARATIONS
 **********************/

const ADCConfig adc_conf = {

};

const ADCConversionGroup adc_conv_conf = {
  .circular     = TRUE,
  .num_channels = ADC_GRP_NUM_CHANNELS,
  .end_cb       = adc_callback,
  .error_cb     = adc_error_callback,
  .cr1          = 0U,
  .cr2          = ADC_CR2_EXTEN_RISING |
				  ADC_CR2_EXTSEL_SRC(0b1000),  /* TIM3_TRGO */
  .smpr1 		= ADC_SMPR1_SMP_AN10(ADC_SAMPLE_15),	/* PRES_1 */
  .smpr2 		= ADC_SMPR2_SMP_AN0(ADC_SAMPLE_15) |	/* TEMP_1 */
				  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_15) |	/* TEMP_2 */
				  ADC_SMPR2_SMP_AN2(ADC_SAMPLE_15) |	/* PRES_2 */
				  ADC_SMPR2_SMP_AN3(ADC_SAMPLE_15),		/* TEMP_3 */

  .ltr 			= 0,
  .htr 			= 0xfff,

  .sqr1			= ADC_SQR1_NUM_CH(ADC_GRP_NUM_CHANNELS),
  .sqr2			= 0U,
  .sqr3			= ADC_SQR3_SQ1_N(SENSOR_CH_TEMP_1) |
				  ADC_SQR3_SQ2_N(SENSOR_CH_TEMP_2) |
				  ADC_SQR3_SQ3_N(SENSOR_CH_TEMP_3) |
				  ADC_SQR3_SQ4_N(SENSOR_CH_PRES_1) |
				  ADC_SQR3_SQ5_N(SENSOR_CH_PRES_2),


};

const GPTConfig gpt_conf = {
	.frequency    =  ADC_TIMER_FREQ,
	.callback     =  NULL,
	.cr2          =  (0b10<<4U),   /* MMS = 010 = TRGO on Update Event.    */
	.dier         =  0U
};


/*
 * ADC ISR
 */
void adc_callback(ADCDriver *adcp) {
	if(adcp == &SENSOR_ADC) {
		//put samples into a fifo buffer
		//samples are preprocessed
		SENSOR_DATA_t data;
		data.pressure_1 = KULITE_322_DECODE(adc_buffer[PRESSURE_1])-offset.pressure_1;
		data.pressure_2 = KULITE_323_DECODE(adc_buffer[PRESSURE_2])-offset.pressure_2;

		if(adc_buffer[TEMPERATURE_1] >= LUT_TEMP_MIN && adc_buffer[TEMPERATURE_1] < LUT_TEMP_MAX) {
				data.temperature[0] = lut_temp[adc_buffer[TEMPERATURE_1]-LUT_TEMP_MIN];
				data.temperature_valid[0] = 1;
		} else {
			data.temperature_valid[0] = 0;
		}
		if(adc_buffer[TEMPERATURE_2] >= LUT_TEMP_MIN && adc_buffer[TEMPERATURE_2] < LUT_TEMP_MAX) {
				data.temperature[1] = lut_temp[adc_buffer[TEMPERATURE_2]-LUT_TEMP_MIN];
				data.temperature_valid[1] = 1;
		} else {
			data.temperature_valid[1] = 0;
		}
		if(adc_buffer[TEMPERATURE_3] >= LUT_TEMP_MIN && adc_buffer[TEMPERATURE_3] < LUT_TEMP_MAX) {
				data.temperature[2] = lut_temp[adc_buffer[TEMPERATURE_3]-LUT_TEMP_MIN];
				data.temperature_valid[2] = 1;
		} else {
			data.temperature_valid[2] = 0;
		}
		util_buffer_SENSOR_add(&sample_bfr, data);
	}
}

static void adc_error_callback(ADCDriver *adcp, adcerror_t err) {
	if(adcp == &SENSOR_ADC) {
		(void) err;
		//Do something in case of error
	}
}



static void sensor_init(void) {
	//PIN SETUP done with the cfg tool

	/*
	palSetGroupMode(GPIOA, PAL_PORT_BIT(SENSOR_PIN_TEMP_1) | PAL_PORT_BIT(SENSOR_PIN_TEMP_2) |
					PAL_PORT_BIT(SENSOR_PIN_TEMP_3) | PAL_PORT_BIT(SENSOR_PIN_PRES_2),
	                0, PAL_MODE_INPUT_ANALOG);
	palSetGroupMode(GPIOC, PAL_PORT_BIT(SENSOR_PIN_PRES_1),
		           	0, PAL_MODE_INPUT_ANALOG);
	*/



	//Init fifo buffers
	util_buffer_SENSOR_init(&sample_bfr, sample_buffer, SAMPLE_BUFFER_LEN);
	util_buffer_SENSOR_init(&filter_bfr, filter_buffer, FILTER_BUFFER_LEN);

	adcStart(&SENSOR_ADC, &adc_conf);

	gptStart(&SENSOR_TIMER, &gpt_conf);

	adcStartConversion(&SENSOR_ADC, &adc_conv_conf, adc_buffer, ADC_GRP_BUF_DEPTH);

	gptStartContinuous(&SENSOR_TIMER, MS_2_SENSOR_TIMER(ADC_HEART_BEAT));




}

void sensor_calib(void) {
	calib_counter = 0;
	calib = 1;
}

uint8_t sensor_calib_done(void) {
	return !calib;
}

SENSOR_DATA_t sensor_get_last(void) {
	return last_data;
}

SENSOR_DATA_t sensor_get_last_bfr(uint8_t n) {
	return util_buffer_SENSOR_access(&filter_bfr, n);
}

uint8_t sensor_new_data_storage() {
	uint8_t tmp = new_data_storage;
	new_data_storage = 0;
	return tmp;
}

uint8_t sensor_new_data_can() {
	uint8_t tmp = new_data_can;
	new_data_can = 0;
	return tmp;
}

static THD_WORKING_AREA(sensor_thread_wa, 1024);
static THD_FUNCTION(sensor_thread, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);
	//perform averaging on the fifo contents
	//perform data processing (Kalman??)
	//save data to internal storage (release semaphore for the storage thread)
	//send data through CAN --> here or another thread..

	sensor_init();

	systime_t thread_time = chVTGetSystemTime();

	for(;;) {



		//TIMING TEST
		//HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);

		//second processing i.e. empty buffer and average all samples
		//trend prediction
		//average value of each sample collected in the fifo buffer
		volatile uint16_t temp_count[3] = {0};
		volatile int32_t temp_val[3] = {0};
		volatile uint16_t pres_count = 0;
		volatile int32_t pres_val[2] = {0};
		while(!util_buffer_SENSOR_isempty(&sample_bfr)) {
			volatile SENSOR_DATA_t data = util_buffer_SENSOR_get(&sample_bfr);
			pres_val[0] += data.pressure_1;
			pres_val[1] += data.pressure_2;
			pres_count += 1;
			temp_val[0] += data.temperature_valid[0]?data.temperature[0]:0;
			temp_count[0] += data.temperature_valid[0];
			temp_val[1] += data.temperature_valid[1]?data.temperature[1]:0;
			temp_count[1] += data.temperature_valid[1];
			temp_val[2] += data.temperature_valid[2]?data.temperature[2]:0;
			temp_count[2] += data.temperature_valid[2];
		}
		if(pres_count != 0) {
			pres_val[0] /= pres_count;
			pres_val[1] /= pres_count;
		}else{
			//pressure error
		}
		if(temp_count[0] != 0) {
			temp_val[0] /= temp_count[0];
		} else {
			//temp 1 error
		}
		if(temp_count[1] != 0) {
			temp_val[1] /= temp_count[1];
		} else {
			//temp 2 error
		}
		if(temp_count[2] != 0) {
			temp_val[2] /= temp_count[2];
		} else {
			//temp 3 error
		}
		volatile SENSOR_DATA_t data = {
				pres_val[0],
				pres_val[1],
				{
						temp_val[0],
						temp_val[1],
						temp_val[2]
				},
				{
					0, 0, 0
				},
				0
		};

		data.time = TIME_I2MS(chVTGetSystemTime());



		// this is while I build the filtering functions
		static volatile uint16_t h = 0;
		static volatile int32_t d_temperature[3] = {0};

		h = data.time - last_data.time;
		if(h) {
			for(uint8_t i = 0; i < 3; i++) {
				int32_t next = last_data.temperature[i] + d_temperature[i]*h;
				if(util_abs(data.temperature[i])>DT_THRESH) {
					data.temperature[i] = next;
				}
				//compute next
				d_temperature[i] = (data.temperature[i] - last_data.temperature[i])/h;
			}
		}

		util_buffer_SENSOR_add(&filter_bfr, data);
		last_data = data;

		//Predictor and wrong measurements rejector (Temperature only)

		//predict next state according to the derivative
		//if measurement is too far, use an estimation from the derivative and the time to get the next measurement

		//CALIBRATION (PRESSURE)
		static uint32_t pressure_1 = 0;
		static uint32_t pressure_2 = 0;
		if(calib) {
			if(calib_counter == 0) {
				offset.pressure_1 = 0;
				offset.pressure_2 = 0;
				pressure_1 = 0;
				pressure_2 = 0;
			} else {
				pressure_1 += data.pressure_1;
				pressure_2 += data.pressure_2;
				if(calib_counter >= CALIBRATION_CYCLES) {
					offset.pressure_1 = pressure_1 / CALIBRATION_CYCLES;
					offset.pressure_2 = pressure_2 / CALIBRATION_CYCLES;
					calib = 0;
					calib_counter = 0;
				}
			}
			calib_counter++;
		}


		//The CAN thread will periodically get the data
		new_data_can = 1;

		//The storage thread will periodically get the data
		new_data_storage = 1;

		//Results accessible from DEBUG UART

		thread_time = chThdSleepUntilWindowed(thread_time, thread_time+TIME_MS2I(SENSOR_HEART_BEAT));
	}
}

void sensor_start(tprio_t prio) {
	chThdCreateStatic(sensor_thread_wa, sizeof(sensor_thread_wa), prio, sensor_thread, NULL);
}



/* END */


