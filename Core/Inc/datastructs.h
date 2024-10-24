#ifndef DATASTRUCTS_H
#define DATASTRUCTS_H

#include <stdbool.h>
#include <stdint.h>
#include "bmsConfig.h"
#include "timer.h"
#include "cmsis_os2.h"

/**
 * @brief Individual chip data
 * @note stores thermistor values, voltage readings, and the discharge status
 */
typedef struct {
	/* These are retrieved from the initial LTC comms */
	uint16_t voltage
		[NUM_CELLS_PER_CHIP]; /* store voltage readings from each chip */
	int8_t thermistor_reading
		[NUM_THERMS_PER_CHIP]; /* store all therm readings from each chip */
	int8_t thermistor_value[NUM_THERMS_PER_CHIP];
	int error_reading;

	/* These are calculated during the analysis of data */
	int8_t cell_temp[NUM_CELLS_PER_CHIP];
	float cell_resistance[NUM_CELLS_PER_CHIP];
	uint16_t open_cell_voltage[NUM_CELLS_PER_CHIP];

	uint8_t noise_reading
		[NUM_CELLS_PER_CHIP]; /* bool representing noise ignored read */
	uint8_t consecutive_noise
		[NUM_CELLS_PER_CHIP]; /* count representing consecutive noisy reads */
} chipdata_t;

/**
 * @brief Enuemrated possible fault codes for the BMS
 * @note  the values increase at powers of two to perform bitwise operations on a main fault code
 *          to set or get the error codes
 */
// clang-format off
typedef enum {
	FAULTS_CLEAR = 0x0,

	/* Orion BMS faults */
	CELLS_NOT_BALANCING		            = 0x1,
	CELL_VOLTAGE_TOO_HIGH	            = 0x4,
	CELL_VOLTAGE_TOO_LOW	            = 0x2,
	PACK_TOO_HOT			            = 0x8,
	OPEN_WIRING_FAULT		            = 0x10, /* cell tap wire is either weakly connected or not connected */
	INTERNAL_SOFTWARE_FAULT             = 0x20, /* general software fault */
	INTERNAL_THERMAL_ERROR              = 0x40, /* internal hardware fault reulting from too hot of onboard temps */
	INTERNAL_CELL_COMM_FAULT            = 0x80, /* this is due to an invalid CRC from retrieving values */
	CURRENT_SENSOR_FAULT	            = 0x100,
	CHARGE_READING_MISMATCH             = 0x200, /* charge voltage when not supposed to be charging*/
	LOW_CELL_VOLTAGE				    = 0x400, /* voltage of a cell falls below 90 mV */
	WEAK_PACK_FAULT					    = 0x800,
	EXTERNAL_CAN_FAULT				    = 0x1000,
	DISCHARGE_LIMIT_ENFORCEMENT_FAULT   = 0x2000,
	CHARGER_SAFETY_RELAY			    = 0x4000,
	BATTERY_THERMISTOR				    = 0x8000,
	CHARGER_CAN_FAULT				    = 0x10000,
	CHARGE_LIMIT_ENFORCEMENT_FAULT	    = 0x20000,

	MAX_FAULTS = 0x80000000 /* Maximum allowable fault code */
} bms_fault_t;
// clang-format on

/**
 * @brief Stores critical values for the pack, and where that critical value can be found
 *
 */
typedef struct {
	int32_t val;
	uint8_t chipIndex;
	uint8_t cellNum;
} crit_cellval_t;

/**
 * @brief Represents one "frame" of BMS data
 * @note the size of this structure is **9752 bits** (~1.3k bytes), as of October 22, 2022
 */
#define ACCUMULATOR_FRAME_SIZE sizeof(acc_data_t);

typedef struct {
	/* Array of data from all chips in the system */
	chipdata_t chip_data[NUM_CHIPS];

	int fault_status;

	int16_t pack_current; /* this value is multiplied by 10 to account for decimal precision */
	uint16_t pack_voltage;
	uint16_t pack_ocv;
	uint16_t pack_res;

	uint16_t discharge_limit;
	uint16_t charge_limit;
	uint16_t cont_DCL;
	uint16_t cont_CCL;
	uint8_t soc;

	int8_t segment_average_temps[NUM_SEGMENTS];
	uint8_t segment_noise_percentage[NUM_SEGMENTS];

	/**
	 * @brief Note that this is a 32 bit integer, so there are 32 max possible fault codes
	 */
	uint32_t fault_code;

	/* Max, min, and avg thermistor readings */
	crit_cellval_t max_temp;
	crit_cellval_t min_temp;
	int8_t avg_temp;

	/* Max and min cell resistances */
	crit_cellval_t max_res;
	crit_cellval_t min_res;

	/* Max, min, and avg voltage of the cells */
	crit_cellval_t max_voltage;
	crit_cellval_t min_voltage;
	uint16_t avg_voltage;
	uint16_t delt_voltage;

	crit_cellval_t max_ocv;
	crit_cellval_t min_ocv;
	uint16_t avg_ocv;
	uint16_t delt_ocv;

	uint16_t boost_setting;

	bool is_charger_connected;

	osMutexId_t mutex;
} acc_data_t;

/**
 * @brief Represents individual BMS states
 */
typedef enum {
	BOOT_STATE, /* State when BMS first starts up, used to initialize everything that needs
					   configuring */
	READY_STATE, /* State when car is not on/BMS is not really doing anything */
	CHARGING_STATE, /* State when car is on and is charging (Filling battery) */
	FAULTED_STATE, /* State when BMS has detected a catastrophic fault and we need to hault
					   operations */
	NUM_STATES

} BMSState_t;

/**
 * @brief Represents fault evaluation operators
 */
typedef enum {
	GT, /* fault if {data} greater than {threshold}             */
	LT, /* fault if {data} less than {threshold}                */
	GE, /* fault if {data} greater than or equal to {threshold} */
	LE, /* fault if {data} less than or equal to {threshold}    */
	EQ, /* fault if {data} equal to {threshold}                 */
	NEQ, /* fault if {data} not equal to {threshold}             */
	NOP /* no operation, use for single threshold faults        */

} fault_evalop_t;

/**
 * @brief Represents data to be packaged into a fault evaluation
 */
typedef struct {
	char *id;
	nertimer_t timer;

	int data_1;
	fault_evalop_t optype_1;
	int lim_1;

	int timeout;
	int code;

	fault_evalop_t optype_2;
	int data_2;
	int lim_2;

	bool is_faulted;
} fault_eval_t;

#endif