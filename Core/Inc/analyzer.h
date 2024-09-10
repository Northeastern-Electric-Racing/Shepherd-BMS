#ifndef ANALYZER_H
#define ANALYZER_H

//#include <nerduino.h> Replace
#include "datastructs.h"
#include "segment.h"

// this is a simple empirical mapping of which therms are returning good data
// Only in use because we can not seem to correclty map incoming therms - this allows us to forcibley select those that we visually noticed were good
extern uint8_t THERM_DISABLE[NUM_CHIPS][NUM_THERMS_PER_CHIP];

extern const uint8_t NO_THERM;
extern const uint8_t MUX_OFFSET;

/**
 * @brief Mapping the Relevant Thermistors for each cell based on cell #
 * @note 0xFF indicates the end of the relevant therms
 * @note Low side
 */
extern const uint8_t RELEVANT_THERM_MAP_L[NUM_CELLS_PER_CHIP]
					 [NUM_RELEVANT_THERMS];

/**
 * @brief Mapping the Relevant Thermistors for each cell based on cell #
 * @note 0xFF indicates the end of the relevant therms
 * @note High side
 */
extern const uint8_t RELEVANT_THERM_MAP_H[NUM_CELLS_PER_CHIP]
					 [NUM_RELEVANT_THERMS];

/*
 * List of therms that we actually read from, NOT reordered by cell
 */
extern const uint8_t POPULATED_THERM_LIST_L[NUM_THERMS_PER_CHIP];

/*
 * List of therms that we actually read from, NOT reordered by cell
 */
extern const uint8_t POPULATED_THERM_LIST_H[NUM_THERMS_PER_CHIP];

/* We want to make sure we aren't doing useless analysis on the same set of data since we are
 * backfilling segment data */
#define ANALYSIS_INTERVAL VOLTAGE_WAIT_TIME

//#define MAX_SIZE_OF_HIST_QUEUE  300000U //bytes

/**
 * @brief Calculate thermistor values and cell temps using thermistors.
 * 
 */
void calc_cell_temps(acc_data_t *bmsdata);

/**
 * @brief Calculates pack temp, and avg, min, and max cell temperatures.
 * 
 */
void calc_pack_temps(acc_data_t *bmsdata);

/**
 * @brief Calculate statistics about pack voltage, such as min and max cell volt, pack and avg voltage, pack and avg OCV, and deltas.
 * 
 */
void calc_pack_voltage_stats(acc_data_t *bmsdata);

/**
 * @brief Calculate open cell voltages based on cell voltages and previous open cell voltages.
 * 
 */
void calc_open_cell_voltage(acc_data_t *bmsdata);

/**
 * @brief Calculate cell resistances using cell temperatures and a cell temp to resistance lookup table.
 * 
 */
void calc_cell_resistances(acc_data_t *bmsdata);

/**
 * @brief Calculate the discharge current limit based on OCV and cell resistance, and ensure that it is stable while driving.
 * 
 */
void calc_dcl(acc_data_t *bmsdata);

/**
 * @brief Calculate the continuous discharge current limit based on cell temperatures and a cell temp to DCL lookup table.
 * 
 */
void calc_cont_dcl(acc_data_t *bmsdata);

/**
 * @brief Calculate the continuous charging current limit based on cell temperatures and a cell temp to CCL lookup table.
 * 
 */
void calc_cont_ccl(acc_data_t *bmsdata);

/**
 * @brief Disable unreliable thermistors.
 * 
 */
void disable_therms(acc_data_t *bmsdata);

/**
 * @brief Calcuate state of charge by interpolation using an OCV to SOC lookup table.
 * 
 */
void calc_state_of_charge(acc_data_t *bmsdata);

/**
 * @brief Calculate the percent of voltage readings that are noisy.
 * 
 */
void calc_noise_volt_percent(acc_data_t *bmsdata);

/**
 * @brief Pushes in a new data point if we have waited long enough.
 *
 * @param data
 */
void analyzer_push(acc_data_t *data);

/**
 * @brief Calculates the PWM required to drive the fans at the current moment in time.
 *
 * @return uint8_t PWM duty cycle from 0-255
 */
uint8_t analyzer_calc_fan_pwm();

#endif