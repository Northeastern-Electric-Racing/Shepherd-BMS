#ifndef SEGMENT_H
#define SEGMENT_H

#include "bmsConfig.h"
#include "datastructs.h"

/**
 * @brief Initializes the segments
 */
void segment_init(acc_data_t *bmsdata);

/**
 * @brief Pulls all cell data from the segments and returns all cell data
 *
 */
void segment_retrieve_data(acc_data_t *bmsdata);

/**
 * @brief Fetch extra data for segment
 * 
 */
void segment_retrieve_debug_data(acc_data_t *bmsdata);

/**
 * @brief Disables balancing for all cells.
 *
 * @param chips Array of ADBMS6830 data structs.
 */
void segment_disable_balancing(acc_data_t *bmsdata);

/**
 * @brief Configure which cells should discharge, and send configuration to ICs.
 * 
 * @param bmsdata Pointer to acc data struct.
 * @param discharge_config Array containing the discharge configuration. true = discharge, false = do not discharge.
 */
void segment_configure_balancing(
	acc_data_t *bmsdata, bool discharge_config[NUM_CHIPS][NUM_CELLS_ALPHA]);

/**
 * @brief Returns if any cells are balancing.
 * 
 * @param chips Array of ADBMS6830 chips.
 * @return true 
 * @return false 
 */
bool segment_is_balancing(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Do a single shot, redundant C-ADC measurement and read
 * the contents of Status Register Group C, which contains the 
 * CSxFLT bits indicating whether the difference between the 
 * C and S ADC measurements was above the CTH[2:0] set in config
 * register A.
 * 
 * @param chips Pointer to accumulator data struct.
 */
void get_adc_comparison(acc_data_t *bmsdata);

/**
 * @brief Read the serial ID of the chip.
 * 
 * @param chips Array of chips to read.
 */
void read_serial_id(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Read voltages in every register connected to AUX2 ADC.
 * 
 * @param chips Array of chips to get voltages of.
 */
void adc_and_read_aux2_registers(cell_asic chips[NUM_CHIPS]);

#endif