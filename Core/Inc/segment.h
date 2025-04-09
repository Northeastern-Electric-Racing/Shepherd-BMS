#ifndef SEGMENT_H
#define SEGMENT_H

#include "datastructs.h"

/**
 * @brief Initialize chips with default values.
 * 
 */
void segment_init(acc_data_t *bmsdata);

/**
 * @brief Freeze result registers
 * 
 * @param bmsdata 
 */
void segment_snap(acc_data_t *bmsdata);
/**
 * @brief Unfreeze result registers
 * 
 * @param bmsdata 
 */
void segment_unsnap(acc_data_t *bmsdata);

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
 * @brief Disables balancing for all cells.  Will also clear balancing setting.
 *
 */
void segment_disable_balancing(acc_data_t *bmsdata);

/**
 * @brief Enable balancing (still need to configure it)
 * 
 */
void segment_enable_balancing(acc_data_t *bmsdata);

/**
 * @brief Configure which cells should discharge, and send configuration to ICs.  Does not enable the actual balancing
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
 * @brief Reset, then wake, then re-configure all chips
 * 
 * @param bmsdata 
 */
void segment_restart(acc_data_t *bmsdata);

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