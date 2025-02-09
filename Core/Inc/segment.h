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
 * @brief Reset, then wake, then re-configure all chips
 * 
 * @param bmsdata 
 */
void segment_restart(acc_data_t *bmsdata);

/**
 * @brief Returns if any cells are balancing.
 * 
 * @param chips Array of ADBMS6830 chips.
 * @return true 
 * @return false 
 */
bool segment_is_balancing(cell_asic chips[NUM_CHIPS]);

#endif