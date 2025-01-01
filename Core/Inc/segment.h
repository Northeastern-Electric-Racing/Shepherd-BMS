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
 * @todo make sure that retrieving cell data doesn't block code too much
 */
void segment_retrieve_data(acc_data_t *bmsdata);

/**
 * @brief Disables balancing for all cells.
 *
 * @param chips Array of ADBMS6830 data structs.
 */
void segment_disable_balancing(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Set the cell balancing configuration and send it to the segments.
 * 
 * @param chips Array of ADBMS6830 data structs.
 * @param discharge_config Configuration for which cells to discharge.
 */
void segment_configure_balancing(
	cell_asic chips[NUM_CHIPS],
	bool discharge_config[NUM_CHIPS][NUM_CELLS_PER_CHIP]);

/**
 * @brief Returns if a specific cell is balancing
 *
 * @param chip_num
 * @return true
 * @return false
 */
bool cell_is_balancing(uint8_t chip_num, uint8_t cell_num);

/**
 * @brief Returns if any cells are balancing.
 * 
 * @param chips Array of ADBMS6830 chips.
 * @return true 
 * @return false 
 */
bool segment_is_balancing(cell_asic chips[NUM_CHIPS]);

#endif