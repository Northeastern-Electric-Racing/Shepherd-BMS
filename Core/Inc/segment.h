#ifndef SEGMENT_H
#define SEGMENT_H

#include "bmsConfig.h"
#include "stm32f4xx_hal.h"
#include "adBms6830Data.h"
#include <stdbool.h>

/**
 * @brief Initialize chips with default values.
 * 
 */
void segment_init(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi);
/**
 * @brief Stop discharge quickly
 * 
 * @param bmsdata 
 */
void segment_mute(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi);
/**
 * @brief Start discharge again (inherits config)
 * 
 * @param bmsdata 
 */
void segment_unmute(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi);

/**
 * @brief Freeze result registers
 * 
 * @param bmsdata 
 */
void segment_snap(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi);
/**
 * @brief Unfreeze result registers
 * 
 * @param bmsdata 
 */
void segment_unsnap(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi);

/**
 * @brief Pulls all cell data from the segments and returns all cell data.  For drive mode.
 *
 */
void segment_retrieve_active_data(cell_asic chips[NUM_CHIPS],
				  SPI_HandleTypeDef *hspi);
/**
 * @brief Pulls all cell data from the segments and returns all cell data.  For charge mode.
 *
 */
void segment_retrieve_charging_data(cell_asic chips[NUM_CHIPS],
				    SPI_HandleTypeDef *hspi);

/**
 * @brief Fetch extra data for segment
 * 
 */
void segment_retrieve_debug_data(cell_asic chips[NUM_CHIPS],
				 SPI_HandleTypeDef *hspi);

/**
 * @brief Disables balancing for all cells.  Will also clear balancing setting.
 *
 */
void segment_disable_balancing(cell_asic chips[NUM_CHIPS],
			       SPI_HandleTypeDef *hspi);

/**
 * @brief Enable balancing (still need to configure it).  Identical to segment_unmute
 * 
 */
void segment_enable_balancing(cell_asic chips[NUM_CHIPS],
			      SPI_HandleTypeDef *hspi);

void segment_manual_balancing(cell_asic chips[NUM_CHIPS],
			      SPI_HandleTypeDef *hspi);

/**
 * @brief Configure which cells should discharge, and send configuration to ICs.  Does not enable the actual balancing
 * 
 * @param bmsdata Pointer to acc data struct.
 * @param discharge_config Array containing the discharge configuration. true = discharge, false = do not discharge.
 */
void segment_configure_balancing(
	cell_asic chips[NUM_CHIPS],
	PWM_DUTY discharge_config[NUM_CHIPS][NUM_CELLS_ALPHA],
	SPI_HandleTypeDef *hspi);

/**
 * @brief Returns if any cells are balancing. Must read back config register B and PWM registers.
 * 
 * Checks both the DCC full discharge bits and the PWM bits.  Does not check if the cell is in thermal shutdown or muted.
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
void segment_restart(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi);

/**
 * @brief Do a single shot, redundant C-ADC measurement and read
 * the contents of Status Register Group C, which contains the 
 * CSxFLT bits indicating whether the difference between the 
 * C and S ADC measurements was above the CTH[2:0] set in config
 * register A.
 * 
 * @param chips Pointer to accumulator data struct.
 */
void get_adc_comparison(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi);

/**
 * @brief Read the serial ID of the chip.
 * 
 * @param chips Array of chips to read.
 */
void read_serial_id(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi);

#endif