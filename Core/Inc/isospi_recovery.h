#ifndef ISOSPI_RECOVERY_H
#define ISOSPI_RECOVERY_H

#include "datastructs.h"

/**
 * @brief Checks whether the isoSPI startup mask period is still active.
 *
 * @return 1 if the startup mask is active (PECs should be ignored), 0 otherwise.
 */
int is_startup_pec_mask_active(void);

/**
 * @brief Initializes ISO SPI break detection timers and state.
 *
 * Should be called during system startup after segment initialization.
 *
 * @param bmsdata Pointer to the accumulator data structure.
 */
void isospi_break_detection_init(acc_data_t *bmsdata);

/**
 * @brief Manages the isoSPI communication state machine.
 *
 * Handles logic and transitions for states in @ref isospi_comm_state_t.
 *
 * @param bmsdata Pointer to accumulator data structure.
 * @param hspi    SPI handle used for isoSPI communication.
 */
void isospi_handle_state(acc_data_t *bmsdata, SPI_HandleTypeDef *hspi);

#endif // ISOSPI_RECOVERY_H