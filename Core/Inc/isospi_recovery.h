#ifndef ISOSPI_RECOVERY_H
#define ISOSPI_RECOVERY_H

#include "datastructs.h"

/**
 * @brief Checks whether the isoSPI startup mask period is still active.
 *
 * @return 1 if the startup mask is active (PEC faults should be ignored), 0 otherwise.
 */
int is_startup_mask_active(void);

/**
 * @brief Initializes ISO SPI break detection timers and state.
 *
 * Should be called during system startup or segment re-initialization.
 *
 * @param bmsdata Pointer to the accumulator data structure.
 */
void isospi_break_detection_init(acc_data_t *bmsdata);

/**
 * @brief Dispatches logic based on current ISO SPI communication state.
 *
 * Handles transitions between NORMAL, BREAK_DETECTED, RECOVERY_SUCCESS, and RECOVERY_FAILED states.
 *
 * @param isospi_state Current communication state.
 * @param bmsdata Pointer to accumulator data structure.
 */
void isospi_state_dispatcher(acc_data_t *bmsdata);

#endif // ISOSPI_RECOVERY_H