#ifndef ISOSPI_RECOVERY_H
#define ISOSPI_RECOVERY_H

#include "datastructs.h"

/**
 * @brief Initializes ISO SPI break detection timers and state.
 *
 * Should be called during system startup or segment re-initialization.
 *
 * @param bmsdata Pointer to the accumulator data structure.
 */
void isospi_break_detection_init(acc_data_t *bmsdata);

/**
 * @brief Checks for ISO SPI communication break using PEC error tracking.
 *
 * If a break is detected based on PEC thresholds, the internal state is updated and
 * a non-critical fault is flagged. Resets counters after each check.
 *
 * @param bmsdata Pointer to accumulator data structure.
 */
void detect_isospi_break(acc_data_t *bmsdata);

/**
 * @brief Attempts to recover from an ISO SPI break by switching to the secondary line.
 *
 * Updates chip direction, sets COMM_BK, writes configs, and verifies recovery by checking PEC errors.
 *
 * @param bmsdata Pointer to the accumulator data structure.
 * @return int 1 if recovery was successful, 0 otherwise.
 */
int32_t attempt_isospi_recovery(acc_data_t *bmsdata);

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