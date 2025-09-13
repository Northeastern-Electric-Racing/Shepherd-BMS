#include "isospi_recovery.h"
#include "adi_interaction.h"
#include "segment.h"
#include "bmsConfig.h"
#include "can_messages.h"
#include "timer.h"

/** @brief Break detect threshold.
 *  PEC errors > this value in the accumulation window indicate a break.
 */
#define ISOSPI_PEC_ERROR_THRESHOLD (25U)

/** @brief Threshold for accumulation timer.
 *  Set just above the PEC error sum noise level per cycle,
 *  so random noise doesn’t start the accumulation window.
 */
#define ISOSPI_PEC_ACCUM_START_THRESH (5U)

/** @brief Validation threshold during recovery.
 *  Maximum PEC errors allowed while verifying recovery success.
 *  Lower this value for stricter validation.
 */
#define ISOSPI_VALIDATION_THRESHOLD (5U)

/** @brief Startup mask time (ms).
 *  Time to ignore PECs after init to avoid false detections.
 */
#define ISOSPI_STARTUP_MASK_TIME (1500U)

/** @brief Accumulation window (ms).
 *  For accumulation, the PEC sum updates at the ADBMS system-wide sample rate
 *  defined in bmsConfig.h.
 *  Observed PECs/run for chips with break: ~9 (discharge_state), ~20 (charge_state)
 *  Current: 2 Hz -> 500 ms * 8 runs = 4000 ms
 */
#define ISOSPI_ACCUM_PERIOD_MS (4000U)

/** @brief Maximum number of verification read attempts after recovery.
 *  Recovery passes if any attempt succeeds; fails if all attempts fail.
 */
#define ISOSPI_VERIFICATION_READS (3U)

/** 
 * @brief Timer to mask PEC faults during startup delay window 
 */
static nertimer_t startup_mask_timer;

/** 
 * @brief Timer to accumulate PEC errors before break detection 
 */
static nertimer_t pec_accum_timer;

/**
 * @brief Reset PEC error accumulators for all chips.
 *
 * Sets pec_error_sum to 0 for each chip in the provided array.
 *
 * @param chips Pointer to the array of cell_asic structures.
 */
static void reset_all_pec_error_sums(cell_asic chips[NUM_CHIPS])
{
	for (uint8_t i = 0; i < NUM_CHIPS; i++) {
		chips[i].pec_error_sum = 0U;
	}
}

/**
 * @brief Verifies whether isoSPI communication recovery succeeded.
 *
 * Performs multiple read cycles and checks if PEC errors have dropped
 * below the acceptable threshold for the chips after the detected break.
 *
 * @param bmsdata Pointer to accumulator data.
 * @param start_chip_idx Index of the chip where the break occurred.
 * @return 0 if all chips recovered successfully, 1 otherwise.
 */
static uint8_t isospi_verify_recovery(acc_data_t *bmsdata,
				      uint8_t start_chip_idx)
{
	uint8_t result = 0U;

	for (uint8_t i = start_chip_idx; i < NUM_CHIPS; i++) {
		if (bmsdata->chips[i].pec_error_sum >
		    ISOSPI_VALIDATION_THRESHOLD) {
			printf("[isoSPI] Verification failed at chip %u (PEC: %u)\n\r",
			       i + 1, bmsdata->chips[i].pec_error_sum);
			result = 1U;
		}
	}

	if (result == 0U) {
		printf("[isoSPI] Verification passed\n\r");
	}

	reset_all_pec_error_sums(bmsdata->chips);

	return result;
}

/**
 * @brief Checks for ISO SPI communication break using PEC error tracking.
 *
 * If a break is detected based on PEC thresholds, the internal state is updated and
 * a non-critical fault is flagged. Resets counters after each check.
 *
 * @param bmsdata Pointer to accumulator data structure.
 */
static void isospi_detect_break(acc_data_t *bmsdata)
{
	// Start accumulation timer on a spike in PEC errors
	if (!is_timer_active(&pec_accum_timer)) {
		uint8_t is_active = 0U;
		for (uint8_t i = 0U; (i < NUM_CHIPS) && (is_active == 0U);
		     i++) {
			if (bmsdata->chips[i].pec_error_sum >
			    ISOSPI_PEC_ACCUM_START_THRESH) {
				is_active = 1U;
			}
		}

		if (is_active == 1U) {
			start_timer(&pec_accum_timer, ISOSPI_ACCUM_PERIOD_MS);
		} else {
			// Reset PEC sums; PEC rise rate not high enough for a break
			reset_all_pec_error_sums(bmsdata->chips);
		}

	} else {
		// Only proceed if timer has expired
		if (is_timer_expired(&pec_accum_timer)) {
			uint8_t first_faulty_chip_idx = 0U, fault_detected = 0U;

			// Find the first chip that has too many PEC errors
			for (uint8_t chip = 0U; chip < NUM_CHIPS; chip++) {
				if (bmsdata->chips[chip].pec_error_sum >
				    ISOSPI_PEC_ERROR_THRESHOLD) {
					first_faulty_chip_idx = chip;
					fault_detected = 1U;
					break;
				}
			}

			// Check that all chips after the break also exceed threshold
			if (fault_detected == 1U) {
				uint8_t all_above_thresh = 1U;

				// clang-format off
				for (uint8_t chip = first_faulty_chip_idx; chip < NUM_CHIPS; chip++) {
					if (bmsdata->chips[chip].pec_error_sum <= ISOSPI_PEC_ERROR_THRESHOLD) {
						all_above_thresh = 0U;
						break;
					}
				}

				if (all_above_thresh == 1U) {

					bmsdata->isospi_status.state = ISOSPI_BREAK_DETECTED;
					bmsdata->isospi_status.break_chip = (uint8_t)(first_faulty_chip_idx + 1U);

					// Sets non-critical isospi break fault
					bmsdata->fault_code_noncrit |= INTERNAL_ISOSPI_BREAK_FAULT;

					printf("[isoSPI] Break Detected at Chip %u\n\r", first_faulty_chip_idx + 1U);
				}
				// clang-format on
			}

			// Reset PEC accumulation
			reset_all_pec_error_sums(bmsdata->chips);
		}
	}
}

/**
 * @brief Recover from an isoSPI break by switching chips after the break to the secondary line.
 *
 * @param bmsdata Pointer to the accumulator data structure.
 * @param hspi    SPI handle used for isoSPI communication.
 */
static void isospi_recover_break(acc_data_t *bmsdata, SPI_HandleTypeDef *hspi)
{
	uint8_t break_chip_idx =
		(uint8_t)(bmsdata->isospi_status.break_chip - 1U);

	if (break_chip_idx < (NUM_CHIPS - 1)) {
		printf("[isoSPI] Switching chips %u to %u to Line B\n\r",
		       break_chip_idx + 1U, NUM_CHIPS);
	} else {
		printf("[isoSPI] Switching chip %u to Line B\n\r",
		       break_chip_idx + 1U);
	}

	// Switch all chips after the break to use the other isoSPI line
	for (uint8_t i = break_chip_idx; i < NUM_CHIPS; i++) {
		set_iso_spi_line(&bmsdata->chips[i], ISOSPI_LINE_B);
	}

	// Only set COMM_BK if we're not rerouting the entire chain
	if (break_chip_idx > 0U) {
		// Set COMM_BK on both sides of the break
		set_comm_break(&bmsdata->chips[break_chip_idx], COMM_BK_ON);
		set_comm_break(&bmsdata->chips[break_chip_idx - 1], COMM_BK_ON);
	}

	// Restart the segment to apply the new isoSPI line setup and resynchronize
	segment_restart(bmsdata->chips, hspi);

	reset_all_pec_error_sums(bmsdata->chips);
}

int is_startup_mask_active(void)
{
	return !is_timer_expired(&startup_mask_timer);
}

void isospi_break_detection_init(acc_data_t *bmsdata)
{
	// Wait a short time before enabling PEC detection to avoid startup noise
	start_timer(&startup_mask_timer, ISOSPI_STARTUP_MASK_TIME);
	cancel_timer(&pec_accum_timer);

	bmsdata->isospi_status.state = ISOSPI_STATE_NORMAL;
	bmsdata->isospi_status.break_chip = 0U;
	bmsdata->isospi_status.verification_attempts = 0U;
	bmsdata->isospi_status.recovery_successful = 0U;
	bmsdata->isospi_status.fault_latched = 0U;

	reset_all_pec_error_sums(bmsdata->chips);

	send_isospi_status_message(&bmsdata->isospi_status);
}

void isospi_handle_state(acc_data_t *bmsdata, SPI_HandleTypeDef *hspi)
{
	switch (bmsdata->isospi_status.state) {
	case ISOSPI_STATE_NORMAL:
		if (bmsdata->isospi_status.recovery_successful == 0U) {
			isospi_detect_break(bmsdata);
		} else {
			// After the first recovery is successful, any further breaks cannot be corrected.
			reset_all_pec_error_sums(bmsdata->chips);
		}
		break;

	case ISOSPI_BREAK_DETECTED:
		send_isospi_status_message(&bmsdata->isospi_status);
		printf("[isoSPI] Recovery Started\n\r");
		isospi_recover_break(bmsdata, hspi);
		bmsdata->isospi_status.state = ISOSPI_STATE_VERIFYING;
		break;

	case ISOSPI_STATE_VERIFYING:
		send_isospi_status_message(&bmsdata->isospi_status);
		// clang-format off
		if (bmsdata->isospi_status.verification_attempts >= ISOSPI_VERIFICATION_READS) {
			printf("[isoSPI] Verification failed after max attempts\n\r");
			bmsdata->isospi_status.state = ISOSPI_RECOVERY_FAILED;
		} else {

			// Confirm PEC errors have dropped below acceptable level after recovery
			uint8_t break_chip_idx = (uint8_t)(bmsdata->isospi_status.break_chip - 1U);

			if (isospi_verify_recovery(bmsdata, break_chip_idx) == 0U) {
				printf("[isoSPI] Recovery succeeded\n\r");
				bmsdata->isospi_status.state = ISOSPI_RECOVERY_SUCCESS;
				bmsdata->isospi_status.recovery_successful = 1U;
			}
			bmsdata->isospi_status.verification_attempts++;
		}
		// clang-format on
		break;

	case ISOSPI_RECOVERY_SUCCESS:
		send_isospi_status_message(&bmsdata->isospi_status);

		// Clear all faults return to normal operation state
		printf("[isoSPI] Recovery Complete, Fault Cleared\n\r");
		bmsdata->fault_code_noncrit &= ~INTERNAL_ISOSPI_BREAK_FAULT;
		bmsdata->isospi_status.state = ISOSPI_STATE_NORMAL;
		break;

	case ISOSPI_RECOVERY_FAILED:
		// Run critical fault logic only once to avoid repeating logs and CAN messages
		if (!bmsdata->isospi_status.fault_latched) {
			send_isospi_status_message(&bmsdata->isospi_status);
			printf("[isoSPI] Recovery Failed. Critical Fault Latched\n\r");

			bmsdata->isospi_status.recovery_successful = 0U;
			bmsdata->isospi_status.fault_latched = 1U;
		}

		reset_all_pec_error_sums(bmsdata->chips);
		break;

	default:
		printf("[isoSPI] Invalid state: %d\n\r",
		       bmsdata->isospi_status.state);
		break;
	}
}