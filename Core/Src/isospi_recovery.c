#include "isospi_recovery.h"
#include "adi_interaction.h"
#include "segment.h"
#include "bmsConfig.h"
#include "can_messages.h"
#include "timer.h"

/* PEC Error Thresholds */
#define ISOSPI_PEC_ERROR_THRESHOLD  20U // Break detect threshold
#define ISOSPI_VALIDATION_THRESHOLD 5U // PECs allowed during recovery

/* Timing (ms) */
#define ISOSPI_STARTUP_MASK_TIME 1500U // Wait before PEC fault evaluation (ms)
#define ISOSPI_ACCUM_PERIOD_MS	 3000U // Accumulation window

/* Recovery Parameters */
#define ISOSPI_VERIFICATION_READS 3U // Reads to confirm success

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
 * @param start_chip Index of the chip where the break occurred.
 * @return 1 if all chips recovered successfully, 0 otherwise.
 */
static int32_t verify_isospi_recovery(acc_data_t *bmsdata, uint8_t start_chip)
{
	for (uint8_t i = start_chip; i < NUM_CHIPS; i++) {
		if (bmsdata->chips[i].pec_error_sum >
		    ISOSPI_VALIDATION_THRESHOLD) {
			printf("[isoSPI] Verification failed at chip %u (PEC: %u)\n\r",
			       i + 1, bmsdata->chips[i].pec_error_sum);
			reset_all_pec_error_sums(bmsdata->chips);
			return 0;
		}
	}
	printf("[isoSPI] Verification passed\n\r");
	reset_all_pec_error_sums(bmsdata->chips);
	return 1;
}

/**
 * @brief Checks for ISO SPI communication break using PEC error tracking.
 *
 * If a break is detected based on PEC thresholds, the internal state is updated and
 * a non-critical fault is flagged. Resets counters after each check.
 *
 * @param bmsdata Pointer to accumulator data structure.
 */
static void detect_isospi_break(acc_data_t *bmsdata)
{
	// Start accumulation timer on first PEC activity
	if (!is_timer_active(&pec_accum_timer)) {
		for (uint8_t i = 0U; i < NUM_CHIPS; i++) {
			if (bmsdata->chips[i].pec_error_sum > 0U) {
				start_timer(&pec_accum_timer,
					    ISOSPI_ACCUM_PERIOD_MS);
				return;
			}
		}
		return;
	}

	// Only proceed if timer has expired
	if (!is_timer_expired(&pec_accum_timer)) {
		return;
	}

	uint8_t first_faulty_chip = NUM_CHIPS;
	int fault_detected = 0;

	// Find the first chip that has too many PEC errors
	for (uint8_t chip = 0U; chip < NUM_CHIPS; chip++) {
		if (bmsdata->chips[chip].pec_error_sum >
		    ISOSPI_PEC_ERROR_THRESHOLD) {
			first_faulty_chip = chip;
			fault_detected = 1;
			break;
		}
	}

	// Check that all chips after the break also exceed threshold
	if (bmsdata->isospi_status.recovery_successful == 0U &&
	    fault_detected) {
		for (uint8_t chip = first_faulty_chip; chip < NUM_CHIPS;
		     chip++) {
			if (bmsdata->chips[chip].pec_error_sum <=
			    ISOSPI_PEC_ERROR_THRESHOLD) {
				fault_detected = 0;
				break;
			}
		}
	}

	if (fault_detected) {
		// Sets non-critical fault initially
		bmsdata->isospi_status.state = ISOSPI_BREAK_DETECTED;
		bmsdata->isospi_status.break_chip_index = first_faulty_chip;
		bmsdata->fault_code_noncrit |= INTERNAL_ISOSPI_BREAK_FAULT;

		printf("[isoSPI] Break Detected at Chip %u\n\r",
		       first_faulty_chip + 1);

		// Immediately escalate to recovery without waiting for next task cycle
		isospi_state_dispatcher(bmsdata);
		return;
	}

	// Reset PEC accumulation and restart timer for next window
	reset_all_pec_error_sums(bmsdata->chips);
}

/**
 * @brief Attempts to recover from an ISO SPI break by switching to the secondary line.
 *
 * Updates chip direction, sets COMM_BK, writes configs, and verifies recovery by checking PEC errors.
 *
 * @param bmsdata Pointer to the accumulator data structure.
 * @return int 1 if recovery was successful, 0 otherwise.
 */
static void attempt_isospi_recovery(acc_data_t *bmsdata)
{
	uint8_t break_chip = bmsdata->isospi_status.break_chip_index;

	printf("[isoSPI] Switching chips %u to %u to Line B\n\r",
	       break_chip + 1, NUM_CHIPS);

	// Switch all chips after the break to use the other isoSPI line
	for (int i = break_chip; i < NUM_CHIPS; i++) {
		set_iso_spi_line(&bmsdata->chips[i], ISOSPI_LINE_B);
	}

	// Only set COMM_BK if we're not rerouting the entire chain
	if (break_chip > 0U) {
		// Set COMM_BK on both sides of the break
		set_comm_break(&bmsdata->chips[break_chip], COMM_BK_ON);
		set_comm_break(&bmsdata->chips[break_chip - 1], COMM_BK_ON);
	}

	// Write updated config to chips
	write_config_regs(bmsdata->chips);

	// Disable Balancing
	mute_chips(bmsdata->chips);

	// Start adc conversions
	start_c_adc_conv(bmsdata->chips);

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
	bmsdata->isospi_status.verification_attempts = 0U;
	bmsdata->isospi_status.recovery_successful = 0U;
	bmsdata->isospi_status.fault_latched = 0U;

	reset_all_pec_error_sums(bmsdata->chips);

	send_isospi_status_message(&bmsdata->isospi_status);
	send_isospi_lines_message(bmsdata->chips);
}

void isospi_state_dispatcher(acc_data_t *bmsdata)
{
	switch (bmsdata->isospi_status.state) {
	case ISOSPI_STATE_NORMAL:
		detect_isospi_break(bmsdata);
		break;

	case ISOSPI_BREAK_DETECTED:
		send_isospi_status_message(&bmsdata->isospi_status);

		if (bmsdata->isospi_status.recovery_successful == 1U) {
			printf("[isoSPI] Break reoccurred after recovery\n\r");
			bmsdata->isospi_status.state = ISOSPI_RECOVERY_FAILED;
			break;
		}

		printf("[isoSPI] Recovery Started\n\r");
		attempt_isospi_recovery(bmsdata);
		bmsdata->isospi_status.state = ISOSPI_STATE_VERIFYING;
		break;

	case ISOSPI_STATE_VERIFYING:
		if (bmsdata->isospi_status.verification_attempts >=
		    ISOSPI_VERIFICATION_READS) {
			printf("[isoSPI] Verification failed after max attempts\n\r");
			bmsdata->isospi_status.state = ISOSPI_RECOVERY_FAILED;
		} else {
			// clang-format off
			// Confirm PEC errors have dropped below acceptable level after switching lines
			if (verify_isospi_recovery(bmsdata, bmsdata->isospi_status.break_chip_index) == 1) {
				printf("[isoSPI] Recovery succeeded\n\r");
				bmsdata->isospi_status.state =
					ISOSPI_RECOVERY_SUCCESS;
				bmsdata->isospi_status.recovery_successful = 1U;
			}
			// clang-format on
			bmsdata->isospi_status.verification_attempts++;
		}
		break;

	case ISOSPI_RECOVERY_SUCCESS:
		send_isospi_status_message(&bmsdata->isospi_status);
		send_isospi_lines_message(bmsdata->chips);

		// Clear all faults return to normal operation state
		printf("[isoSPI] Recovery Complete, Fault Cleared\n\r");
		bmsdata->fault_code_noncrit &= ~INTERNAL_ISOSPI_BREAK_FAULT;
		bmsdata->fault_code_crit &= ~INTERNAL_ISOSPI_BREAK_FAULT;
		bmsdata->isospi_status.state = ISOSPI_STATE_NORMAL;
		break;

	case ISOSPI_RECOVERY_FAILED:
		// Run critical fault logic only once to avoid repeating logs and CAN messages
		if (!bmsdata->isospi_status.fault_latched) {
			send_isospi_status_message(&bmsdata->isospi_status);
			printf("[isoSPI] Recovery Failed. Critical Fault Latched\n\r");

			bmsdata->fault_code_noncrit &=
				~INTERNAL_ISOSPI_BREAK_FAULT;
			bmsdata->fault_code_crit |= INTERNAL_ISOSPI_BREAK_FAULT;

			bmsdata->isospi_status.recovery_successful = 0U;
			bmsdata->isospi_status.fault_latched = 1U;
		}

		reset_all_pec_error_sums(bmsdata->chips);
		break;

	default:
		break;
	}
}