#include "isospi_recovery.h"
#include "adi_interaction.h"
#include "segment.h"
#include "bmsConfig.h"
#include "can_messages.h"
#include "timer.h"

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
	for (uint8_t r = 0U; r < ISOSPI_VERIFICATION_READS; r++) {
		segment_retrieve_active_data(bmsdata);
		count_pec_errors(bmsdata->chips);

		for (uint8_t i = start_chip; i < NUM_CHIPS; i++) {
			if (bmsdata->chips[i].pec_error_sum > 0) {
				return 0;
			}
		}

		reset_all_pec_error_sums(bmsdata->chips);
		osDelay(ISOSPI_VERIFICATION_DELAY);
	}
	return 1;
}

void isospi_break_detection_init(acc_data_t *bmsdata)
{
	// Wait a short time before enabling PEC detection to avoid startup noise
	start_timer(&bmsdata->isospi_status.startup_mask_timer,
		    ISOSPI_STARTUP_MASK_TIME_MS);
	cancel_timer(&bmsdata->isospi_status.pec_accum_timer);

	bmsdata->isospi_status.state = ISOSPI_STATE_NORMAL;
	bmsdata->isospi_status.recovery_attempts = 0U;
	bmsdata->isospi_status.recovery_successful = 0U;

	reset_all_pec_error_sums(bmsdata->chips);

	send_isospi_status_message(&bmsdata->isospi_status);
	send_isospi_lines_message(bmsdata->chips);
}

void detect_isospi_break(acc_data_t *bmsdata)
{
	if (!is_timer_expired(&bmsdata->isospi_status.startup_mask_timer)) {
		return;
	}

	// Start accumulation timer on first PEC activity
	if (!is_timer_active(&bmsdata->isospi_status.pec_accum_timer)) {
		for (uint8_t i = 1U; i < NUM_CHIPS; i++) {
			if (bmsdata->chips[i].pec_error_sum > 0U) {
				start_timer(
					&bmsdata->isospi_status.pec_accum_timer,
					ISOSPI_ACCUM_PERIOD_MS);
				return;
			}
		}
		return;
	}

	// Only proceed if timer has expired
	if (!is_timer_expired(&bmsdata->isospi_status.pec_accum_timer)) {
		return;
	}

	uint8_t first_faulty_chip = NUM_CHIPS;
	int fault_detected = 0;

	// Find the first chip that has too many PEC errors
	for (uint8_t chip = 1U; chip < NUM_CHIPS; chip++) {
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
		bmsdata->isospi_status.state = ISOSPI_BREAK_DETECTED;
		bmsdata->isospi_status.break_chip_index = first_faulty_chip;
		bmsdata->fault_code_noncrit |= INTERNAL_ISOSPI_BREAK_FAULT;

		printf("[isoSPI] Break Detected at Chip %u\n\r",
		       first_faulty_chip + 1);
	}

	// Reset PEC accumulation and restart timer for next window
	reset_all_pec_error_sums(bmsdata->chips);

	start_timer(&bmsdata->isospi_status.pec_accum_timer,
		    ISOSPI_ACCUM_PERIOD_MS);
}

int32_t attempt_isospi_recovery(acc_data_t *bmsdata)
{
	uint8_t break_chip = bmsdata->isospi_status.break_chip_index;

	// Switch all chips after the break to use the other isoSPI line
	for (int i = break_chip; i < NUM_CHIPS; i++) {
		bmsdata->chips[i].isospi_line = ISOSPI_LINE_B;
	}

	// Only set COMM_BK if we're not rerouting the entire chain
	if (break_chip > 0U) {
		// Set COMM_BK on both sides of the break
		set_comm_break(&bmsdata->chips[break_chip], COMM_BK_ON);
		set_comm_break(&bmsdata->chips[break_chip - 1], COMM_BK_ON);
	}

	// Write updated config to chips
	write_config_regs(bmsdata->chips);
	mute_chips(bmsdata->chips);
	start_c_adc_conv(bmsdata->chips);

	return verify_isospi_recovery(bmsdata, break_chip);
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
			printf("[isoSPI] Break reoccurred after recovery — escalation\n\r");
			bmsdata->isospi_status.state = ISOSPI_RECOVERY_FAILED;
			break;
		}

		if (bmsdata->isospi_status.recovery_attempts >=
		    ISOSPI_RECOVERY_RETRIES_MAX) {
			printf("[isoSPI] Recovery attempt limit reached\n\r");
			bmsdata->isospi_status.state = ISOSPI_RECOVERY_FAILED;
			break;
		}

		if (attempt_isospi_recovery(bmsdata) != 0) {
			printf("[isoSPI] Recovery Succeeded\n\r");
			bmsdata->isospi_status.state = ISOSPI_RECOVERY_SUCCESS;
			bmsdata->isospi_status.recovery_successful = 1U;
		} else {
			bmsdata->isospi_status.recovery_attempts++;
			printf("[isoSPI] Recovery Failed (attempt %u)\n\r",
			       bmsdata->isospi_status.recovery_attempts);
			osDelay(250);
		}
		break;

	case ISOSPI_RECOVERY_SUCCESS:
		send_isospi_status_message(&bmsdata->isospi_status);
		send_isospi_lines_message(bmsdata->chips);

		printf("[isoSPI] Recovery Complete, Fault Cleared\n\r");
		bmsdata->fault_code_noncrit &= ~INTERNAL_ISOSPI_BREAK_FAULT;
		bmsdata->fault_code_crit &= ~INTERNAL_ISOSPI_BREAK_FAULT;
		bmsdata->isospi_status.state = ISOSPI_STATE_NORMAL;
		break;

	case ISOSPI_RECOVERY_FAILED:
		send_isospi_status_message(&bmsdata->isospi_status);

		printf("[isoSPI] Recovery Failed — Critical Fault\n\r");
		bmsdata->fault_code_noncrit &= ~INTERNAL_ISOSPI_BREAK_FAULT;
		bmsdata->fault_code_crit |= INTERNAL_ISOSPI_BREAK_FAULT;
		reset_all_pec_error_sums(bmsdata->chips);
		break;

	default:
		break;
	}
}