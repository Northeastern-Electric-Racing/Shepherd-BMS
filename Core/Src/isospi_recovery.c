#include "isospi_recovery.h"
#include "adi_interaction.h"
#include "segment.h"
#include "bmsConfig.h"
#include "timer.h"

void isospi_break_detection_init(acc_data_t *bmsdata)
{
	// Wait a short time before enabling PEC detection to avoid startup noise
	start_timer(&bmsdata->isospi_status.startup_mask_timer,
		    ISOSPI_STARTUP_MASK_TIME_MS);
	cancel_timer(&bmsdata->isospi_status.pec_accum_timer);

	bmsdata->isospi_status.state = ISOSPI_STATE_NORMAL;
	bmsdata->isospi_status.recovery_attempts = 0;
	bmsdata->isospi_status.recovery_successful = 0;

	memset(bmsdata->isospi_status.pec_error_sum, 0,
	       sizeof(bmsdata->isospi_status.pec_error_sum));
}

void detect_isospi_break(acc_data_t *bmsdata)
{
	if (!is_timer_expired(&bmsdata->isospi_status.startup_mask_timer)) {
		return;
	}

	// Only run detection periodically based on timer
	if (!is_timer_expired(&bmsdata->isospi_status.pec_accum_timer)) {
		return;
	}

	uint8_t chip = 0U;
	uint8_t first_faulty_chip = NUM_CHIPS;
	int fault_detected = 0;

	// Find the first chip that has too many PEC errors
	for (chip = 0U; chip < NUM_CHIPS; chip++) {
		if (bmsdata->isospi_status.pec_error_sum[chip] >
		    ISOSPI_PEC_ERROR_THRESHOLD) {
			first_faulty_chip = chip;
			fault_detected = 1;
			break;
		}
	}

	// Make sure all chips after the first bad one are also bad
	if (fault_detected) {
		for (chip = first_faulty_chip; chip < NUM_CHIPS; chip++) {
			if (bmsdata->isospi_status.pec_error_sum[chip] <=
			    ISOSPI_PEC_ERROR_THRESHOLD) {
				fault_detected = 0;
				break;
			}
		}
	}

	if (fault_detected) {
		bmsdata->isospi_status.state = ISOSPI_BREAK_DETECTED;
		bmsdata->isospi_status.break_chip_index = first_faulty_chip;

		// Log and set non-critical fault
		printf("ISO SPI Break Detected! First bad chip: %u\n",
		       first_faulty_chip);
		bmsdata->fault_code_noncrit |= INTERNAL_ISOSPI_BREAK_FAULT;
	}

	// Reset PEC counters and restart timer
	for (chip = 0U; chip < NUM_CHIPS; chip++) {
		bmsdata->isospi_status.pec_error_sum[chip] = 0U;
	}
	start_timer(&bmsdata->isospi_status.pec_accum_timer,
		    ISOSPI_ACCUM_PERIOD_MS);
}

int attempt_isospi_recovery(acc_data_t *bmsdata)
{
	uint8_t break_chip = bmsdata->isospi_status.break_chip_index;

	// Switch all chips after the break to use the other isoSPI line
	for (int i = break_chip; i < NUM_CHIPS; i++) {
		bmsdata->chips[i].isospi_line = ISOSPI_LINE_B;
	}

	// Only set COMM_BK if we're not rerouting the entire chain
	if (break_chip > 0) {
		// Set COMM_BK on both sides of the break
		set_comm_break(&bmsdata->chips[break_chip], COMM_BK_ON);
		set_comm_break(&bmsdata->chips[break_chip - 1], COMM_BK_ON);
	}

	// Write updated config to chips
	write_config_regs(bmsdata->chips);

	// Try reading data again
	segment_retrieve_active_data(bmsdata);
	count_pec_errors(bmsdata);

	// Check if errors are gone
	for (int i = break_chip; i < NUM_CHIPS; i++) {
		if (bmsdata->isospi_status.pec_error_sum[i] >
		    ISOSPI_PEC_ERROR_THRESHOLD) {
			return 0; // Recovery failed
		}
	}

	return 1; // Recovery successful
}

void isospi_state_dispatcher(isospi_comm_state_t isospi_state,
			     acc_data_t *bmsdata)
{
	switch (isospi_state) {
	case ISOSPI_STATE_NORMAL:
		detect_isospi_break(bmsdata);
		break;

	case ISOSPI_BREAK_DETECTED:
		// Don’t retry if we already tried once
		if (bmsdata->isospi_status.recovery_attempts > 0) {
			bmsdata->isospi_status.state = ISOSPI_RECOVERY_FAILED;
			break;
		}

		int result = attempt_isospi_recovery(bmsdata);
		bmsdata->isospi_status.recovery_attempts++;

		if (result) {
			bmsdata->isospi_status.state = ISOSPI_RECOVERY_SUCCESS;
			bmsdata->isospi_status.recovery_successful = 1;
		} else {
			bmsdata->isospi_status.state = ISOSPI_RECOVERY_FAILED;
		}
		break;

	case ISOSPI_RECOVERY_SUCCESS:
		// Clear all fault flags if recovery worked
		bmsdata->fault_code_noncrit &= ~INTERNAL_ISOSPI_BREAK_FAULT;
		bmsdata->fault_code_crit &= ~INTERNAL_ISOSPI_BREAK_FAULT;
		break;

	case ISOSPI_RECOVERY_FAILED:
		// Escalate to critical fault if recovery failed
		bmsdata->fault_code_noncrit &= ~INTERNAL_ISOSPI_BREAK_FAULT;
		bmsdata->fault_code_crit |= INTERNAL_ISOSPI_BREAK_FAULT;
		break;

	default:
		break;
	}
}