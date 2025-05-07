#include "isospi_recovery.h"
#include "bmsConfig.h"
#include "timer.h"

void isospi_break_detection_init(acc_data_t *bmsdata)
{
	start_timer(&bmsdata->isospi_status.startup_mask_timer,
		    ISOSPI_STARTUP_MASK_TIME_MS);
	cancel_timer(&bmsdata->isospi_status.pec_accum_timer);

	bmsdata->isospi_status.state = ISOSPI_STATE_NORMAL;
	bmsdata->isospi_status.recovery_attempts = 0;
	bmsdata->isospi_status.recovery_successful = false;

	memset(bmsdata->isospi_status.pec_error_sum, 0,
	       sizeof(bmsdata->isospi_status.pec_error_sum));
}

void detect_isospi_break(acc_data_t *bmsdata)
{
	// Skip detection during startup mask period
	if (!is_timer_expired(&bmsdata->isospi_status.startup_mask_timer)) {
		return;
	}

	// Only run detection periodically
	if (!is_timer_expired(&bmsdata->isospi_status.pec_accum_timer)) {
		return;
	}

	uint8_t chip = 0U;
	uint8_t first_faulty_chip = NUM_CHIPS;
	bool fault_detected = false;

	// Find the first chip exceeding the PEC threshold
	for (chip = 0U; chip < NUM_CHIPS; chip++) {
		if (bmsdata->isospi_status.pec_error_sum[chip] >
		    ISOSPI_PEC_ERROR_THRESHOLD) {
			first_faulty_chip = chip;
			fault_detected = true;
			break;
		}
	}

	// Confirm all subsequent chips also exceed threshold
	if (fault_detected) {
		for (chip = first_faulty_chip; chip < NUM_CHIPS; chip++) {
			if (bmsdata->isospi_status.pec_error_sum[chip] <=
			    ISOSPI_PEC_ERROR_THRESHOLD) {
				fault_detected = false;
				break;
			}
		}
	}

	if (fault_detected) {
		bmsdata->isospi_status.state = ISOSPI_BREAK_DETECTED;
		bmsdata->isospi_status.break_chip_index = first_faulty_chip;

		printf("ISO SPI Break Detected! First bad chip: %u\n",
		       first_faulty_chip);

		// Set non-critical fault on first detection
		bmsdata->fault_code_noncrit |= INTERNAL_ISOSPI_BREAK_FAULT;
	}

	// Reset PEC counters and restart accumulation timer
	for (chip = 0U; chip < NUM_CHIPS; chip++) {
		bmsdata->isospi_status.pec_error_sum[chip] = 0U;
	}
	start_timer(&bmsdata->isospi_status.pec_accum_timer,
		    ISOSPI_ACCUM_PERIOD_MS);
}

void isospi_state_dispatcher(isospi_comm_state_t isospi_state,
			     acc_data_t *bmsdata)
{
	switch (isospi_state) {
	case ISOSPI_STATE_NORMAL:
		detect_isospi_break(bmsdata);
		break;

	case ISOSPI_BREAK_DETECTED:

		break;

	case ISOSPI_RECOVERY_SUCCESS:

		break;

	case ISOSPI_RECOVERY_FAILED:

		break;
	}
}