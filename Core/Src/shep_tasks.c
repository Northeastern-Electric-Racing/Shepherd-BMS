/**
 * @file tasks.c
 * @brief Define Shepherd's RTOS tasks. This file is most likely temporary.
 * @version 0.1
 * @date 2024-09-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "shep_tasks.h"

#include "bmsConfig.h"
#include "can_messages.h"
#include "c_utils.h"
#include "compute.h"
#include "segment.h"
#include "serialPrintResult.h"
#include "stateMachine.h"

#define STATE_MACHINE_FLAG 1

#define ANALYZER_FLAG 1

osThreadId_t get_segment_data_thread;
const osThreadAttr_t get_segment_data_attrs = { .name = "Get Segment Data",
						.stack_size = 2048,
						.priority = osPriorityNormal };

void vGetSegmentData(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;

	int i = 0;
	for (;;) {
		// printf("Get segment data\n");
		segment_retrieve_data(bmsdata);

		if (DEBUG_MODE_ENABLED) {
			segment_retrieve_debug_data(bmsdata);
		}

// if in normal drive mode, reboot the segment every 45 seconds in case the chips go out of sync
#ifndef CHARGING_ENABLED
		if (++i % (45 * SAMPLE_RATE) == 0) {
			printf(" ***********  REBOOTING SEGMENT\n\n");
			segment_restart(bmsdata);
		}
#endif

		osThreadFlagsSet(analyzer_thread, ANALYZER_FLAG);
		osDelay(1000 / SAMPLE_RATE);
	}
}

osThreadId_t analyzer_thread;
const osThreadAttr_t analyzer_attrs = { .name = "Analyzer",
					.stack_size = 4096,
					.priority = osPriorityNormal };
void vAnalyzer(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;

	for (;;) {
		osThreadFlagsWait(ANALYZER_FLAG, osFlagsWaitAny, osWaitForever);

		osMutexAcquire(bmsdata->mutex, osWaitForever);
		// disable_therms(bmsdata);

		calc_cell_temps(bmsdata);
		calc_pack_temps(bmsdata);
		calc_cell_voltages(bmsdata);
		calc_pack_voltage_stats(bmsdata);
		calc_open_cell_voltage(bmsdata);
		calc_cell_resistances(bmsdata);
		calc_dcl(bmsdata);
		calc_cont_dcl(bmsdata);
		//calcCCL();
		calc_cont_ccl(bmsdata);
		// temporary
		bmsdata->charge_limit = bmsdata->cont_CCL;
		send_mc_charge_message(bmsdata);
		send_current_message(bmsdata);
		// temporary end

		// calc_state_of_charge(bmsdata);
		// calc_noise_volt_percent(bmsdata);

		osMutexRelease(bmsdata->mutex);
	}
}

osThreadId_t current_monitor_thread;
const osThreadAttr_t current_monitor_attrs = { .name = "Get Segment Data",
					       .stack_size = 2048,
					       .priority = osPriorityNormal };
void vCurrentMonitor(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		bmsdata->pack_current = compute_get_pack_current();
		send_acc_status_message(bmsdata);
		send_current_message(bmsdata);
		osDelay(1000 / SAMPLE_RATE);
	}
}

osThreadId_t state_machine_thread;
const osThreadAttr_t state_machine_attrs = { .name = "State machine task",
					     .stack_size = 2048,
					     .priority = osPriorityRealtime };
void vStateMachine(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;

	for (;;) {
		sm_handle_state(bmsdata);
		osDelay(10);
	}
}

osThreadId_t debug_mode_thread;
const osThreadAttr_t debug_mode_attrs = { .name = "Debug Mode Thread",
					  .stack_size = 2048,
					  .priority = osPriorityNormal };
void vDebugMode(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;

	while (69 < 420) {
		for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
			uint8_t num_cells =
				get_num_cells(&bmsdata->chip_data[chip]);
			for (int cell = 0; cell < num_cells; cell += 2) {
				send_cell_data_message(
					bmsdata->chip_data[chip].alpha,

					bmsdata->chip_data[chip].cell_temp[cell],

					10000 * getVoltage(
							bmsdata->chip_data[chip]
								.cell_voltages
									[cell]),

					10000 * getVoltage(
							bmsdata->chip_data[chip]
								.cell_voltages
									[cell +
									 1]),

					chip,

					cell,

					cell + 1,

					(bmsdata->chips[chip].tx_cfgb.dcc >>
					 cell) & 1,

					(bmsdata->chips[chip].tx_cfgb.dcc >>
					 (cell + 1)) &
						1);
				osDelay(1000 / NUM_CHIPS);
			}

			// Send chip status messages
			if (!bmsdata->chip_data[chip].alpha) {
				send_beta_status_a_message(
					10000 * getVoltage(
							bmsdata->chip_data[chip]
								.cell_temp[10]),
					10000 * getVoltage(
							bmsdata->chip_data[chip]
								.cell_voltages
									[10]),
					NER_GET_BIT(
						bmsdata->chips[chip].tx_cfgb.dcc,
						10),
					chip,
					bmsdata->chip_data[chip].on_board_temp,
					(getVoltage(bmsdata->chips[chip]
							    .stata.itmp) /
					 0.0075) -
						273,
					10000 * 20 *
						getVoltage( // VPV is ra_code 11 w/ different scale
							bmsdata->chips[chip]
								.raux
								.ra_codes[11]));
				send_beta_status_b_message(
					10000 * getVoltage(
							bmsdata->chips[chip]
								.stata.vref2),
					10000 * getVoltage(bmsdata->chips[chip]
								   .statb.va),
					10000 * getVoltage(bmsdata->chips[chip]
								   .statb.vd),
					chip,
					10000 * getVoltage(bmsdata->chips[chip]
								   .statb.vr4k),
					10000 * 20 *
						getVoltage( // VMV is ra_code 10
							bmsdata->chips[chip]
								.raux
								.ra_codes[10]));
				send_beta_status_c_message(
					chip, &bmsdata->chips[chip].statc);
			} else {
				send_alpha_status_a_message(
					bmsdata->chip_data->on_board_temp, chip,
					(getVoltage(bmsdata->chips[chip]
							    .stata.itmp) /
					 0.0075) -
						273,
					10000 * getVoltage(
							bmsdata->chips[chip]
								.raux
								.ra_codes[9]),
					10000 * getVoltage(
							bmsdata->chips[chip]
								.raux
								.ra_codes[8]),
					&bmsdata->chips[chip].statc);
				send_alpha_status_b_message(
					10000 * getVoltage(bmsdata->chips[chip]
								   .statb.vr4k),
					chip,
					10000 * getVoltage(
							bmsdata->chips[chip]
								.stata.vref2),
					10000 * getVoltage(bmsdata->chips[chip]
								   .statb.va),
					10000 * getVoltage(bmsdata->chips[chip]
								   .statb.vd),
					&bmsdata->chips[chip].statc);
			}
		}
	}
}