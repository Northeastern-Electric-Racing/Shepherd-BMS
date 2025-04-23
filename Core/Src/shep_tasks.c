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

	segment_init(bmsdata);

	// must delay after init for some reason
	osDelay(500);

	for (;;) {
		int current_state = CHARGING_STATE;
		if (current_state == CHARGING_STATE) {
			segment_mute(bmsdata);
			osDelay(50);
		} else { // snap before getting data
			//segment_snap(bmsdata);
		}

		if (current_state == CHARGING_STATE)
			segment_retrieve_charging_data(bmsdata);
		else
			segment_retrieve_active_data(bmsdata);

		if (DEBUG_MODE_ENABLED) {
			segment_retrieve_debug_data(bmsdata);
		}

		// if in normal drive mode, reboot the segment every 45 seconds in case the chips go out of sync
		// if (current_state == READY_STATE) {
		// 	if (++i % (45 * SAMPLE_RATE) == 0) {
		// 		printf(" ***********  REBOOTING SEGMENT\n\n");
		// 		segment_restart(bmsdata);
		// 	}
		// }

		//segment_disable_balancing(bmsdata);

		if (current_state == CHARGING_STATE) {
			segment_unmute(bmsdata);
		} else {
			// unsnap after getting data
			//segment_unsnap(bmsdata);
		}

		segment_enable_balancing(bmsdata);
		segment_manual_balancing(bmsdata);

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

		// calculate base values for later safety calcs
		calc_cell_temps(bmsdata);
		calc_pack_temps(bmsdata);
		calc_cell_voltages(bmsdata);
		calc_pack_voltage_stats(bmsdata);
		calc_open_cell_voltage(bmsdata);
		calc_cell_resistances(bmsdata);

		// these are dependent on above calculations
		calc_cont_dcl(bmsdata);
		calc_cont_ccl(bmsdata);
		calc_state_of_charge(bmsdata);

		// send out telemetry data sourced from the above functions
		send_acc_status_message(bmsdata->pack_voltage,
					bmsdata->pack_current, bmsdata->soc);
		send_cell_voltage_message(bmsdata->max_voltage,
					  bmsdata->min_voltage,
					  bmsdata->avg_voltage);
		send_segment_volt_message(bmsdata);
		send_cell_temp_message(bmsdata->max_temp, bmsdata->min_temp,
				       bmsdata->avg_temp);
		send_segment_temp_message(bmsdata);

		osMutexRelease(bmsdata->mutex);
	}
}

osThreadId_t current_monitor_thread;
const osThreadAttr_t current_monitor_attrs = { .name = "Get Current Data",
					       .stack_size = 2048,
					       .priority = osPriorityNormal };
void vCurrentMonitor(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		// this info is sent in with the state machine debugging code
		bmsdata->pack_current = compute_get_pack_current();
		float humidity;
		compute_measure_temp(&bmsdata->internal_temp, &humidity);
		osDelay(100);
	}
}

osThreadId_t state_machine_thread;
const osThreadAttr_t state_machine_attrs = { .name = "State machine task",
					     .stack_size = 4096,
					     .priority = osPriorityRealtime };
void vStateMachine(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;

	nertimer_t telem_timer;
	// sends unimportant telemetry messages every 500ms
	start_timer(&telem_timer, 500);

	for (;;) {
		sm_handle_state(bmsdata);

		if (is_timer_expired(&telem_timer)) {
			// these are unimportant telemetry messages so they can be sent infrequently
			send_bms_status_message(
				bmsdata->avg_temp, bmsdata->internal_temp,
				current_state,
				segment_is_balancing(bmsdata->chips));
			send_fault_status_message(bmsdata->fault_code_crit,
						  bmsdata->fault_code_noncrit);
			start_timer(&telem_timer, 300);
		}

		osDelay(100);
	}
}

osThreadId_t debug_mode_thread;
const osThreadAttr_t debug_mode_attrs = { .name = "Debug Mode Thread",
					  .stack_size = 2048,
					  .priority = osPriorityNormal };
void vDebugMode(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;

	// try to even everything out for a 1 Hz refresh rate
	uint16_t time_per_chip = 750 / NUM_CHIPS;

	while (69 < 420) {
		for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
			uint8_t num_cells =
				get_num_cells(&bmsdata->chip_data[chip]);
			// dont send the 11th cell of beta as it goes in a beta stat msg
			if (!bmsdata->chip_data[chip].alpha) {
				num_cells -= 1;
			}
			for (int cell = 0; cell < num_cells; cell += 2) {
				send_cell_data_message(
					bmsdata->chip_data[chip].alpha,

					bmsdata->chip_data[chip].cell_temp[cell],

					bmsdata->chip_data[chip]
						.cell_voltages[cell],

					bmsdata->chip_data[chip]
						.cell_voltages[cell + 1],

					chip,

					cell,

					cell + 1,

					(bmsdata->chips[chip].tx_cfgb.dcc >>
					 cell) & 1,

					(bmsdata->chips[chip].tx_cfgb.dcc >>
					 (cell + 1)) &
						1,
					(bmsdata->chips[chip].statc.cs_flt >>
					 cell) & 1,
					(bmsdata->chips[chip].statc.cs_flt >>
					 (cell + 1)) &
						1);
				// split half the time amongst the cells (over 2)
				osDelay(10);
			}

			// Send chip status messages
			if (!bmsdata->chip_data[chip].alpha) {
				send_beta_status_a_message(
					bmsdata->chip_data[chip].cell_temp[10],
					bmsdata->chip_data[chip]
						.cell_voltages[10],
					NER_GET_BIT(
						bmsdata->chips[chip].tx_cfgb.dcc,
						10),
					chip,

					bmsdata->chip_data[chip].on_board_temp,

					(getVoltage(bmsdata->chips[chip]
							    .stata.itmp) /
					 0.0075) -
						273,
					20.0 * getVoltage( // VPV is ra_code 11 w/ different scale
						       bmsdata->chips[chip]
							       .aux
							       .a_codes[11]));
				// wait for 1/4 the chip time
				osDelay(30);
				send_beta_status_b_message(
					getVoltage(bmsdata->chips[chip]
							   .stata.vref2),
					getVoltage(
						bmsdata->chips[chip].statb.va),
					getVoltage(
						bmsdata->chips[chip].statb.vd),
					chip,
					getVoltage(
						bmsdata->chips[chip].statb.vr4k),
					20.0 * getVoltage( // VMV is ra_code 10
						       bmsdata->chips[chip]
							       .aux.a_codes[10]),
					(bmsdata->chips[chip].statc.cs_flt >>
					 10) & 1);
				send_beta_status_c_message(
					chip, &bmsdata->chips[chip].statc);
			} else {
				send_alpha_status_a_message(
					bmsdata->chip_data->on_board_temp, chip,
					((getVoltage(bmsdata->chips[chip]
							     .stata.itmp) /
					  0.0075) -
					 273),
					(20.0 *
					 getVoltage( // VPV is ra_code 11 w/ different scale
						 bmsdata->chips[chip]
							 .aux.a_codes[11])),
					(20.0 *
					 getVoltage( // VMV is ra_code 10
						 bmsdata->chips[chip]
							 .aux.a_codes[10])),
					&bmsdata->chips[chip].statc);
				// wait for 1/4 the chip time
				osDelay(30);
				send_alpha_status_b_message(
					getVoltage(
						bmsdata->chips[chip].statb.vr4k),
					chip,
					getVoltage(bmsdata->chips[chip]
							   .stata.vref2),
					getVoltage(
						bmsdata->chips[chip].statb.va),
					getVoltage(
						bmsdata->chips[chip].statb.vd),
					&bmsdata->chips[chip].statc);
			}
			// wait for 1/4 the chip time
			osDelay(30);
		}
	}
}