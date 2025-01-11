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
#include "stm32f4xx_hal.h"
#include "can_handler.h"
#include "analyzer.h"
#include "compute.h"
#include <stdio.h>

#define STATE_MACHINE_FLAG 1

#define ANALYZER_FLAG 1

osThreadId_t get_segment_data_thread;
const osThreadAttr_t get_segment_data_attrs = { .name = "Get Segment Data",
						.stack_size = 2048,
						.priority = osPriorityNormal };

void vGetSegmentData(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		// printf("Get segment data\n");
		segment_retrieve_data(bmsdata);
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
		calc_pack_voltage_stats(bmsdata);
		calc_open_cell_voltage(bmsdata);
		calc_cell_resistances(bmsdata);
		calc_dcl(bmsdata);
		calc_cont_dcl(bmsdata);
		//calcCCL();
		calc_cont_ccl(bmsdata);
		// temporary
		bmsdata->charge_limit = bmsdata->cont_CCL;
		compute_send_mc_charge_message(bmsdata);
		compute_send_current_message(bmsdata);
		// temporary end

		calc_state_of_charge(bmsdata);
		calc_noise_volt_percent(bmsdata);

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
		compute_send_acc_status_message(bmsdata);
		compute_send_current_message(bmsdata);
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
