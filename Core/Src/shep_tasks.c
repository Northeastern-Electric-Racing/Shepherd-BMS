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

#define STATE_MACHINE_FLAG 1

#define CAN_DISPATCH_FLAG 1

#define ANALYZER_FLAG 1

osThreadId_t get_segment_data_thread;
const osThreadAttr_t get_segment_data_attrs = { .name = "Get Segment Data",
						.stack_size = 2048,
						.priority = osPriorityNormal };

void vGetSegmentData(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		segment_retrieve_data(bmsdata->chip_data);
		osThreadFlagsSet(analyzer_thread, ANALYZER_FLAG);
		osThreadYield();
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
		disable_therms(bmsdata);

		calc_cell_temps(bmsdata);
		calc_pack_temps(bmsdata);
		calc_pack_voltage_stats(bmsdata);
		calc_open_cell_voltage(bmsdata);
		calc_cell_resistances(bmsdata);
		calc_dcl(bmsdata);
		calc_cont_dcl(bmsdata);
		//calcCCL();
		calc_cont_ccl(bmsdata);
		calc_state_of_charge(bmsdata);
		calc_noise_volt_percent(bmsdata);

		bmsdata->charge_limit = bmsdata->cont_CCL;

		osMutexRelease(bmsdata->mutex);
		osThreadYield();
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
		osDelay(100);
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

osThreadId_t can_dispatch_thread;
const osThreadAttr_t can_dispatch_attrs = {
	.name = "Can Dispatch",
	.stack_size = 128 * 8,
	.priority = (osPriority_t)osPriorityRealtime5,
};

void vCanDispatch(void *pv_params)
{
	can_msg_t msg_from_queue;
	HAL_StatusTypeDef msg_status;

	can_t *line;
#ifdef CHARGING
	line = &can2;
#endif
	line = &can1;

	for (;;) {
		osThreadFlagsWait(CAN_DISPATCH_FLAG, osFlagsWaitAny,
				  osFlagsWaitAny);
		/* Send CAN message */
		if (osOK == osMessageQueueGet(can_outbound_queue,
					      &msg_from_queue, NULL,
					      osWaitForever)) {
			msg_status = can_send_msg(line, &msg_from_queue);
			if (msg_status == HAL_ERROR) {
				// TODO: error handling
				// fault_data.diag = "Failed to send CAN message";
			} else if (msg_status == HAL_BUSY) {
				// TODO: error handling
				//"Outbound mailbox full!";
			}
		}

		osDelay(CAN_DISPATCH_DELAY);
	}
}
