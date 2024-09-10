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

osThreadId_t get_segment_data_thread;
const osThreadAttr_t get_segment_data_attrs = { .name = "Get Segment Data",
						.stack_size = 2048,
						.priority = osPriorityNormal };

void vGetSegmentData(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		segment_retrieve_data(bmsdata->chip_data);
		osThreadFlagsSet(calc_ocv_thread, CALC_OCV_FLAG);
		osThreadFlagsSet(calc_noise_thread, CALC_NOISE_FLAG);
		osThreadFlagsSet(calc_therms_thread, CALC_THERMS_FLAG);
		osDelay(10);
	}
}

osThreadId_t calc_ocv_thread;
const osThreadAttr_t calc_ocv_attrs = { .name = "Get Segment Data",
					.stack_size = 2048,
					.priority = osPriorityNormal };
void vCalcOCV(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		osThreadFlagsWait(CALC_OCV_FLAG, osFlagsWaitAny, osWaitForever);
		calc_open_cell_voltage(bmsdata);
		osThreadFlagsSet(calc_volt_stats_thread, CALC_VOLT_STATS_FLAG);
		osDelay(30);
	}
}

osThreadId_t calc_noise_thread;
const osThreadAttr_t calc_noise_attrs = { .name = "Get Segment Data",
					  .stack_size = 2048,
					  .priority = osPriorityNormal };
void vCalcNoise(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		osThreadFlagsWait(CALC_NOISE_FLAG, osFlagsWaitAny,
				  osWaitForever);
		calc_noise_volt_percent(bmsdata);
		osDelay(1000);
	}
}

osThreadId_t calc_volt_stats_thread;
const osThreadAttr_t calc_volt_stats_attrs = { .name = "Get Segment Data",
					       .stack_size = 2048,
					       .priority = osPriorityNormal };
void vCalcVoltageStats(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		osThreadFlagsWait(CALC_VOLT_STATS_FLAG, osFlagsWaitAny,
				  osWaitForever);
		calc_pack_voltage_stats(bmsdata);
		osThreadFlagsSet(calc_soc_thread, CALC_SOC_FLAG);
		osThreadFlagsSet(calc_dcl_thread, CALC_DCL_FLAG1);
		osDelay(1000);
	}
}

osThreadId_t calc_soc_thread;
const osThreadAttr_t calc_soc_attrs = { .name = "Get Segment Data",
					.stack_size = 2048,
					.priority = osPriorityNormal };
void vCalcSoC(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		osThreadFlagsWait(CALC_SOC_FLAG, osFlagsWaitAny, osWaitForever);
		calc_state_of_charge(bmsdata);
		osDelay(1000);
	}
}

osThreadId_t calc_therms_thread;
const osThreadAttr_t calc_therms_attrs = { .name = "Get Segment Data",
					   .stack_size = 2048,
					   .priority = osPriorityNormal };
void vCalcTherms(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		osThreadFlagsWait(CALC_THERMS_FLAG, osFlagsWaitAny,
				  osWaitForever);
		disable_therms(bmsdata);
		calc_cell_temps(bmsdata);
		calc_pack_temps(bmsdata);
		osThreadFlagsSet(calc_resistance_thread, CALC_RESIST_FLAG);
	}
}

osThreadId_t calc_resistance_thread;
const osThreadAttr_t calc_resistance_attrs = { .name = "Get Segment Data",
					       .stack_size = 2048,
					       .priority = osPriorityNormal };
void vCalcResistance(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		osThreadFlagsWait(CALC_RESIST_FLAG, osFlagsWaitAny,
				  osWaitForever);
		calc_cell_resistances(bmsdata);
		osThreadFlagsSet(calc_dcl_thread, CALC_DCL_FLAG2);
		osThreadFlagsSet(calc_ccl_thread, CALC_CCL_FLAG);
	}
}

osThreadId_t calc_dcl_thread;
const osThreadAttr_t calc_dcl_attrs = { .name = "Get Segment Data",
					.stack_size = 2048,
					.priority = osPriorityAboveNormal };
void vCalcDCL(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;
	for (;;) {
		osThreadFlagsWait(CALC_DCL_FLAG2 + CALC_DCL_FLAG1,
				  osFlagsWaitAll, osWaitForever);
		calc_dcl(bmsdata);
		calc_cont_dcl(bmsdata);
	}
}

osThreadId_t calc_ccl_thread;
const osThreadAttr_t calc_ccl_attrs = { .name = "Get Segment Data",
					.stack_size = 2048,
					.priority = osPriorityAboveNormal };
void vCalcCCL(void *pv_params)
{
	acc_data_t *bmsdata = (acc_data_t *)pv_params;

	for (;;) {
		osThreadFlagsWait(CALC_CCL_FLAG, osFlagsWaitAny, osWaitForever);
		calc_cont_ccl(bmsdata);
		bmsdata->charge_limit = bmsdata->cont_CCL;
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

osThreadId_t charging_thread;
const osThreadAttr_t charging_attrs = { .name = "Get Segment Data",
					.stack_size = 2048,
					.priority = osPriorityNormal };
void vCharging(void *pv_params)
{
	for (;;) {
		//TODO: all of it
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
	.name = "CanDispatch",
	.stack_size = 128 * 8,
	.priority = (osPriority_t)osPriorityRealtime5,
};

void vCanDispatch(void *pv_params)
{
	can_msg_t msg_from_queue;
	HAL_StatusTypeDef msg_status;

	for (;;) {
		osThreadFlagsWait(CAN_DISPATCH_FLAG, osFlagsWaitAny,
				  osFlagsWaitAny);
		/* Send CAN message */
		if (osOK == osMessageQueueGet(can_outbound_queue,
					      &msg_from_queue, NULL,
					      osWaitForever)) {
			msg_status = can_send_msg(&can1, &msg_from_queue);
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
