#include "can_handler.h"
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#define CAN_MSG_QUEUE_SIZE 50 /* messages */

#define CAN_DISPATCH_FLAG 1U

#define NEW_CAN_MSG_FLAG 1U

static osMessageQueueId_t can_outbound_queue;
static osMessageQueueId_t can_inbound_queue;

can_t *can1;
can_t *can2;

can_msg_t bms_can_msgs[RL_MSG_COUNT];
rl_data_t rl_data[RL_MSG_COUNT];

static uint32_t can1_id_list[] = {
	//CANID_X,
	0x002
};

static uint32_t can2_id_list[] = {
	//CANID_X,
	0x18FF50E5
};

osStatus_t queue_and_set_flag(osMessageQueueId_t queue, const void *msg_ptr,
			      osThreadId_t thread_id, uint32_t flags)
{
	osStatus_t status = osMessageQueuePut(queue, msg_ptr, 0U, 0U);
	if (status == osOK) {
		osThreadFlagsSet(thread_id, flags);
	} else {
		printf("Could not put item into queue: %d", status);
	}
	return status;
}

void init_can_msg_config()
{
	can_msg_t discharge_msg = { 0 };
	discharge_msg.id =
		DISCHARGE_CANID; // 0x0A is the dcl id, 0x22 is the device id set by us
	discharge_msg.len = 8;

	can_msg_t charge_msg = { 0 };
	charge_msg.id =
		CHARGE_CANID; // 0x0A is the dcl id, 0x157 is the device id set by us
	charge_msg.len = 8;

	can_msg_t acc_status_msg;
	acc_status_msg.id = ACC_STATUS_CANID;
	acc_status_msg.len = 8;

	can_msg_t bms_status_msg;
	bms_status_msg.id = BMS_STATUS_CANID;
	bms_status_msg.len = 4;

	can_msg_t fault_status_msg;
	fault_status_msg.id = FAULT_STATUS_CANID;
	fault_status_msg.len = 8;

	can_msg_t shutdown_ctrl_msg;
	shutdown_ctrl_msg.id = SHUTDOWN_CTRL_CANID;
	shutdown_ctrl_msg.len = 1;

	can_msg_t cell_data_msg;
	cell_data_msg.id = CELL_DATA_CANID;
	cell_data_msg.len = 8;

	can_msg_t cell_voltage_msg;
	cell_voltage_msg.id = CELL_VOLTAGE_CANID;
	cell_voltage_msg.len = 8;

	can_msg_t current_msg;
	current_msg.id = CURRENT_CANID;
	current_msg.len = 6;

	can_msg_t cell_temp_msg;
	cell_temp_msg.id = CELL_TEMP_CANID;
	cell_temp_msg.len = 8;

	can_msg_t segment_temp_msg;
	segment_temp_msg.id = SEGMENT_TEMP_CANID;
	segment_temp_msg.len = 6;

	can_msg_t fault_detail_msg;
	fault_detail_msg.id = FAULT_CANID;
	fault_detail_msg.len = 5;

	can_msg_t noise_msg;
	noise_msg.id = NOISE_CANID;
	noise_msg.len = 6;

	can_msg_t debug_msg;
	debug_msg.id = DEBUG_CANID;
	debug_msg.len = 8; // yaml decodes this to 8 bytes

	// rl_data_t rl_discharge_data = { .msg_rate = 5000 };
	// rl_data_t rl_charge_data = { .msg_rate = 0 };

	bms_can_msgs[DISCHARGE] = discharge_msg;
	bms_can_msgs[CHARGE] = charge_msg;
	bms_can_msgs[ACC_STATUS] = acc_status_msg;
	bms_can_msgs[BMS_STATUS] = bms_status_msg;
	bms_can_msgs[FAULT_STATUS] = fault_status_msg;
	bms_can_msgs[SHUTDOWN_CTRL] = shutdown_ctrl_msg;
	bms_can_msgs[CELL_DATA] = cell_data_msg;
	bms_can_msgs[CELL_VOLTAGE] = cell_voltage_msg;
	bms_can_msgs[CURRENT] = current_msg;
	bms_can_msgs[CELL_TEMP] = cell_temp_msg;
	bms_can_msgs[SEGMENT_TEMP] = segment_temp_msg;
	bms_can_msgs[FAULT] = fault_detail_msg;
	bms_can_msgs[NOISE] = noise_msg;
	bms_can_msgs[DEBUG] = debug_msg;

	// rl_data[DISCHARGE] = rl_discharge_data;
	// rl_data[CHARGE] = rl_charge_data;
}

rl_data_t *get_rl_msg(uint32_t can_id)
{
	switch (can_id) {
	case CHARGE_CANID:
		return &rl_data[CHARGE];
		break;
	case DISCHARGE_CANID:
		return &rl_data[DISCHARGE];
		break;
	case ACC_STATUS_CANID:
		return &rl_data[ACC_STATUS];
		break;
	case BMS_STATUS_CANID:
		return &rl_data[BMS_STATUS];
		break;
	case SHUTDOWN_CTRL_CANID:
		return &rl_data[SHUTDOWN_CTRL];
		break;
	case CELL_DATA_CANID:
		return &rl_data[CELL_DATA];
		break;
	case CELL_VOLTAGE_CANID:
		return &rl_data[CELL_VOLTAGE];
		break;
	case CURRENT_CANID:
		return &rl_data[CURRENT];
		break;
	case CELL_TEMP_CANID:
		return &rl_data[CELL_TEMP];
		break;
	case SEGMENT_TEMP_CANID:
		return &rl_data[SEGMENT_TEMP];
		break;
	case FAULT_CANID:
		return &rl_data[FAULT];
		break;
	case NOISE_CANID:
		return &rl_data[NOISE];
		break;
	case DEBUG_CANID:
		return &rl_data[DEBUG];
		break;
	default:
		break;
	}

	return NULL;
}

void init_both_can(CAN_HandleTypeDef *hcan1, CAN_HandleTypeDef *hcan2)
{
	assert(hcan1);
	assert(hcan2);

	can1 = malloc(sizeof(can_t));
	assert(can1);
	can2 = malloc(sizeof(can_t));
	assert(can2);

	can1->hcan = hcan1;

	uint32_t can1_id_list_size_four[4] = { can1_id_list[0], can1_id_list[0],
					       can1_id_list[0],
					       can1_id_list[0] };
	assert(!can_add_filter(can1, can1_id_list_size_four));
	assert(!can_init(can1));

	can2->hcan = hcan2;

	uint32_t can2_id_list_size_four[4] = { can2_id_list[0], can2_id_list[0],
					       can2_id_list[0],
					       can2_id_list[0] };
	assert(!can_add_filter(can2, can2_id_list_size_four));
	assert(!can_init(can2));

	can_outbound_queue =
		osMessageQueueNew(CAN_MSG_QUEUE_SIZE, sizeof(can_msg_t), NULL);
	can_inbound_queue =
		osMessageQueueNew(CAN_MSG_QUEUE_SIZE, sizeof(can_msg_t), NULL);

	init_can_msg_config();
}

void can_receive_callback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	can_msg_t new_msg;
	/* Read in CAN message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,
				 new_msg.data) != HAL_OK) {
		// TODO add non crtical fault capability - could create one for failed can receieve
		return;
	}
	new_msg.len = rx_header.DLC;

	if (hcan == can1->hcan) {
		new_msg.id = rx_header.StdId;
	} else {
		new_msg.id = rx_header.ExtId;
	}

	queue_and_set_flag(can_inbound_queue, &new_msg, can_receive_thread,
			   NEW_CAN_MSG_FLAG);
}

int8_t queue_can_msg(can_msg_t msg)
{
	if (!can_outbound_queue)
		return -1;

	rl_data_t *rl_data = get_rl_msg(msg.id);

	if (rl_data != NULL && rl_data->msg_rate != 0) {
		if (HAL_GetTick() <=
		    pdMS_TO_TICKS(rl_data->prev_tick) + rl_data->msg_rate) {
			// block message
			return 0;
		} else {
			rl_data->prev_tick = HAL_GetTick();
		}
	}

	return queue_and_set_flag(can_outbound_queue, &msg, can_dispatch_handle,
				  CAN_DISPATCH_FLAG);
}

osThreadId_t can_dispatch_handle;
const osThreadAttr_t can_dispatch_attributes = {
	.name = "CanDispatch",
	.stack_size = 128 * 8,
	.priority = (osPriority_t)osPriorityRealtime6,
};

void vCanDispatch(void *pv_params)
{
	can_msg_t msg_from_queue;
	HAL_StatusTypeDef msg_status;

	can_t *line;
#ifdef CHARGING
	line = can2;
#endif
	line = can1;

	for (;;) {
		osThreadFlagsWait(CAN_DISPATCH_FLAG, osFlagsWaitAny,
				  osWaitForever);

		/* Send all CAN messages in the queue */
		while (osOK == osMessageQueueGet(can_outbound_queue,
						 &msg_from_queue, NULL, 0)) {
			/* Wait if CAN outbound queue is full */
			while (HAL_CAN_GetTxMailboxesFreeLevel(line->hcan) ==
			       0) {
				osDelay(1);
			}

			msg_status = can_send_msg(can1, &msg_from_queue);

			if (msg_status != HAL_OK) {
				// temporary
				printf("CAN ERROR %d", msg_status);
			}
		}

		osDelay(1);
	}
}

osThreadId_t can_receive_thread;
const osThreadAttr_t can_receive_attributes = {
	.name = "CanProcessing",
	.stack_size = 128 * 8,
	.priority = (osPriority_t)osPriorityRealtime,
};

void vCanReceive(void *pv_params)
{
	can_msg_t msg;

	for (;;) {
		osThreadFlagsWait(NEW_CAN_MSG_FLAG, osFlagsWaitAny,
				  osWaitForever);
		while (osOK ==
		       osMessageQueueGet(can_inbound_queue, &msg, 0U, 0U)) {
			printf("RECIEVED MESSAGE: %lu", msg.id);
			switch (msg.id) {
			default:
				break;
			}
		}
	}
}
