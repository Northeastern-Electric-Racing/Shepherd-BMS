#include "can_handler.h"
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define CAN_MSG_QUEUE_SIZE 50 /* messages */

#define CAN_DISPATCH_FLAG 1U

#define NEW_CAN_MSG_FLAG 1U

static osMessageQueueId_t can_outbound_queue;
static osMessageQueueId_t can_inbound_queue;

can_t *can1;
can_t *can2;

struct rl_bms_msgs_t rl_bms_msgs;

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

void init_can_msg(uint32_t id, uint8_t len)
{
	init_rl_can_msg(id, len, 0);
}

void init_rl_can_msg(uint32_t id, uint8_t len, uint32_t msg_rate)
{
	if (rl_bms_msgs.num_elements == rl_bms_msgs.capacity) {
		rl_can_msg_t *temp = (rl_can_msg_t *)malloc(
			sizeof(rl_can_msg_t) * rl_bms_msgs.num_elements);

		memcpy(temp, rl_bms_msgs.bms_can_msgs,
		       sizeof(rl_can_msg_t) * rl_bms_msgs.num_elements);

		// do you even gotta do this if ur mallocing later
		free(rl_bms_msgs.bms_can_msgs);

		rl_bms_msgs.capacity *= 2;
		rl_bms_msgs.bms_can_msgs =
			malloc(sizeof(rl_can_msg_t) * rl_bms_msgs.capacity);

		memcpy(rl_bms_msgs.bms_can_msgs, temp,
		       sizeof(rl_can_msg_t) * rl_bms_msgs.num_elements);
		free(temp);
	}
	rl_can_msg_t *msgptr =
		&rl_bms_msgs.bms_can_msgs[rl_bms_msgs.num_elements];
	msgptr->msg.id = id;
	msgptr->msg.len = len;
	msgptr->msg_rate = msg_rate;
}

void init_can_msg_config()
{
	rl_bms_msgs.bms_can_msgs = (rl_can_msg_t *)malloc(sizeof(rl_can_msg_t));
	rl_bms_msgs.capacity = 1;
	rl_bms_msgs.num_elements = 0;

	init_can_msg(DISCHARGE_CANID, 8);
	init_can_msg(CHARGE_CANID, 8);
	init_can_msg(ACC_STATUS_CANID, 8);
	init_can_msg(BMS_STATUS_CANID, 8);
	init_can_msg(SHUTDOWN_CTRL_CANID, 1);
	init_can_msg(CELL_DATA_CANID, 8);
	init_can_msg(CELL_VOLTAGE_CANID, 8);
	init_can_msg(CURRENT_CANID, 6);
	init_can_msg(CELL_TEMP_CANID, 8);
	init_can_msg(SEGMENT_TEMP_CANID, 6);
	init_can_msg(FAULT_CANID, 5);
	init_can_msg(NOISE_CANID, 6);
	init_can_msg(DEBUG_CANID, 8); // yaml decodes this to 8 bytes

	// TODO: Test
	init_rl_can_msg(DISCHARGE_CANID, 8, 5000);
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
