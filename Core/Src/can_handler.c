#include "analyzer.h"
#include "ringbuffer.h"
#include "can_handler.h"
#include "shep_tasks.h"
#include <stdio.h>

ringbuffer_t *can1_rx_queue = NULL;
ringbuffer_t *can2_rx_queue = NULL;

can_msg_t bms_can_msgs[RL_MSG_COUNT];
rl_data_t rl_data[RL_MSG_COUNT];

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
	// TODO: Make receiving compatible with standard IDs
	new_msg.id = rx_header.ExtId;
	if (hcan == &hcan1) {
		ringbuffer_enqueue(can1_rx_queue, &new_msg);
	} else {
		ringbuffer_enqueue(can2_rx_queue, &new_msg);
	}
}

int8_t get_can1_msg()
{
	/* no messages to read */
	if (ringbuffer_is_empty(can1_rx_queue))
		return -1;

	can_msg_t msg;
	ringbuffer_dequeue(can1_rx_queue, &msg);

	// TODO list :
	// 1.
	switch (msg.id) {
	default:
		break;
	}
	return 0;
}

int8_t get_can2_msg()
{
	/* no messages to read */
	if (ringbuffer_is_empty(can2_rx_queue)) {
		return -1;
	}
	can_msg_t msg;
	ringbuffer_dequeue(can2_rx_queue, &msg);

	// TODO list :
	// 1. Charger connection flag -  have Charger set up with following logic, add correct CAN ID
	switch (msg.id) {
	/* CAN ID of message charger sends every second. */
	case 0x18FF50E5:
		// This doesn't work anyway
		// bmsdata->is_charger_connected = true;
		break;
	default:
		break;
	}
	return 0;
}

osStatus_t queue_can_msg(can_msg_t msg)
{
	if (!can_outbound_queue)
		return -1;

	rl_data_t *rl_data = get_rl_msg(msg.id);
	if (rl_data != NULL && is_timer_active(&rl_data->timer)) {
		return 0;
	} else {
		start_timer(&rl_data->timer, rl_data->msg_rate);
	}

	osStatus_t res = osMessageQueuePut(can_outbound_queue, &msg, 0U, 0U);

	if (res) {
		printf("CAN Queue full\r\n");
	}

	osThreadFlagsSet(can_dispatch_thread, CAN_DISPATCH_FLAG);

	return res;
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
	default:
		return NULL;
	}
}
