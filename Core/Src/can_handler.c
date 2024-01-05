#include "can_handler.h"
#include "analyzer.h"
#include "ringbuffer.h"

void can_receive_callback(CAN_HandleTypeDef* hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	can_msg_t new_msg;
	/* Read in CAN message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, new_msg.data) != HAL_OK) {

		// TODO add non crtical fault capability - could create one for failed can receieve
		return;
	}

	new_msg.len = rx_header.DLC;
	new_msg.id	= rx_header.StdId;
	if (hcan == &hcan1) {
		ringbuffer_enqueue(can1_rx_queue, new_msg);
	} else {
		ringbuffer_enqueue(can2_rx_queue, new_msg);
	}
}

int8_t get_can1_msg(can_msg_t* msg)
{
	/* no messages to read */
	if (ringbuffer_is_empty(&can1_rx_queue))
		return -1;

	ringbuffer_dequeue(&can1_rx_queue, msg);

	// TODO list :
	// 1.
	switch (msg->id) {
	case NULL:
		break;
	}
	return 0;
}

int8_t get_can2_msg(can_msg_t* msg)
{
	/* no messages to read */
	if (ringbuffer_is_empty(&can2_rx_queue))
		return -1;

	ringbuffer_dequeue(&can2_rx_queue, msg);

	// TODO list :
	// 1. Charger connection flag -  have Charger set up with following logic, add correct CAN ID
	switch (msg->id) {
	case NULL:
		if (msg->data[0] == 0x01) {
			bmsdata->is_charger_connected = true;
		} else {
			bmsdata->is_charger_connected = false;
		}
		break;
	}
	return 0;
}
