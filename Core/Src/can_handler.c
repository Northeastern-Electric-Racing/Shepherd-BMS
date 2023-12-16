#include "can_handler.h"
#include "can.h"
#include "ringbuffer.h"

#define NUM_INBOUND_CAN1_IDS 1
#define NUM_INBOUND_CAN2_IDS 1

ringbuffer_t can_receive_queue;



static const uint16_t i can1_id_list[NUM_INBOUND_CAN1_IDS] = {
	//CANID_X,
	NULL
};

static const uint16_t i can2_id_list[NUM_INBOUND_CAN2_IDS] = {
	//CANID_X,
	NULL
};

void can_receive_callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    can_msg_t new_msg;
    /* Read in CAN message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, new_msg.data) != HAL_OK) {

        // TODO add non crtical fault capability - could create one for failed can receieve
        return;
    }

    new_msg.len = rx_header.DLC;
	new_msg.id = rx_header.StdId;

    ringbuffer_enqueue(can_receive_queue, new_msg);
}

uint8_t get_can_msg(can_msg_t* msg)
{
    /* no messages to read */
    if (ringbuffer_is_empty(&can_receive_queue)) return -1;

    ringbuffer_dequeue(&can_receive_queue, msg);
}
