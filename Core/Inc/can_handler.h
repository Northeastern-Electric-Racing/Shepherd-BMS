#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "can.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>


#define NUM_INBOUND_CAN1_IDS 1
#define NUM_INBOUND_CAN2_IDS 1

static const uint16_t can1_id_list[NUM_INBOUND_CAN1_IDS] = {
	//CANID_X,
	0x0000
};

static const uint16_t can2_id_list[NUM_INBOUND_CAN2_IDS] = {
	//CANID_X,
	0x0000
};

/* Shepherd hanbdles both can busses the same way */
void can_receive_callback(CAN_HandleTypeDef *hcan);

uint8_t get_can_msg(can_msg_t* msg);

#endif // CAN_HANDLER_H