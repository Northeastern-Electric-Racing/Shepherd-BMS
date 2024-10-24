#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "can.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "ringbuffer.h"
#include "cmsis_os2.h"

#define NUM_INBOUND_CAN1_IDS 1
#define NUM_INBOUND_CAN2_IDS 1

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern ringbuffer_t *can1_rx_queue;
extern ringbuffer_t *can2_rx_queue;

#define CAN_MSG_QUEUE_SIZE 50 /* messages */
extern osMessageQueueId_t can_outbound_queue;

static const uint32_t can1_id_list[NUM_INBOUND_CAN1_IDS] = {
	//CANID_X,
	0x0000
};

static const uint32_t can2_id_list[NUM_INBOUND_CAN2_IDS] = {
	//CANID_X,
	0x18FF50E5
};

void can_receive_callback(CAN_HandleTypeDef *hcan);

/* for 1st CAN bus */
int8_t get_can1_msg();

/* for 2nd CAN bus */
int8_t get_can2_msg();

/**
 * @brief Push a CAN message into the outbound CAN queue.
 * 
 * @param msg The CAN message to queue
 * @return osStatus_t Result of queueing message
 */
osStatus_t queue_can_msg(can_msg_t msg);

#endif // CAN_HANDLER_H