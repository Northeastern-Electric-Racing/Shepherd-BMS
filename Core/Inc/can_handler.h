#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "can.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "ringbuffer.h"
#include "FreeRTOS.h"
#include "datastructs.h"

#define NUM_INBOUND_CAN1_IDS 1
#define NUM_INBOUND_CAN2_IDS 1

#define CHARGE_CANID	    0x176
#define DISCHARGE_CANID	    0x156
#define ACC_STATUS_CANID    0x80
#define BMS_STATUS_CANID    0x81
#define SHUTDOWN_CTRL_CANID 0x82
#define CELL_DATA_CANID	    0x83
#define CELL_VOLTAGE_CANID  0x87
#define CURRENT_CANID	    0x86
#define CELL_TEMP_CANID	    0x84
#define SEGMENT_TEMP_CANID  0x85
#define FAULT_CANID	    0x703
#define NOISE_CANID	    0x88
#define DEBUG_CANID	    0x702

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern ringbuffer_t *can1_rx_queue;
extern ringbuffer_t *can2_rx_queue;

#define CAN_MSG_QUEUE_SIZE 50 /* messages */
extern osMessageQueueId_t can_outbound_queue;

typedef struct {
	nertimer_t timer;
	uint32_t msg_rate; /* in milliseconds */
} rl_data_t;

typedef enum {
	CHARGE,
	DISCHARGE,
	ACC_STATUS,
	BMS_STATUS,
	SHUTDOWN_CTRL,
	CELL_DATA,
	CELL_VOLTAGE,
	CURRENT,
	CELL_TEMP,
	SEGMENT_TEMP,
	FAULT,
	NOISE,
	DEBUG,
	RL_MSG_COUNT
} rate_lim_t;

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

/**
 * returns the rate limit data based on the specific can  id
 */
rl_data_t *get_rl_msg(uint32_t can_id);

#endif // CAN_HANDLER_H