#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "can.h"
#include "cmsis_os.h"

/**
 * @brief Callback to be called when a message is received on CAN line 1.
 * 
 * @param hcan Pointer to struct representing CAN hardware.
 */
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


typedef struct {
	uint32_t prev_tick;
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

void can_receive_callback(CAN_HandleTypeDef *hcan);

/**
 * @brief Place a CAN message in a queue.
 * 
 * @param msg CAN message to be sent.
 * @return int8_t Error code.
 */
int8_t queue_can_msg(can_msg_t msg);

/**
 * @brief Initialize CAN lines.
 * 
 */
void init_both_can(CAN_HandleTypeDef *hcan1, CAN_HandleTypeDef *hcan2);

/**
 * @brief Task for sending CAN messages.
 * 
 * @param pv_params CAN_HandleTypeDef for the CAN line that messages will be sent out on.
 */
void vCanDispatch(void *pv_params);
extern osThreadId_t can_dispatch_handle;
extern const osThreadAttr_t can_dispatch_attributes;

/**
 * @brief Task for processing received can messages.
 * 
 * @param pv_params A can_receive_args_t*.
 */
void vCanReceive(void *pv_params);
extern osThreadId_t can_receive_thread;
extern const osThreadAttr_t can_receive_attributes;

/**
 * returns the rate limit data based on the specific can  id
 */
rl_data_t *get_rl_msg(uint32_t can_id);

#endif // CAN_HANDLER_H