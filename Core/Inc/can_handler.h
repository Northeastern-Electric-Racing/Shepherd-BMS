#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "can.h"

/**
 * @brief Callback to be called when a message is received on CAN line 1.
 * 
 * @param hcan Pointer to struct representing CAN hardware.
 */
#define NUM_INBOUND_CAN1_IDS 1
#define NUM_INBOUND_CAN2_IDS 1

#define CHARGE_CANID		   0x176
#define CHARGE_SIZE		   8
#define DISCHARGE_CANID		   0x156
#define DISCHARGE_SIZE		   8
#define ACC_STATUS_CANID	   0x80
#define ACC_STATUS_SIZE		   8
#define BMS_STATUS_CANID	   0x81
#define BMS_STATUS_SIZE		   4
#define FAULT_STATUS_CANID	   0x89
#define FAULT_STATUS_SIZE	   8
#define SHUTDOWN_CTRL_CANID	   0x82
#define SHUTDOWN_CTRL_SIZE	   1
#define CELL_DATA_CANID		   0x83
#define CELL_DATA_SIZE		   8
#define CELL_VOLTAGE_CANID	   0x87
#define CELL_VOLTAGE_SIZE	   8
#define CURRENT_SIZE		   6
#define CELL_TEMP_CANID		   0x84
#define CELL_TEMP_SIZE		   8
#define SEGMENT_TEMP_CANID	   0x85
#define SEGMENT_TEMP_SIZE	   5
#define SEGMENT_AVERAGE_VOLT_CANID 0x90
#define SEGMENT_AVERAGE_VOLT_SIZE  8
#define SEGMENT_TOTAL_VOLT_CANID   0x91
#define SEGMENT_TOTAL_VOLT_SIZE	   8
#define FAULT_CANID		   0x703 // TODO: cleanup
#define FAULT_SIZE		   5
#define NOISE_CANID		   0x88
#define NOISE_SIZE		   6
#define DEBUG_CANID		   0x702
#define CHARGER_CANID		   0x1806E5F4
#define CHARGERBOX_CANID	   0x18FF50E5
#define DTI_CURRENT_CANID	   0x436

#define OVERFLOW_CANID	   0x6F1
#define OVERFLOW_SIZE	   6
#define PEC_ERROR_CANID	   0x6F2
#define PEC_ERROR_SIZE	   3
#define ALPHA_CELL_CANID   0x6FA
#define BETA_CELL_CANID	   0x6FB
#define CELL_MSG_SIZE	   7
#define BETA_STAT_A_CANID  0x6FD
#define BETA_STAT_A_SIZE   8
#define BETA_STAT_B_CANID  0x6FE
#define BETA_STAT_B_SIZE   8
#define BETA_STAT_C_CANID  0x6F0
#define BETA_STAT_C_SIZE   3
#define ALPHA_STAT_A_CANID 0x6FC
#define ALPHA_STAT_A_SIZE  8
#define ALPHA_STAT_B_CANID 0x6FF
#define ALPHA_STAT_B_SIZE  8

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

#define DEBUG_SIZE	  8
#define FAULT_TIMER_CANID 0x6F9
#define FAULT_TIMER_SIZE  4

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
 * @param pv_params Pointer to acc_data_t struct containing BMS data
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

#endif // CAN_HANDLER_H
