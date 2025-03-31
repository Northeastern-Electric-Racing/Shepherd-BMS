#ifndef BMS_STATES_H
#define BMS_STATES_H

#include "analyzer.h"

/* global that can be read for debugging in main */
extern BMSState_t current_state;

/* global defined in segment.c that keeps an eye on the number of crc errors */
extern uint16_t crc_error_check;

#define NUM_FAULTS 9

/**
 * @brief Called when we receive a message from the charger
 * 
 * @param bmsdata
 * 
 */
void charger_message_recieved(acc_data_t *bmsdata);

/**
 * @brief Returns if we want to balance cells during a particular frame
 *
 * @param bmsdata
 * @return true
 * @return false
 */
bool sm_balancing_check(acc_data_t *bmsdata);

/**
 * @brief Returns if we want to charge cells during a particular frame
 *
 * @param bmsdata
 * @return true
 * @return false
 */
bool sm_charging_check(acc_data_t *bmsdata);

/**
 * @brief Returns any new faults or current faults that have come up
 * @note Should be bitwise OR'ed with the current fault status
 *
 * @param accData
 * @return uint64_t to be cast to a bms_fault_t
 */
uint64_t sm_fault_return(acc_data_t *accData);

/**
 * @brief Used in parellel to faultReturn(), calculates each fault to append the
 * fault status
 *
 * @param fault_item
 * @return fault_status
 */
bool sm_fault_eval(fault_eval_t *fault_item);

/**
 * @brief handles the state machine, calls the appropriate handler function and
 * runs every loop functions
 *
 * @param bmsdata
 */
void sm_handle_state(acc_data_t *bmsdata);

/**
 * @brief Algorithm behind determining which cells we want to balance
 * @note Directly interfaces with the segments
 *
 * @param bms_data
 */
void sm_balance_cells(acc_data_t *bms_data);

#endif // BMS_STATES_H