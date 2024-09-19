/**
 * @file tasks.h
 * @brief Define Shepherd's RTOS tasks. This file is most likely temporary.
 * @version 0.1
 * @date 2024-09-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef SHEP_TASKS_H
#define SHEP_TASKS_H

#include "cmsis_os2.h"

/**
 * @brief Task for retrieving segment data
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vGetSegmentData(void *pv_params);
extern osThreadId_t get_segment_data_thread;
extern const osThreadAttr_t get_segment_data_attrs;

/**
 * @brief Task for analyzing data after it is collected
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vAnalyzer(void *pv_params);
extern osThreadId_t analyzer_thread;
extern const osThreadAttr_t analyzer_attrs;

/**
 * @brief Task for reading the current sensor
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vCurrentMonitor(void *pv_params);
extern osThreadId_t current_monitor_thread;
extern const osThreadAttr_t current_monitor_attrs;

/**
 * @brief Task for handling state transitions
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vStateMachine(void *pv_params);
extern osThreadId_t state_machine_thread;
extern const osThreadAttr_t state_machine_attrs;

/**
 * @brief Task for sending CAN messages
 * 
 * @param pv_params NULL
 */
void vCanDispatch(void *pv_params);
extern osThreadId_t can_dispatch_thread;
extern const osThreadAttr_t can_dispatch_attrs;

#endif
