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

#define CALC_OCV_FLAG 1

#define CALC_NOISE_FLAG 1

#define CALC_VOLT_STATS_FLAG 1

#define CALC_SOC_FLAG 1

#define CALC_THERMS_FLAG 1

#define CALC_RESIST_FLAG 1

/* Set from calc voltage stats task */
#define CALC_DCL_FLAG1 0x00000001
/* Set from calc resistance task */
#define CALC_DCL_FLAG2 0x00000002

#define CALC_CCL_FLAG 0x00000001

#define STATE_MACHINE_FLAG 1

#define CAN_DISPATCH_FLAG 1

/**
 * @brief Task for retrieving segment data
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vGetSegmentData(void *pv_params);
extern osThreadId_t get_segment_data_thread;
extern const osThreadAttr_t get_segment_data_attrs;

/**
 * @brief Task for calculating OCV
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vCalcOCV(void *pv_params);
extern osThreadId_t calc_ocv_thread;
extern const osThreadAttr_t calc_ocv_attrs;

/**
 * @brief Task for calculating the percent of voltage readings that are noisy
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vCalcNoise(void *pv_params);
extern osThreadId_t calc_noise_thread;
extern const osThreadAttr_t calc_noise_attrs;

/**
 * @brief Task for calculating statistics about voltage readings
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vCalcVoltageStats(void *pv_params);
extern osThreadId_t calc_volt_stats_thread;
extern const osThreadAttr_t calc_volt_stats_attrs;

/**
 * @brief Task for calculating state of charge
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vCalcSoC(void *pv_params);
extern osThreadId_t calc_soc_thread;
extern const osThreadAttr_t calc_soc_attrs;

/**
 * @brief Task for calculating thermal data
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vCalcTherms(void *pv_params);
extern osThreadId_t calc_therms_thread;
extern const osThreadAttr_t calc_therms_attrs;

/**
 * @brief Task for calculating cell resistances
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vCalcResistance(void *pv_params);
extern osThreadId_t calc_resistance_thread;
extern const osThreadAttr_t calc_resistance_attrs;

/**
 * @brief Task for calculating DCL and continuous DCL
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vCalcDCL(void *pv_params);
extern osThreadId_t calc_dcl_thread;
extern const osThreadAttr_t calc_dcl_attrs;

/**
 * @brief Task for calculating CCL and continuous CCL
 * 
 * @param pv_params Pointer to acc_data_t struct containing BMS data
 */
void vCalcCCL(void *pv_params);
extern osThreadId_t calc_ccl_thread;
extern const osThreadAttr_t calc_ccl_attrs;

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
