#ifndef COMPUTE_H
#define COMPUTE_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx.h"

#define CURRENT_SENSOR_PIN_L A1
#define CURRENT_SENSOR_PIN_H A0
#define MEAS_5VREF_PIN	     A7
#define FAULT_PIN	     2
#define CHARGE_SAFETY_RELAY  4
#define CHARGE_DETECT	     5
#define CHARGER_BAUD	     250000U
#define MC_BAUD		     1000000U
#define MAX_ADC_RESOLUTION   4095 // 12 bit ADC

typedef enum { FAN1, FAN2, FAN3, FAN4, FAN5, FAN6, FANMAX } fan_select_t;

/**
 * @brief inits the compute interface
 */
uint8_t compute_init();

/**
 * @brief Returns if charger interlock is engaged, indicating charger LV connector is plugged in
 *
 * @return true
 * @return false
 */
bool compute_charger_connected();

/**
 * @brief Handle any messages received from the charger
 *
 * @param msg
 */
//static void compute_charger_callback(const CAN_message_t& msg);

//static void compute_mc_callback(const CAN_message_t& msg);

/**
 * @brief Sets the desired fan speed
 * 
 * @param new_fan_speed 
 * @param fan_select 
 * 
 * @return uint8_t 0 = success, 1 = fan_select is out of range, 2 = PWM channel not able to be configured
 */
uint8_t compute_set_fan_speed(TIM_HandleTypeDef *pwmhandle,
			      fan_select_t fan_select, uint8_t duty_cycle);

/**
 * @brief Returns the pack current sensor reading
 *
 * @return int16_t
 */
int16_t compute_get_pack_current();

/**
 * @brief updates fault relay
 *
 * @param fault_state
 */
void compute_set_fault(int fault_state);

#endif // COMPUTE_H
