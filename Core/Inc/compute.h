#ifndef COMPUTE_H
#define COMPUTE_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32xx_hal.h"

#define CURRENT_SENSOR_PIN_L A1
#define CURRENT_SENSOR_PIN_H A0
#define MEAS_5VREF_PIN	     A7
#define FAULT_PIN	     2
#define CHARGE_SAFETY_RELAY  4
#define CHARGE_DETECT	     5
#define CHARGER_BAUD	     250000U
#define MC_BAUD		     1000000U
#define MAX_ADC_RESOLUTION   4095 // 12 bit ADC

/**
 * @brief Init all necessary peripherals on compute, minus CAN, see can_handler
 * 
 */
void compute_init();

/**
 * @brief Measure the onboard temperature
 * 
 * @param temp the temperature
 * @param humidity the humidity
 * @return int8_t the error status
 */
int8_t compute_measure_temp(float *temp, float *humidity);

/**
 * @brief Returns the pack current sensor reading
 *
 * @return float
 */
float compute_get_pack_current();

/**
 * @brief updates fault relay
 *
 * @param fault_state
 */
void compute_set_fault(bool fault_state);

/**
 * @brief blinks LED 1.
 */
void toggle_debug_led_1();

/**
 * @brief turns LED 2 on or off,
 * 
 * @param mode
 */
void set_debug_led_2(int mode);

/**
 * @brief Pets the external watchdog
 * 
 */
void pet_watchdog();

/**
 * @brief Checks if the shutdown circuit is open.
 * 
 * @return If the shutdown circuit is open, return true. If it is closed, return false.
 */
bool read_shutdown();

#endif // COMPUTE_H
