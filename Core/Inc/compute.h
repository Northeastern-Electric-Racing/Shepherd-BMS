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

#define PORT_SHUTDOWN_3V3 Interlock_Read_GPIO_Port
#define PIN_SHUTDOWN_3V3  Interlock_Read_Pin // Pin PA2

/**
 * @brief Init all necessary peripherals on compute, minus CAN, see can_handler
 * 
 */
void compute_init();

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
 * @brief blinks debug LED.
 */
void toggle_debug_led();

/**
 * @brief turns LED on or off, for adc polling.A
 * 
 * @param mode
 */
void set_poll_led(int mode);

/**
 * @brief Checks if the shutdown circuit is open.
 * 
 * @return If the shutdown circuit is open, return true. If it is closed, return false.
 */
bool read_shutdown();

#endif // COMPUTE_H
