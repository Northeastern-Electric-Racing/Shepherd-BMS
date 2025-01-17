#ifndef COMPUTE_H
#define COMPUTE_H

#include "datastructs.h"
#include "stateMachine.h"

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
 * @brief sets safeguard bool to check whether charging is enabled or disabled
 *
 * @param is_enabled
 */
void compute_enable_charging(bool enable_charging);

/**
 * @brief sends charger message
 *
 * @param voltage_to_set
 * @param currentToSet
 *
 * @return Returns a fault if we are not able to communicate with charger
 */
int compute_send_charging_message(uint16_t voltage_to_set,
				  uint16_t current_to_set,
				  acc_data_t *bms_data);

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
 * @brief Sends max discharge current to Motor Controller.
 *
 * @param bmsdata data structure containing the discharge limit
 */
void compute_send_mc_discharge_message(acc_data_t *bmsdata);

/**
 * @brief sends max charge/discharge current to Motor Controller
 *
 * @param bmsdata
 */
void compute_send_mc_charge_message(acc_data_t *bmsdata);

/**
 * @brief updates fault relay
 *
 * @param fault_state
 */
void compute_set_fault(int fault_state);

/**
 * @brief sends acc status message
 *
 * @param voltage
 * @param current
 * @param ah
 * @param soc
 * @param health
 *
 * @return Returns a fault if we are not able to send
 */
void compute_send_acc_status_message(acc_data_t *bmsdata);

/**
 * @brief sends BMS status message
 *
 * @param bms_state
 * @param fault_status
 * @param tempAvg
 * @param tempInternal
 *
 * @return Returns a fault if we are not able to send
 */
void compute_send_bms_status_message(acc_data_t *bmsdata, int bms_state,
				     bool balance);

/**
 * @brief sends shutdown control message
 * @note unused
 *
 * @param mpe_state
 *
 * @return Returns a fault if we are not able to send
 */
void compute_send_shutdown_ctrl_message(uint8_t mpe_state);

/**
 * @brief sends cell data message
 *
 * @param high_voltage
 * @param low_voltage
 * @param avg_voltage
 *
 * @return Returns a fault if we are not able to send
 */
void compute_send_cell_voltage_message(acc_data_t *bmsdata);

/**
 * @brief sends out the calculated values of currents
 *
 * @param discharge
 * @param charge
 * @param current
 */
void compute_send_current_message(acc_data_t *bmsdata);

/**
 * @brief sends cell temperature message
 *
 * @return Returns a fault if we are not able to send
 */
void compute_send_cell_temp_message(acc_data_t *bmsdata);

/**
 * @brief sends the average segment temperatures
 *
 *
 *
 * @return Returns a fault if we are not able to send
 */
void compute_send_segment_temp_message(acc_data_t *bmsdata);

void compute_send_fault_message(uint8_t status, int16_t curr, int16_t in_dcl);

/**
 * @brief Send CAN message for debugging the car on the fly.
 * 
 * @param debug0 
 * @param debug1 
 * @param debug2 
 * @param debug3 
 */
void compute_send_debug_message(uint8_t debug0, uint8_t debug1, uint16_t debug2,
				uint32_t debug3);

/**
 * @brief Send CAN message containing voltage noise data.
 * @note Unused
 * 
 * @param bmsdata 
 */
void compute_send_voltage_noise_message(acc_data_t *bmsdata);

/**
 * @brief Send a message containing cell data.
 * 
 * @param alpha If this message contains alpha cell data. False sends a beta cell message.
 * @param temperature Temperature in Celsius. Has a maximum value of 80 degrees celsius.
 * @param voltage_a The voltage of cell A.
 * @param voltage_b The voltage of cell B.
 * @param chip_ID The chip ID.
 * @param cell_a The number of cell A.
 * @param cell_b The number of cell B.
 * @param discharging_a The state of cell A while balancing.
 * @param discharging_b The state of cell B while balancing.
 */
void compute_send_cell_data_message(bool alpha, uint16_t temperature,
				    uint16_t voltage_a, uint16_t voltage_b,
				    uint8_t chip_ID, uint8_t cell_a,
				    uint8_t cell_b, bool discharging_a,
				    bool discharging_b);

/**
 * @brief Send cell message containing Beta cell 10, the Beta onboard therm, the temperature of the ADBMS6830 die, and the voltage from V+ to V-.
 * 
 * @param cell_temperature Temperature of Beta cell 10.
 * @param voltage Voltage of Beta cell 10.
 * @param discharging Whether or not the cell is discharging.
 * @param chip The ID of the chip.
 * @param segment_temperature The output of the onboard therm.
 * @param die_temperature The temperature of the ADBMS6830 die.
 * @param vpv The voltage from V+ to V-.
 */
void compute_send_beta_status_a_message(uint16_t cell_temperature,
					uint16_t voltage, bool discharging,
					uint8_t chip,
					uint16_t segment_temperature,
					uint16_t die_temperature, uint16_t vpv);

/**
 * @brief Send message containing ADBMS6830 diagnostic data.
 * 
 * @param vref2 Second reference voltage for ADBMS6830.
 * @param v_analog Analog power supply voltage.
 * @param v_digital Digital power supply voltage.
 * @param chip ID of the chip.
 * @param v_res VREF2 across a resistor for open wire detection.
 * @param vmv Voltage between S1N and V-.
 */
void compute_send_beta_status_b_message(uint16_t vref2, uint16_t v_analog,
					uint16_t v_digital, uint8_t chip,
					uint16_t v_res, uint16_t vmv);

/**
 * @brief Send message containing ADBMS6830 diagnostic data and onboard therm data.
 * 
 * @param segment_temp Temperature reading from on-board therm.
 * @param chip ID of the chip.
 * @param die_temperature Temperature of the ADBOS6830 die.
 * @param vpv The voltage from V+ to V-.
 * @param vmv Voltage between S1N and V-.
 */
void compute_send_alpha_status_a_message(uint16_t segment_temp, uint8_t chip,
					 uint16_t die_temperature, uint16_t vpv,
					 uint16_t vmv);

/**
 * @brief Send message containing ADBMS6830 diagnostic data.
 * 
 * @param v_res VREF2 across a resistor for open wire detection.
 * @param chip ID of the chip.
 * @param vref2 Second reference voltage for ADBMS6830.
 * @param v_analog Analog power supply voltage.
 * @param v_digital Digital power supply voltage.
 */
void compute_send_alpha_status_b_message(uint16_t v_res, uint8_t chip,
					 uint16_t vref2, uint16_t v_analog,
					 uint16_t v_digital);

#endif // COMPUTE_H
