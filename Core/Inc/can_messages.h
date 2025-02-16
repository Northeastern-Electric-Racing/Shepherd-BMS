#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

#include "datastructs.h"

/**
 * @brief sends charger message
 *
 * @param voltage_to_set
 * @param currentToSet
 *
 * @return Returns a fault if we are not able to communicate with charger
 */
int send_charging_message(uint16_t voltage_to_set, uint16_t current_to_set,
			  acc_data_t *bms_data, bool charging_enabled);

/**
 * @brief Sends max discharge current to Motor Controller.
 *
 * @param bmsdata data structure containing the discharge limit
 */
void send_mc_discharge_message(acc_data_t *bmsdata);

/**
 * @brief sends max charge/discharge current to Motor Controller
 *
 * @param bmsdata
 */
void send_mc_charge_message(acc_data_t *bmsdata);

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
void send_acc_status_message(acc_data_t *bmsdata);

/**
 * @brief sends fault status message
 *
 * @param bms_state
 *
 */
void send_fault_status_message(acc_data_t *bmsdata);

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
void send_bms_status_message(acc_data_t *bmsdata, int bms_state, bool balance);

/**
 * @brief sends shutdown control message
 * @note unused
 *
 * @param mpe_state
 *
 * @return Returns a fault if we are not able to send
 */
void send_shutdown_ctrl_message(uint8_t mpe_state);

/**
 * @brief sends cell data message
 *
 * @param high_voltage
 * @param low_voltage
 * @param avg_voltage
 *
 * @return Returns a fault if we are not able to send
 */
void send_cell_voltage_message(acc_data_t *bmsdata);

/**
 * @brief sends out the calculated values of currents
 *
 * @param discharge
 * @param charge
 * @param current
 */
void send_current_message(acc_data_t *bmsdata);

/**
 * @brief sends cell temperature message
 *
 * @return Returns a fault if we are not able to send
 */
void send_cell_temp_message(acc_data_t *bmsdata);

/**
 * @brief sends the average segment temperatures
 *
 *
 *
 * @return Returns a fault if we are not able to send
 */
void send_segment_temp_message(acc_data_t *bmsdata);

void send_fault_message(uint8_t status, int16_t curr, int16_t in_dcl);

void send_fault_timer_message(uint8_t start_stop, uint32_t fault_code,
			      uint16_t data_1);

/**
 * @brief Send CAN message for debugging the car on the fly.
 * 
 * @param debug0 
 * @param debug1 
 * @param debug2 
 * @param debug3 
 */
void send_debug_message(uint8_t debug0, uint8_t debug1, uint16_t debug2,
			uint32_t debug3);

/**
 * @brief Send CAN message containing voltage noise data.
 * @note Unused
 * 
 * @param bmsdata 
 */
void send_voltage_noise_message(acc_data_t *bmsdata);

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
void send_cell_data_message(bool alpha, uint16_t temperature,
			    uint16_t voltage_a, uint16_t voltage_b,
			    uint8_t chip_ID, uint8_t cell_a, uint8_t cell_b,
			    bool discharging_a, bool discharging_b);

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
void send_beta_status_a_message(uint16_t cell_temperature, uint16_t voltage,
				bool discharging, uint8_t chip,
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
void send_beta_status_b_message(uint16_t vref2, uint16_t v_analog,
				uint16_t v_digital, uint8_t chip,
				uint16_t v_res, uint16_t vmv);

/**
 * @brief Send a message for the faults of beta chips.
 * TODO: remove thus
 * 
 * @param chip ID of chip
 * @param flt_reg  the fault data register
 */
void send_beta_status_c_message(uint8_t chip, stc_ *flt_reg);

/**
 * @brief Send message containing ADBMS6830 diagnostic data and onboard therm data.
 * 
 * @param segment_temp Temperature reading from on-board therm.
 * @param chip ID of the chip.
 * @param die_temperature Temperature of the ADBOS6830 die.
 * @param vpv The voltage from V+ to V-.
 * @param vmv Voltage between S1N and V-.
 * @param flt_reg The fault register of the chip (statc)
 */
void send_alpha_status_a_message(uint16_t segment_temp, uint8_t chip,
				 uint16_t die_temperature, uint16_t vpv,
				 uint16_t vmv, stc_ *flt_reg);

/**
 * @brief Send message containing ADBMS6830 diagnostic data.
 * 
 * @param v_res VREF2 across a resistor for open wire detection.
 * @param chip ID of the chip.
 * @param vref2 Second reference voltage for ADBMS6830.
 * @param v_analog Analog power supply voltage.
 * @param v_digital Digital power supply voltage.
 * @param flt_reg The fault register of the chip (statc)
 */
void send_alpha_status_b_message(uint16_t v_res, uint8_t chip, uint16_t vref2,
				 uint16_t v_analog, uint16_t v_digital,
				 stc_ *flt_reg);

#endif