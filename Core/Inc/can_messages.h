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
int send_charging_message(uint16_t voltage_to_set, uint16_t current_to_set);

/**
 * @brief Sends max discharge current to Motor Controller.
 *
 * @param bmsdata data structure containing the discharge limit
 */
void send_mc_discharge_message(float discharge_limit);

/**
 * @brief sends max charge/discharge current to Motor Controller
 *
 * @param charge_limit
 */
void send_mc_charge_message(float charge_limit);

/**
 * @brief sends acc status message
 *
 * @param pack_voltage
 * @param pack_current
 * @param soc
 *
 * @return Returns a fault if we are not able to send
 */
void send_acc_status_message(float pack_voltage, float pack_current, float soc);

/**
 * @brief sends fault status message
 *
 * @param fault_code_crit
 * @param fault_code_noncrit
 *
 */
void send_fault_status_message(uint32_t fault_code_crit,
			       uint32_t fault_code_noncrit);

/**
 * @brief sends BMS status message
 *
 * @param avg_temp
 * @param bms_state
 * @param balance
 *
 * @return Returns a fault if we are not able to send
 */
void send_bms_status_message(float avg_temp, int bms_state, bool balance);

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
 * @param max_voltage
 * @param min_voltage
 * @param avg_voltage
 *
 * @return Returns a fault if we are not able to send
 */
void send_cell_voltage_message(crit_cellval_t max_voltage,
			       crit_cellval_t min_voltage, float avg_voltage);

/**
 * @brief sends cell temperature message
 *
 * @param max_temp
 * @param min_temp
 * @param avg_temp
 * 
 * @return Returns a fault if we are not able to send
 */
void send_cell_temp_message(crit_cellval_t max_temp, crit_cellval_t min_temp,
			    float avg_temp);

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
void send_cell_data_message(bool alpha, float temperature, float voltage_a,
			    float voltage_b, uint8_t chip_ID, uint8_t cell_a,
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
void send_beta_status_a_message(float cell_temperature, float voltage,
				bool discharging, uint8_t chip,
				float segment_temperature,
				float die_temperature, float vpv);

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
void send_beta_status_b_message(float vref2, float v_analog, float v_digital,
				uint8_t chip, float v_res, float vmv);

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
void send_alpha_status_a_message(float segment_temp, uint8_t chip,
				 float die_temperature, float vpv, float vmv,
				 stc_ *flt_reg);

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
void send_alpha_status_b_message(float v_res, uint8_t chip, float vref2,
				 float v_analog, float v_digital,
				 stc_ *flt_reg);

/**
 * @brief Sends a CAN message containing the PEC error count for a specific chip.
 *
 * @param chip_num The index of the chip that reported PEC errors.
 * @param pec_count The total number of PEC errors detected for the specified chip.
 */
void send_pec_error_message(uint8_t chip_num, uint16_t pec_count);

#endif