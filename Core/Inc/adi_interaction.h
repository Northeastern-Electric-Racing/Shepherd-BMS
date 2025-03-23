#ifndef ADI_INTERACTION_H
#define ADI_INTERACTION_H

#include "adBms6830Data.h"
#include "bmsConfig.h"

// --- BEGIN SET HELPERS ---

/**
 * @brief Set the status of the REFON bit.
 * WARNING, THE ENUM IS WRONG, CHECK TABLE 102
 * 
 * Config A
 * 
 * @param chip Pointer to the chip to modify.
 * @param state New state of the REFON bit.
 */
void set_REFON(cell_asic *chip, REFON state);

/**
 * @brief Set the C-ADC vs. S-ADC comparison voltage threshold 
 * 
 * Config A
 * 
 * @param chip Pointer to the chip to modify.
 * @param threshold Threshold to set.
 */
void set_volt_adc_comp_thresh(cell_asic *chip, CTH threshold);

/**
 * @brief Set the diagnostic flags 
 * 
 * Config A
 * 
 * @param chip Pointer to the chip to modify.
 * @param config the type of diagnostic flag to set
 * @param state the state of the diagnostic flag choosen
 */
void set_diagnostic_flags(cell_asic *chip, FLAG_D config, CFGA_FLAG state);
/**
 * @brief Clear all diagnostic flags
 * 
 * Config A
 * 
 * @param chip Pointer to the chip to modify.
 */
void clear_diagnostic_flags(cell_asic *chip);

/**
 * @brief Set the discharge state of a cell.
 * 
 * Config B
 * 
 * @param chip Pointer to chip with cell to modify.
 * @param cell ID of cell to modify.
 * @param discharge Cell discharge state.
 */
void set_cell_discharge(cell_asic *chip, DCC cell, DCC_BIT discharge);
/**
 * @brief Clear the discharge state of the cell (turn off dcc)
 * 
 * Config B
 * 
 * @param chip Pointer to chip with cell to modify.
 */
void clear_cell_discharge(cell_asic *chip);


/**
 * @brief Set the state of the SOAKON bit to either enable or disable soak times.
 * 
 * Config A
 * 
 * @param chip Pointer to chip to configure
 * @param state Enable or disable SOAKON
 */
void set_soak_on(cell_asic *chip, SOAKON state);

/**
 * @brief Set the soak time range.
 * 
 * Config A
 * 
 * @param chip Pointer to chip to configure
 * @param range The range of time over which to soak for aux and aux2
 */
void set_aux_soak_range(cell_asic *chip, OWRNG range);

/**
 * @brief Set the open wire soak time. See data sheet for formula.
 * 
 * Config A
 * 
 * @param chip Pointer to chip configuration
 * @param time The amount of time to soak for. Higher OWA is a higher soak time.
 */
void set_open_wire_soak_time(cell_asic *chip, OWA time);

/**
 * @brief Set the pull of a GPIO pin on an ADBMS8630.
 * 
 * Config A
 * 
 * @param chip ADBMS6830 chip
 * @param gpio GPIO pin to change
 * @param input Whether to set the pulldown
 */
void set_gpio_pull(cell_asic *chip, GPO gpio, CFGA_GPO input);

/**
 * @brief Set the corner frequency of the IIR filter.
 * 
 * Config A
 * 
 * @param chip Pointer to chip config
 * @param freq Corner frequency (see IIR_FPA enum for frequencies)
 */
void set_iir_corner_freq(cell_asic *chip, IIR_FPA freq);

/**
 * @brief Configure a chip as a break in the isoSPI daisy chain.
 * 
 * Config A
 * 
 * @param chip Pointer to chip config
 * @param is_break whether to break the comms at that chip
 */
void set_comm_break(cell_asic *chip, COMM_BK is_break);

/**
 * @brief Enable/disable discharging through the mute discharge bit.
 * 
 * Config A
 * 
 * @param chip Pointer to chip config
 * @param disable_discharge True to disable discharge, false to enable discharge.
 */
void set_mute_state(cell_asic *chip, bool disable_discharge);

/**
 * @brief Set whether or not this chip is taking a snapshot. The chip will not begin reading new values unless the snapshot bit is cleared.
 * 
 * Config A
 * 
 * @param chip Pointer to chip config
 * @param take_snapshot whether to take a snapshot
 */
void set_snapshot(cell_asic *chip, SNAPSHOT take_snapshot);

/**
 * @brief Enable/disable the discharge timer monitor.
 * 
 * Config B
 * 
 * @param chip Pointer to chip config
 * @param enabled whether to enable the discharge monitor
 */
void set_discharge_timer_monitor(cell_asic *chip, DTMEN enabled);

/**
 * @brief Configure the discharge timer range, which affects the resolution.
 * 
 * Config B
 * 
 * @param chip Pointer to chip config
 * @param large range to set
 */
void set_discharge_timer_range(cell_asic *chip, DTRNG large);

/**
 * @brief Set the discharge monitor timeout, which is dependent on the discharge timer range.
 * 
 * Config B
 * 
 * @param chip Pointer to chip config
 * @param timeout Timeout to set, dependent on `set_discharge_timer_range`
 */
void set_discharge_timeout(cell_asic *chip, DCTO timeout);

// --- END SET HELPERS ---

// --- BEGIN WRITE COMMANDS ---

/**
 * @brief Soft reset all chips, then re-wake them
 * 
 * @param chips 
 */
void soft_reset_chips(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Write config registers. Wakes chips before writing.
 * 
 * @param chips Array of chips to write config registers of.
 */
void write_config_regs(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Clears all status regster C flags except the CS FLT
 * 
 * @param chips 
 */
void write_clear_flags(cell_asic chips[NUM_CHIPS]);

// --- END WRITE COMMANDS ---

// --- BEGIN READ COMMANDS ---

/**
 * @brief Read all filtered voltage results.  IIR must be on and ADC must be continous
 * 
 * @param chips The chips to read voltages from
 */
void read_filtered_voltage_registers(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Read every register connected to the AUX ADC.
 * 
 * @param chips Array of chips to get voltage readings of.
 */
void adc_and_read_aux_registers(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Read voltages in every register connected to AUX2 ADC.
 * 
 * @param chips Array of chips to get voltages of.
 */
void adc_and_read_aux2_registers(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Read status registers.
 * 
 * @param chips Array of chips to read.
 */
void read_status_registers(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Read status register c, containing chip level faults.
 * 
 * @param chips 
 */
void read_status_register_c(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Read status and aux registers in one command.
 * 
 * @param chips Array of chips to read.
 */
void read_status_aux_registers(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Read the serial ID of the chip.
 * 
 * @param chips Array of chips to read.
 */
void read_serial_id(cell_asic chips[NUM_CHIPS]);

// --- END READ COMMANDS ---

// --- BEGIN ADC POLL ---

/**
 * @brief Get voltage readings from the C-ADCs. Takes a single shot measurement.
 * 
 * @param chips Array of chips to get voltage readings from.
 */
void get_c_adc_voltages(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Get voltages from the S-ADCs. Makes a single shot measurement.
 * 
 * @param chip Array of chips to get voltage readings from.
 */
void get_s_adc_voltages(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Get the avgeraged cell voltages.
 * 
 * @param chip Array of chips to get voltage readings of.
 */
void get_avgd_cell_voltages(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Get the filtered cell volrages.
 * 
 * @param chip Array of chips to get voltage readings of.
 */
void get_filtered_cell_voltages(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Get the c and s adc voltages. Does this with RDCSALL command.
 * 
 * @param chips Array of chips to get voltage readings of.
 */
void get_c_and_s_adc_voltages(cell_asic chips[NUM_CHIPS]);

/**
 * @brief Starts a continous c ADC conversion with S redundancy
 * 
 */
void start_c_adc_conv();

// --- END ADC POLL ---

#endif