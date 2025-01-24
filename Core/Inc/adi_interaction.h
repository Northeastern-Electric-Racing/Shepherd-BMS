#include "adBms6830Data.h"
#include "bmsConfig.h"

// --- BEGIN SET HELPERS ---

/**
 * @brief Set the status of the REFON bit.
 * 
 * @param chip Pointer to the chip to modify.
 * @param state New state of the REFON bit.
 */
void set_REFON(cell_asic *chip, REFON state);

/**
 * @brief Set the C-ADC vs. S-ADC comparison voltage threshold 
 * 
 * @param chip Pointer to the chip to modify.
 * @param threshold Threshold to set.
 */
void set_volt_adc_comp_thresh(cell_asic *chip, CTH threshold);

void set_diagnostic_flags(cell_asic *chip, FLAG_D config);

/**
 * @brief Set the discharge state of a cell.
 * 
 * @param chip Pointer to chip with cell to modify.
 * @param cell ID of cell to modify. Cell indexes start are from 1-16 (NOT ZERO INDEXED).
 * @param discharge Cell discharge state. true to discharge, false to disable discharge.
 */
void set_cell_discharge(cell_asic *chip, uint8_t cell, bool discharge);

/**
 * @brief Set the state of the SOAKON bit to either enable or disable soak times.
 * 
 * @param chip Pointer to chip to configure
 * @param state Enable or disable SOAKON
 */
void set_soak_on(cell_asic *chip, SOAKON state);

/**
 * @brief Set the soak time range.
 * 
 * @param chip Pointer to chip to configure
 * @param range The range of time over which to soak for aux and aux2
 */
void set_aux_soak_range(cell_asic *chip, OWRNG range);

/**
 * @brief Set the open wire soak time. See data sheet for formula.
 * 
 * @param chip Pointer to chip configuration
 * @param time The amount of time to soak for. Higher OWA is a higher soak time.
 */
void set_open_wire_soak_time(cell_asic *chip, OWA time);

/**
 * @brief Set the mode of a GPIO pin on an ADBMS8630.
 * 
 * @param chip ADBMS6830 chip
 * @param gpio Number of the GPIO pin to change (1-10)
 * @param input True is input, False is output.
 */
void set_gpio_mode(cell_asic *chip, uint8_t gpio, bool input);

/**
 * @brief Set the corner frequency of the IIR filter.
 * 
 * @param chip Pointer to chip config
 * @param freq Corner frequency (see IIR_FPA enum for frequencies)
 */
void set_iir_corner_freq(cell_asic *chip, IIR_FPA freq);

/**
 * @brief Configure a chip as a break in the isoSPI daisy chain.
 * 
 * @param chip Pointer to chip config
 * @param is_break True if chip is break, false if chip is not break
 */
void set_comm_break(cell_asic *chip, bool is_break);

/**
 * @brief Enable/disable discharging through the mute discharge bit.
 * 
 * @param chip Pointer to chip config
 * @param disable_discharge True to disable discharge, false to enable discharge.
 */
void set_mute_state(cell_asic *chip, bool disable_discharge);

/**
 * @brief Set whether or not this chip is taking a snapshot. The chip will not begin reading new values unless the snapshot bit is cleared.
 * 
 * @param chip Pointer to chip config
 * @param take_snapshot True to take a snapshot, false to end the snapshot
 */
void set_snapshot(cell_asic *chip, bool take_snapshot);

/**
 * @brief Enable/disable the discharge timer monitor.
 * 
 * @param chip Pointer to chip config
 * @param enabled True if discharge timer monitor is enabled, false if otherwise
 */
void set_discharge_timer_monitor(cell_asic *chip, bool enabled);

/**
 * @brief Configure the discharge timer range, which affects the resolution.
 * 
 * @param chip Pointer to chip config
 * @param large True for large range, False for small range
 */
void set_discharge_timer_range(cell_asic *chip, bool large);

/**
 * @brief Set the discharge monitor timeout, which is dependent on the discharge timer range.
 * 
 * @param chip Pointer to chip config
 * @param timeout Base for timeout multiplicaiton. Must be below six bits.
 */
void set_discharge_timeout(cell_asic *chip, uint8_t timeout);

// --- END SET HELPERS ---

// --- BEGIN WRITE COMMANDS ---

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
