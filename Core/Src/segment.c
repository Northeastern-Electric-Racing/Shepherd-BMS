#include "segment.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "analyzer.h"
#include "c_utils.h"

#include "common.h"
#include "adBms6830CmdList.h"
#include "adBms6830GenericType.h"
#include "serialPrintResult.h"
#include "adBms6830ParseCreate.h"
#include "mcuWrapper.h"
#include "cmsis_os.h"

#define T_READY 10 /* microseconds*/
#define T_IDLE	4.3 /* milliseconds, minimum. typ is 5.5, max is 6.7 */
#define T_WAKE	200 /* microseconds */
#define T_SLEEP 1.8 /* seconds minimum, typ is 2, max is 2.2 */
#define T_REFUP 2.7 /* milliseconds minimum, typ is 3.5, max is 4.4 */

#define THERM_WAIT_TIME	   500 /* ms */
#define VOLTAGE_WAIT_TIME  500 /* ms */
#define THERM_AVG	   15 /* Number of values to average */
#define MAX_VOLT_DELTA	   2500
#define MAX_CONSEC_NOISE   10
#define GPIO_EXPANDER_ADDR 0x40
#define GPIO_REGISTER_ADDR 0x09

// TODO ensure spi 1 is correct for talking to segs
extern SPI_HandleTypeDef hspi1;

uint8_t therm_avg_counter = 0;

chipdata_t previous_data[NUM_CHIPS] = {};

nertimer_t variance_timer;

uint32_t pec_error_count = 0;

/* private function prototypes */
void variance_therm_check(void);
void discard_neutrals(chipdata_t segment_data[NUM_CHIPS]);
void pull_chip_configuration(void);
int16_t calc_average(chipdata_t segment_data[NUM_CHIPS]);
int8_t calc_therm_standard_dev(int16_t avg_temp);
void init_chip(cell_asic *chip);
void set_cell_discharge(cell_asic *chip, uint8_t cell, bool discharge);

/**
 * @brief Set a bit in a uint16
 *  
 * @param number uint16 to change.
 * @param n Nth bit to change.
 * @param x true sets, false clears.
 * @return uint16_t New uint16.
 */
inline uint16_t set_uint16_bit(uint16_t number, uint16_t n, bool x)
{
	return (number & ~((uint16_t)1 << n)) | ((uint16_t)x << n);
}

/**
 * @brief Wake every ADBMS6830 IC in the daisy chain. Takes NUM_CHIPS * 8 ms to finish.
 * 
 */
void adbms_wake()
{
	adBmsCsLow();
	adBmsCsHigh();

	/* 
	DEBUG: Theoretically, below should work for one IC, but it is not. 
	It shold be necessarry for multiple IC operation.
	*/

	// for (uint8_t ic = 0; ic < total_ic; ic++) {
	// 	adBmsCsLow();
	// 	Delay_ms(4);
	// 	adBmsCsHigh();
	// 	Delay_ms(4);
	// }

	// adBmsWakeupIc(NUM_CHIPS);
}

/**
 * @brief Set the status of the REFON bit.
 * 
 * @param chip Pointer to the chip to modify.
 * @param state New state of the REFON bit.
 */
void set_REFON(cell_asic *chip, REFON state)
{
	chip->tx_cfga.refon = state;
}

/**
 * @brief Set the C-ADC vs. S-ADC comparison voltage threshold 
 * 
 * @param chip Pointer to the chip to modify.
 * @param threshold Threshold to set.
 */
void set_volt_adc_comp_thresh(cell_asic *chip, CTH threshold)
{
	chip->tx_cfga.cth = threshold;
}

void set_diagnostic_flags(cell_asic *chip, FLAG_D config)
{
	chip->tx_cfga.flag_d =
		(uint8_t)set_uint16_bit(chip->tx_cfga.flag_d, config, true);
}

/**
 * @brief Set the discharge state of a cell.
 * 
 * @param chip Pointer to chip with cell to modify.
 * @param cell ID of cell to modify. Cell indexes start are from 1-16 (NOT ZERO INDEXED).
 * @param discharge Cell discharge state. true to discharge, false to disable discharge.
 */
void set_cell_discharge(cell_asic *chip, uint8_t cell, bool discharge)
{
	chip->tx_cfgb.dcc = set_uint16_bit(chip->tx_cfgb.dcc, cell, discharge);
}

/**
 * @brief Set the state of the SOAKON bit to either enable or disable soak times.
 * 
 * @param chip Pointer to chip to configure
 * @param state Enable or disable SOAKON
 */
void set_soak_on(cell_asic *chip, SOAKON state)
{
	chip->tx_cfga.soakon = state;
}

/**
 * @brief Set the open wire detection soak time range.
 * 
 * @param chip Pointer to chip to configure
 * @param range The range of time over which to soak for open wire detection
 */
void set_open_wire_soak_range(cell_asic *chip, OWRNG range)
{
	chip->tx_cfga.owrng = range;
}

/**
 * @brief Set the open wire soak time. See data sheet for formula.
 * 
 * @param chip Pointer to chip configuration
 * @param time The amount of time to soak for. Higher OWA is a higher soak time.
 */
void set_open_wire_soak_time(cell_asic *chip, OWA time)
{
	chip->tx_cfga.owa = time;
}

/**
 * @brief Set the mode of a GPIO pin on an ADBMS8630.
 * 
 * @param chip ADBMS6830 chip
 * @param gpio Number of the GPIO pin to change (1-10)
 * @param input True is input, False is output.
 */
void set_gpio_mode(cell_asic *chip, uint8_t gpio, bool input)
{
	if (gpio > 10 || gpio < 1) {
		printf("ERROR: Invalid GPIO pin %d\n", gpio);
		return;
	}
	chip->tx_cfga.gpo = set_uint16_bit(chip->tx_cfga.gpo, gpio - 1, input);
}

/**
 * @brief Set the corner frequency of the IIR filter.
 * 
 * @param chip Pointer to chip config
 * @param freq Corner frequency (see IIR_FPA enum for frequencies)
 */
void set_iir_corner_freq(cell_asic *chip, IIR_FPA freq)
{
	chip->tx_cfga.fc = freq;
}

/**
 * @brief Configure a chip as a break in the isoSPI daisy chain.
 * 
 * @param chip Pointer to chip config
 * @param is_break True if chip is break, false if chip is not break
 */
void set_comm_break(cell_asic *chip, bool is_break)
{
	chip->tx_cfga.comm_bk = is_break;
}

/**
 * @brief Enable/disable discharging through the mute discharge bit.
 * 
 * @param chip Pointer to chip config
 * @param disable_discharge True to disable discharge, false to enable discharge.
 */
void set_mute_state(cell_asic *chip, bool disable_discharge)
{
	chip->tx_cfga.mute_st = disable_discharge;
}

/**
 * @brief Set whether or not this chip is taking a snapshot. The chip will not begin reading new values unless the snapshot bit is cleared.
 * 
 * @param chip Pointer to chip config
 * @param take_snapshot True to take a snapshot, false to end the snapshot
 */
void set_snapshot(cell_asic *chip, bool take_snapshot)
{
	chip->tx_cfga.snap = take_snapshot;
}

/**
 * @brief Enable/disable the discharge timer monitor.
 * 
 * @param chip Pointer to chip config
 * @param enabled True if discharge timer monitor is enabled, false if otherwise
 */
void set_discharge_timer_monitor(cell_asic *chip, bool enabled)
{
	chip->tx_cfgb.dtmen = enabled;
}

/**
 * @brief Configure the discharge timer range, which affects the resolution.
 * 
 * @param chip Pointer to chip config
 * @param large True for large range, False for small range
 */
void set_discharge_timer_range(cell_asic *chip, bool large)
{
	chip->tx_cfgb.dtrng = large;
}

/**
 * @brief Set the discharge monitor timeout, which is dependent on the discharge timer range.
 * 
 * @param chip Pointer to chip config
 * @param timeout Base for timeout multiplicaiton. Must be below six bits.
 */
void set_discharge_timeout(cell_asic *chip, uint8_t timeout)
{
	if (timeout >> 6 > 0) {
		printf("Invalid discharge time\n");
		return;
		//TODO: Non-critical fault
	}
	chip->tx_cfgb.dcto = timeout;
}

/**
 * @brief Initialize a chip with default values. 
 * 
 * @param chip Pointer to chip to initialize.
 */
void init_chip(cell_asic *chip)
{
	set_REFON(chip, PWR_UP);
	set_volt_adc_comp_thresh(chip, CVT_8_1mV);
	chip->tx_cfga.flag_d = 0;

	// No soak on AUX ADCs
	set_soak_on(chip, SOAKON_CLR);

	// short soak time by default
	set_open_wire_soak_range(chip, TIME_32US_TO_4_1MS);

	// No open wire detect soak
	set_open_wire_soak_time(chip, OWA0);

	// All GPIOs are inputs by default
	set_gpio_mode(chip, 1, true);
	set_gpio_mode(chip, 2, true);
	set_gpio_mode(chip, 3, true);
	set_gpio_mode(chip, 4, true);
	set_gpio_mode(chip, 5, true);
	set_gpio_mode(chip, 6, true);
	set_gpio_mode(chip, 7, true);
	set_gpio_mode(chip, 8, true);
	set_gpio_mode(chip, 9, true);
	set_gpio_mode(chip, 10, true);

	// Registers are unfrozen
	set_snapshot(chip, SNAP_OFF);

	// Charging is deactivated
	set_mute_state(chip, true);

	// Not an endpoint in the daisy chain
	set_comm_break(chip, false);

	// IIR filter disabled
	set_iir_corner_freq(chip, IIR_FPA_OFF);

	// Init config B

	// If the corresponding fault bits are sent high, it does not affect the IC
	chip->tx_cfgb.vov = SetOverVoltageThreshold(4.2);
	chip->tx_cfgb.vuv = SetUnderVoltageThreshold(3.0);

	// Discharge timer monitor off
	set_discharge_timer_monitor(chip, false);

	// Set discharge timer range to 0 to 63 minutes with 1 minute increments
	set_discharge_timer_range(chip, RANG_0_TO_63_MIN);

	// Disable discharge for all cells
	chip->tx_cfgb.dcc = 0;
}

/**
 * @brief Write data to all chips.
 * 
 * @param chip Array of chips to write data to.
 * @param command Command to issue to the chip.
 * @param type Register type to write to.
 * @param group Group of registers to write to.
 */
void write_adbms_data(cell_asic chips[NUM_CHIPS], uint8_t command[2], TYPE type,
		      GRP group)
{
	adBmsWriteData(NUM_CHIPS, chips, command, type, group);
}

/**
 * @brief Read data from all chips.
 * 
 * @param chips Array of chips to read data to.
 * @param command Command to issue to the chip.
 * @param type Register type to write to.
 * @param group Group of registers to write to.
 */
void read_adbms_data(cell_asic chips[NUM_CHIPS], uint8_t command[2], TYPE type,
		     GRP group)
{
	adBmsReadData(NUM_CHIPS, chips, command, type, group);

	// Count PEC errors
	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
		// Yes, they did separate every PEC as if that mattered.
		pec_error_count +=
			chips[chip].cccrc.cfgr_pec + chips[chip].cccrc.sid_pec +
			chips[chip].cccrc.cell_pec +
			chips[chip].cccrc.acell_pec +
			chips[chip].cccrc.scell_pec +
			chips[chip].cccrc.fcell_pec +
			chips[chip].cccrc.aux_pec + chips[chip].cccrc.raux_pec +
			chips[chip].cccrc.stat_pec +
			chips[chip].cccrc.comm_pec + chips[chip].cccrc.pwm_pec;
		if (pec_error_count > 0) {
			printf("PEC COUNT: %ld\n", pec_error_count);
		}
	}
}

/**
 * @brief Write config registers. Wakes chips before writing.
 * 
 * @param chips Array of chips to write config registers of.
 */
void write_config_regs(cell_asic chips[NUM_CHIPS])
{
	adbms_wake();
	write_adbms_data(chips, WRCFGA, Config, A);
	write_adbms_data(chips, WRCFGB, Config, B);
}

/**
 * @brief Initialize chips with default values.
 * 
 */
void segment_init(acc_data_t *bmsdata)
{
	printf("Initializing Segments...");
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		init_chip(&bmsdata->chips[chip]);
		// TODO: Make sure this is accurate
		bmsdata->chip_data->alpha = chip % 2 == 0;
	}
	write_config_regs(bmsdata->chips);
}

/**
 * @brief Get voltage readings from the C-ADCs. Takes a single shot measurement.
 * 
 * @param chips Array of chips to get voltage readings from.
 */
void get_c_adc_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);

	// Take single shot measurement
	adBms6830_Adcv(RD_OFF, SINGLE, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);
	read_adbms_data(chips, RDCVALL, Rdcvall, ALL_GRP);
}

/**
 * @brief Get voltages from the S-ADCs. Makes a single shot measurement.
 * 
 * @param chip Array of chips to get voltage readings from.
 */
void get_s_adc_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake();
	adBms6830_Adsv(SINGLE, DCP_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLSADC);

	adbms_wake();
	read_adbms_data(chips, RDSALL, Rdsall, ALL_GRP);
	// read_adbms_data(chip, RDSVA, S_volt, A);
	// read_adbms_data(chip, RDSVB, S_volt, B);
	// read_adbms_data(chip, RDSVC, S_volt, C);
	// read_adbms_data(chip, RDSVD, S_volt, D);
	// read_adbms_data(chip, RDSVE, S_volt, E);
	// read_adbms_data(chip, RDSVF, S_volt, F);
}

/**
 * @brief Do a single shot, redundant C-ADC measurement and read
 * the contents of Status Register Group C, which contains the 
 * CSxFLT bits indicating whether the difference between the 
 * C and S ADC measurements was above the CTH[2:0] set in config
 * register A.
 * 
 * @param chips Pointer to accumulator data struct.
 */
void get_adc_comparison(acc_data_t *bmsdata)
{
	write_config_regs(bmsdata->chips);

	// Take single shot measurement
	adBms6830_Adcv(RD_ON, SINGLE, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);
	read_adbms_data(bmsdata->chips, RDCVALL, Rdcvall, ALL_GRP);

	// Result of C-ADC and S-ADC comparison is stored in status register group C
	read_adbms_data(bmsdata->chips, RDSTATC, Status, C);

	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
		uint8_t cells = get_num_cells(&bmsdata->chip_data[chip]);
		for (uint8_t cell = 0; cell < cells; cell++) {
			if (NER_GET_BIT(bmsdata->chips[chip].statc.cs_flt,
					cell)) {
				printf("ADC VOLTAGE DISCREPANCY ERROR\nChip %d, Cell %d\nC-ADC: %f, S-ADC%f\n",
				       chip + 1, cell + 1,
				       getVoltage(bmsdata->chips[chip]
							  .cell.c_codes[cell]),
				       getVoltage(
					       bmsdata->chips[chip]
						       .scell.sc_codes[cell]));
			}
		}
	}
}

/**
 * @brief Get the avgeraged cell voltages.
 * 
 * @param chip Array of chips to get voltage readings of.
 */
void get_avgd_cell_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);

	adbms_wake();
	read_adbms_data(chips, RDACALL, Rdacall, ALL_GRP);
}

/**
 * @brief Get the filtered cell volrages.
 * 
 * @param chip Array of chips to get voltage readings of.
 */
void get_filtered_cell_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);

	adbms_wake();
	read_adbms_data(chips, RDFCALL, Rdfcall, ALL_GRP);
}

/**
 * @brief Get the c and s adc voltages. Does this with RDCSALL command.
 * 
 * @param chips Array of chips to get voltage readings of.
 */
void get_c_and_s_adc_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);

	adbms_wake();
	read_adbms_data(chips, RDCSALL, Rdcsall, ALL_GRP);
}

/**
 * @brief Read every register connected to the AUX ADC.
 * 
 * @param chips Array of chips to get voltage readings of.
 */
void read_aux_registers(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
	adBmsPollAdc(PLAUX1);

	adbms_wake();
	read_adbms_data(chips, RDAUXA, Aux, A);
	read_adbms_data(chips, RDAUXB, Aux, B);
	read_adbms_data(chips, RDAUXC, Aux, C);
	read_adbms_data(chips, RDAUXD, Aux, D);
}

/**
 * @brief Read voltages in every register connected to AUX2 ADC.
 * 
 * @param chips Array of chips to get voltages of.
 */
void read_aux2_registers(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adBms6830_Adax2(AUX_ALL);
	adBmsPollAdc(PLAUX2);

	adbms_wake();
	read_adbms_data(chips, RDRAXA, RAux, A);
	read_adbms_data(chips, RDRAXB, RAux, B);
	read_adbms_data(chips, RDRAXC, RAux, C);
	read_adbms_data(chips, RDRAXD, RAux, D);
}

/**
 * @brief Read status registers.
 * 
 * @param chips Array of chips to read voltages of.
 */
void adBms6830_read_status_registers(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
	adBmsPollAdc(PLAUX1);

	read_adbms_data(chips, RDSTATA, Status, A);
	read_adbms_data(chips, RDSTATB, Status, B);
	read_adbms_data(chips, RDSTATC, Status, C);
	read_adbms_data(chips, RDSTATD, Status, D);
	read_adbms_data(chips, RDSTATE, Status, E);
}

void segment_retrieve_data(acc_data_t *bmsdata)
{
	// printf("Get C adc voltages\n");
	get_c_adc_voltages(bmsdata->chips);

	// get_s_adc_voltages(bmsdata->chips);

	// The GPIOs in the AUX registers contain voltage readings from the therms.
	// printf("Get therms\n");
	read_aux_registers(bmsdata->chips);
	// If you want redundant Thermistor readings, uncomment the following.
	read_aux2_registers(bmsdata->chips);

	/* Save the contents of the reading so that we can use it to fill in missing
   * data */
	memcpy(previous_data, bmsdata->chip_data,
	       sizeof(chipdata_t) * NUM_CHIPS);
}

bool segment_is_balancing(cell_asic chips[NUM_CHIPS])
{
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		if (chips[chip].tx_cfgb.dcc > 0) {
			return true;
		}
	}
	return false;
}

void segment_disable_balancing(acc_data_t *bmsdata)
{
	// Initializes all array elements to zero
	bool discharge_config[NUM_CHIPS][NUM_CELLS_ALPHA] = { 0 };
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		set_mute_state(&bmsdata->chips[chip], true);
	}
	segment_configure_balancing(bmsdata, discharge_config);
}

void segment_configure_balancing(
	acc_data_t *bmsdata, bool discharge_config[NUM_CHIPS][NUM_CELLS_ALPHA])
{
	// TODO: Test
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		uint8_t num_cells = get_num_cells(bmsdata->chip_data);
		for (int cell = 0; cell < num_cells; cell++) {
			set_cell_discharge(&bmsdata->chips[chip], cell + 1,
					   discharge_config[chip][cell]);
			set_mute_state(&bmsdata->chips[chip], false);
		}
	}
	write_config_regs(bmsdata->chips);
}

// void averaging_therm_check(chipdata_t segment_data[NUM_CHIPS])
// {
// 	for (int therm = 1; therm <= 16; therm++) {
// 		for (int c = 0; c < NUM_CHIPS; c++) {
// 			/* Directly update for a set time from start up due to therm voltages
//        * needing to settle */
// 			if (therm_avg_counter < THERM_AVG * 10) {
// 				segment_data[c].thermistor_value[therm - 1] =
// 					segment_data[c]
// 						.thermistor_reading[therm - 1];
// 				segment_data[c].thermistor_value[therm + 15] =
// 					segment_data[c]
// 						.thermistor_reading[therm + 15];
// 				therm_avg_counter++;
// 			} else {
// 				/* We need to investigate this. Very sloppy */
// 				/* Discard if reading is 33C */
// 				if (segment_data[c]
// 					    .thermistor_reading[therm - 1] !=
// 				    33) {
// 					/* If measured value is larger than current "averaged" value,
//            * increment value */
// 					if (segment_data[c]
// 						    .thermistor_reading[therm -
// 									1] >
// 					    segment_data[c]
// 						    .thermistor_value[therm -
// 								      1]) {
// 						segment_data[c]
// 							.thermistor_value[therm -
// 									  1]++;
// 						/* If measured value is smaller than current "averaged" value,
//              * decrement value */
// 					} else if (segment_data[c]
// 							   .thermistor_reading
// 								   [therm - 1] <
// 						   segment_data[c]
// 							   .thermistor_value
// 								   [therm - 1]) {
// 						segment_data[c]
// 							.thermistor_value[therm -
// 									  1]--;
// 					}
// 				}

// 				/* See comments above. Identical but for the upper 16 therms */
// 				if (segment_data[c]
// 					    .thermistor_reading[therm + 15] !=
// 				    33) {
// 					if (segment_data[c]
// 						    .thermistor_reading[therm +
// 									15] >
// 					    segment_data[c]
// 						    .thermistor_value[therm +
// 								      15]) {
// 						segment_data[c]
// 							.thermistor_value[therm +
// 									  15]++;
// 					} else if (segment_data[c]
// 							   .thermistor_reading
// 								   [therm + 15] <
// 						   segment_data[c].thermistor_value
// 							   [therm + 15]) {
// 						segment_data[c]
// 							.thermistor_value[therm +
// 									  15]--;
// 					}
// 				}
// 			}
// 		}
// 	}
// }

// void standard_dev_therm_check(chipdata_t segment_data[NUM_CHIPS])
// {
// 	if (previous_data == NULL)
// 		return;
// 	int16_t avg_temp = calc_average(segment_data);
// 	uint8_t standard_dev = calc_therm_standard_dev(avg_temp);
// 	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
// 		for (uint8_t therm = 17; therm < 28; therm++) {
// 			/*
//        * If difference between thermistor and average is more than
//        * MAX_STANDARD_DEV set the therm to pack average
//        */
// 			if (abs(segment_data[c].thermistor_value[therm] -
// 				avg_temp) > (MAX_STANDARD_DEV * standard_dev)) {
// 				/* Nullify thermistor by setting to pack average */
// 				segment_data[c].thermistor_value[therm] =
// 					previous_data[c].thermistor_value[therm];
// 			}
// 		}
// 	}
// }

// int8_t calc_therm_standard_dev(chipdata_t segment_data[NUM_CHIPS], int16_t avg_temp)
// {
// 	uint16_t sum_diff_sqrd = 0;
// 	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
// 		for (uint8_t therm = 17; therm < 28; therm++) {
// 			uint16_t sum_diff =
// 				abs(segment_data[chip].thermistor_value[therm] -
// 				    avg_temp);
// 			sum_diff_sqrd += sum_diff * sum_diff;
// 		}
// 	}

// 	uint8_t standard_dev = sqrt(sum_diff_sqrd / 88);
// 	if (standard_dev < 8) {
// 		standard_dev = 8;
// 	}
// 	return standard_dev;
// }

// int16_t calc_average(chipdata_t segment_data[NUM_CHIPS])
// {
// 	int16_t avg = 0;
// 	for (int chip = 0; chip < NUM_CHIPS; chip++) {
// 		for (int therm = 17; therm < 28; therm++) {
// 			avg += segment_data[chip].thermistor_value[therm];
// 		}
// 	}

// 	avg = avg / (NUM_CHIPS * 11);
// 	return avg;
// }

// void variance_therm_check()
// {
// 	if (previous_data == NULL) {
// 		start_timer(&variance_timer, 1000);
// 		return;
// 	}

// 	if (is_timer_expired(&variance_timer)) {
// 		for (uint8_t c = 0; c < NUM_CHIPS; c++) {
// 			for (uint8_t therm = 17; therm < 28; therm++) {
// 				if (abs(segment_data[c]
// 						.thermistor_reading[therm] -
// 					previous_data[c]
// 						.thermistor_reading[therm]) >
// 					    5 &&
// 				    (segment_data[c].thermistor_reading[therm] <
// 					     10 ||
// 				     segment_data[c].thermistor_reading[therm] >
// 					     30)) {
// 					segment_data[c]
// 						.thermistor_reading[therm] =
// 						previous_data[c]
// 							.thermistor_reading
// 								[therm];
// 					segment_data[c].thermistor_value[therm] =
// 						previous_data[c]
// 							.thermistor_value[therm];
// 				}
// 			}
// 		}
// 	}
// }

// void discard_neutrals(chipdata_t segment_data[NUM_CHIPS])
// {
// 	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
// 		for (uint8_t therm = 17; therm < 28; therm++) {
// 			if (segment_data[c].thermistor_reading[therm] == 33) {
// 				segment_data[c].thermistor_reading[therm] = 25;
// 				segment_data[c].thermistor_value[therm] = 25;
// 			}
// 		}
// 	}
// }
