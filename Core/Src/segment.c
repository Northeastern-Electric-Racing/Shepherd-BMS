#include "segment.h"
#include <math.h>
#include "analyzer.h"
#include "c_utils.h"

#include "serialPrintResult.h"
#include "adBms6830ParseCreate.h"
#include "adi_interaction.h"

#define T_READY 10 /* microseconds*/
#define T_IDLE	4.3 /* milliseconds, minimum. typ is 5.5, max is 6.7 */
#define T_WAKE	200 /* microseconds */
#define T_SLEEP 1.8 /* seconds minimum, typ is 2, max is 2.2 */
#define T_REFUP 2.7 /* milliseconds minimum, typ is 3.5, max is 4.4 */

#define THERM_WAIT_TIME	  500 /* ms */
#define VOLTAGE_WAIT_TIME 500 /* ms */
#define THERM_AVG	  15 /* Number of values to average */
#define MAX_VOLT_DELTA	  2500
#define MAX_CONSEC_NOISE  10

extern TIM_HandleTypeDef htim2;

uint8_t therm_avg_counter = 0;

nertimer_t variance_timer;

/* private function prototypes */
// void variance_therm_check(void);
// void discard_neutrals(chipdata_t segment_data[NUM_CHIPS]);
// void pull_chip_configuration(void);
// int16_t calc_average(chipdata_t segment_data[NUM_CHIPS]);
// int8_t calc_therm_standard_dev(int16_t avg_temp);
void init_chip(cell_asic *chip);
void write_config_regs(cell_asic chip[NUM_CHIPS]);
void set_cell_discharge(cell_asic *chip, uint8_t cell, bool discharge);

/**
 * @brief Initialize a chip with our default values.
 * 
 * @param chip Pointer to chip to initialize.
 */
void init_chip(cell_asic *chip)
{
	set_REFON(chip, PWR_UP);
	// WARNING, THE ENUM IS WRONG, CHECK TABLE 102
	set_volt_adc_comp_thresh(chip, CVT_45mV);
	chip->tx_cfga.flag_d = 0;

	// Short soak on ADAX
	set_soak_on(chip, SOAKON_SET);
	set_aux_soak_range(chip, SHORT);

	// No open wire detect soak
	set_open_wire_soak_time(chip, OWA0);

	// Set therm GPIOs
	set_gpio_pull(chip, 1, true);
	set_gpio_pull(chip, 2, true);
	set_gpio_pull(chip, 3, true);
	set_gpio_pull(chip, 4, true);
	set_gpio_pull(chip, 5, true);
	set_gpio_pull(chip, 6, true);
	set_gpio_pull(chip, 7, true); // this is a on board therm for beta only
	set_gpio_pull(chip, 8, true); // this is a on board therm

	// set outputs, 9=iso led 10=bal LED
	set_gpio_pull(chip, 9, false);
	set_gpio_pull(chip, 10, false);

	// Registers are unfrozen
	set_snapshot(chip, SNAP_OFF);

	// Charging is deactivated
	set_mute_state(chip, true);

	// Not an endpoint in the daisy chain
	set_comm_break(chip, false);

	// IIR filter disabled
	set_iir_corner_freq(chip, IIR_FPA16);

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
 * @brief Initialize chips with default values.
 * 
 */
void segment_init(acc_data_t *bmsdata)
{
	printf("Initializing Segments...");
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		init_chip(&bmsdata->chips[chip]);
		// TODO: Make sure this is accurate
		bmsdata->chip_data[chip].alpha = chip % 2 == 0;
	}
	write_config_regs(bmsdata->chips);

	start_c_adc_conv();
}

void segment_adc_comparison(acc_data_t *bmsdata)
{
	// TODO: S-ADC measurements are all over the place.

	// Take single shot measurement
	// adBms6830_Adcv(RD_ON, SINGLE, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	// adBmsPollAdc(PLCADC);
	// read_adbms_data(bmsdata->chips, RDCVALL, Rdcvall, ALL_GRP);

	// Result of C-ADC and S-ADC comparison is stored in status register group C
	read_status_registers(bmsdata->chips);

	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
		uint8_t cells = get_num_cells(&bmsdata->chip_data[chip]);
		for (uint8_t cell = 0; cell < cells; cell++) {
			if (NER_GET_BIT(bmsdata->chips[chip].statc.cs_flt,
					cell)) {
				printf("ADC VOLTAGE DISCREPANCY ERROR\nChip %d, Cell %d\nC-ADC: %f, S-ADC: %f\n",
				       chip + 1, cell + 1,
				       getVoltage(
					       bmsdata->chips[chip]
						       .fcell.fc_codes[cell]),
				       getVoltage(
					       bmsdata->chips[chip]
						       .scell.sc_codes[cell]));
			}
		}
	}
}

void segment_monitor_flts(cell_asic chips[NUM_CHIPS])
{
	read_status_registers(chips);
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		if (chips[chip].statc.va_ov) {
			printf("A OV FLT\n");
		}
		if (chips[chip].statc.va_uv) {
			printf("A UV FLT\n");
		}
		if (chips[chip].statc.vd_ov) {
			printf("D OV FLT\n");
		}
		if (chips[chip].statc.vd_uv) {
			printf("D OV FLT\n");
		}
		if (chips[chip].statc.vde) {
			printf("VDE FLT\n");
		}
		if (chips[chip].statc.vdel) {
			printf("VDEL FLT\n");
		}
		if (chips[chip].statc.spiflt) {
			printf("SPI SLV FLT\n");
		}
		if (chips[chip].statc.sleep) {
			printf("SLEEP OCCURED\n");
		}
		if (chips[chip].statc.thsd) {
			printf("THERMAL FLT\n");
		}
		if (chips[chip].statc.oscchk) {
			printf("OSC FLT\n");
		}
		if (chips[chip].statc.otp1_med) {
			printf("CMED? FLT\n");
		}
		if (chips[chip].statc.otp2_med) {
			printf("SMED? FLT\n");
		}
	}
	// clear them
	write_clear_flags(chips);
}

// ensure stuff used is in the correctfunction
void segment_retrieve_data(acc_data_t *bmsdata)
{
	// read from ADC convs
	read_filtered_voltage_registers(bmsdata->chips);

	// check our fault flags
	segment_monitor_flts(bmsdata->chips);

	// read all therms using AUX 2
	adc_and_read_aux2_registers(bmsdata->chips);
}
void segment_retrieve_debug_data(acc_data_t *bmsdata)
{
	// poll stuff like vref, etc.
	adc_and_read_aux_registers(bmsdata->chips);

	// read the above into status registers
	read_status_registers(bmsdata->chips);
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
	// write_config_regs(bmsdata->chips);
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
