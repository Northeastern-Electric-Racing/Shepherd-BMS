#include "analyzer.h"

#include <math.h>
#include <float.h>

#include "can_messages.h"
#include "serialPrintResult.h"

#define GPIO1  0
#define GPIO2  1
#define GPIO3  2
#define GPIO4  3
#define GPIO5  4
#define GPIO6  5
#define GPIO7  6
#define GPIO8  7
#define GPIO9  8
#define GPIO10 9

typedef enum {
	THERM1 = 0,
	THERM2,
	THERM3,
	THERM4,
	THERM5,
	THERM6,
	THERM7
} THERM;

// clang-format off

//TODO: CHANGE ALL FOR NEW CELLS

/**
 * @brief Mapping Cell temperatue to the discharge current limit based on the
 *      temperature discharge limit curve profile of the Samsung 186500 INR
 *      in the Orion BMS software utility app
 *
 * @note Units are in Amps and indicies are in (degrees C)/5, stops at 65C
 * @note Limit should be *interpolated* from these values (i.e. if we are
 *      at 27C, we should take the limit that is halfway between 25C and 30C)
 *
 */
const uint8_t TEMP_TO_DCL[14] =
{
	110, 125, 140, 140, 140, 140,
	140, 140, 140, 100, 60, 20, 0, 0
};

/**
 * @brief Mapping Cell temperatue to the charge current limit based on the
 *      temperature charge limit curve profile of the Samsung 186500 INR
 *      in the Orion BMS software utility app
 *
 * @note Units are in Amps and indicies are in (degrees C)/5, stops at 65C
 * @note Limit should be *interpolated* from these values (i.e. if we are
 *      at 27C, we should take the limit that is halfway between 25C and 30C)
 *
 */
const uint8_t TEMP_TO_CCL[14] =
{
	0, 25, 25, 25, 25, 25, 25, 25,
	20, 15, 10, 5, 1, 1
};

// END TODO

/**
 * @brief Mapping desired fan speed PWM to the cell temperature
 *
 * @note Units are in PWM out of 255 and indicies are in (degrees C)/5, stops at 65C
 * @note Limit should be *interpolated* from these values (i.e. if we are
 *      at 27C, we should take the limit that is halfway between 25C and 30C)
 */
const uint8_t FAN_CURVE[16] =
{
	0, 0, 0, 0, 0, 0, 0, 0, 32, 64,
	128, 255, 255, 255, 255, 255
};

const uint8_t NO_THERM = 0xFF;
const uint8_t MUX_OFFSET = 16;

/**
 * @brief Map cells to therms.
 * 
 */
const THERM THERM_MAP[NUM_CELLS_ALPHA] =
{
	THERM1,
	THERM1,
	THERM2,
	THERM2,
	THERM3,
	THERM3,
	THERM4,
	THERM4,
	THERM5,
	THERM5,
	THERM6,
	THERM6,
	THERM7,
	THERM7
};

// clang-format on

nertimer_t analysisTimer;
nertimer_t ocvTimer;

bool is_first_reading_ = true;

uint8_t get_num_cells(chipdata_t *chip_data)
{
	if (chip_data->alpha) {
		return NUM_CELLS_ALPHA;
	} else {
		return NUM_CELLS_BETA;
	}
}

/**
 * @brief Calculate a cell temperature based on the thermistor reading.
 * 
 * @param x The thremistor reading.
 * @return float The temperature in degrees Celsius.
 */
float calc_cell_temp(uint16_t x)
{
	/* Polynomial fit of temperatures -7 -> 65 celsius vs. thermistor voltage. */
	return 0.6984 * pow(x, 4) + 4.4933 * pow(x, 3) - 10.278 * pow(x, 2) +
	       34.184 * x + 2.7608;
}

void calc_cell_temps(acc_data_t *bmsdata)
{
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		uint8_t num_cells = get_num_cells(&bmsdata->chip_data[chip]);

		for (int cell = 0; cell < num_cells; cell++) {
			int16_t x = bmsdata->chips[chip]
					    .raux.ra_codes[THERM_MAP[cell]];

			bmsdata->chip_data[chip].cell_temp[cell] =
				calc_cell_temp(getVoltage(x));
		}

		// Calculate onboard therm temps and chip temp

		if (!bmsdata->chip_data[chip].alpha) {
			// Take average of both onboard therms
			bmsdata->chip_data[chip].on_board_temp =
				(calc_cell_temp(getVoltage(
					 bmsdata->chips[chip].raux.ra_codes[6])) +
				 calc_cell_temp(getVoltage(
					 bmsdata->chips[chip]
						 .raux.ra_codes[7]))) /
				2;
		} else {
			bmsdata->chip_data[chip].on_board_temp =
				calc_cell_temp(getVoltage(
					bmsdata->chips[chip].raux.ra_codes[7]));
		}

		/* set the die temp */
		// conversion rate from datasheet, Table 105.  also in driver src
		bmsdata->chip_data[chip].die_temp =
			(getVoltage(bmsdata->chips[chip].stata.itmp) / 0.0075) -
			273;
	}

	/*
	

	22A CODE FOR REFERENCE

	//we are not corrctly mapping each therm reading to the correct cell. So, we are taking the average of all good readings (not disabled) for a given chip, 
    //and assigning that to be the cell val for every cell in the chip

	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
		for (uint8_t cell = 0; cell < NUM_CELLS_PER_CHIP; cell++) {
			const uint8_t(*therm_map)[NUM_RELEVANT_THERMS] =
				(c % 2 == 0) ? RELEVANT_THERM_MAP_L :
					       RELEVANT_THERM_MAP_H;
			uint8_t therm_count = 0;
			int temp_sum = 0;
			for (uint8_t therm = 0; therm < NUM_RELEVANT_THERMS;
			     therm++) {
				uint8_t thermNum = therm_map[cell][therm];

				if (thermNum != NO_THERM) {
					//printf("%d\t", bmsdata->chip_data[c].thermistor_value[therm]);
					temp_sum +=
						bmsdata->chip_data[c]
							.thermistor_value[therm];
					therm_count++;
				}
			}
			//printf("\r\n");

			// Takes the average temperature of all the relevant thermistors
			bmsdata->chip_data[c].cell_temp[cell] = temp_sum / therm_count;
			therm_count = 0;
	}
	*/
}

void calc_pack_temps(acc_data_t *bmsdata)
{
	bmsdata->max_temp.val = FLT_MIN;
	bmsdata->max_temp.cellNum = 0;
	bmsdata->max_temp.chipIndex = 0;

	bmsdata->min_temp.val = FLT_MAX;
	bmsdata->min_temp.cellNum = 0;
	bmsdata->min_temp.chipIndex = 0;

	bmsdata->max_chiptemp.val = 0;

	float total_temp = 0;
	float total_seg_temp = 0;

	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
		uint8_t num_cells = get_num_cells(&bmsdata->chip_data[c]);

		for (uint8_t cell = 0; cell < num_cells; cell++) {
			if (bmsdata->chip_data[c].cell_temp[cell] >
			    bmsdata->max_temp.val) {
				bmsdata->max_temp.val =
					bmsdata->chip_data[c].cell_temp[cell];
				bmsdata->max_temp.cellNum = cell;
				bmsdata->max_temp.chipIndex = c;
			}

			/* finds out the minimum cell temp and location */
			if (bmsdata->chip_data[c].cell_temp[cell] <
			    bmsdata->min_temp.val) {
				bmsdata->min_temp.val =
					bmsdata->chip_data[c].cell_temp[cell];
				bmsdata->min_temp.cellNum = cell;
				bmsdata->min_temp.chipIndex = c;
			}

			total_temp += bmsdata->chip_data[c].cell_temp[cell];
			total_seg_temp += bmsdata->chip_data[c].cell_temp[cell];
		}
		/* only for NERO */
		if (c % 2 == 0) {
			bmsdata->segment_average_temps[c / 2] =
				total_seg_temp /
				((float)(NUM_CELLS_ALPHA + NUM_CELLS_BETA));
			total_seg_temp = 0;
		}

		if (bmsdata->max_chiptemp.val <
		    bmsdata->chip_data[c].die_temp) {
			bmsdata->max_chiptemp = (crit_chipval_t){
				.chipNum = c,
				.val = bmsdata->chip_data[c].die_temp
			};
		}
	}

	/* Takes the average of all the cell temperatures. */
	bmsdata->avg_temp = total_temp / NUM_CELLS;

	send_cell_temp_message(bmsdata->max_temp, bmsdata->min_temp,
			       bmsdata->avg_temp);
}

void calc_cell_voltages(acc_data_t *bmsdata)
{
	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
		uint8_t num_cells = get_num_cells(&bmsdata->chip_data[chip]);

		for (uint8_t cell = 0; cell < num_cells; cell++) {
			bmsdata->chip_data[chip].cell_voltages[cell] =
				getVoltage(bmsdata->chips[chip]
						   .fcell.fc_codes[cell]);
		}
	}
}

void calc_pack_voltage_stats(acc_data_t *bmsdata)
{
	bmsdata->max_voltage.val = FLT_MIN;
	bmsdata->max_voltage.cellNum = 0;
	bmsdata->max_voltage.chipIndex = 0;

	bmsdata->max_ocv.val = FLT_MIN;
	bmsdata->max_ocv.cellNum = 0;
	bmsdata->max_ocv.chipIndex = 0;

	bmsdata->min_voltage.val = FLT_MAX;
	bmsdata->min_voltage.cellNum = 0;
	bmsdata->min_voltage.chipIndex = 0;

	bmsdata->min_ocv.val = FLT_MAX;
	bmsdata->min_ocv.cellNum = 0;
	bmsdata->min_ocv.chipIndex = 0;

	float total_volt = 0;
	float total_ocv = 0;

	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
		uint8_t num_cells = get_num_cells(&bmsdata->chip_data[c]);
		for (uint8_t cell = 0; cell < num_cells; cell++) {
			/* fings out the maximum cell voltage and location */
			if (bmsdata->chip_data[c].cell_voltages[cell] >
			    bmsdata->max_voltage.val) {
				bmsdata->max_voltage.val =
					bmsdata->chip_data[c]
						.cell_voltages[cell];
				bmsdata->max_voltage.chipIndex = c;
				bmsdata->max_voltage.cellNum = cell;
			}

			if (bmsdata->chip_data[c].open_cell_voltage[cell] >
			    bmsdata->max_ocv.val) {
				bmsdata->max_ocv.val =
					bmsdata->chip_data[c]
						.open_cell_voltage[cell];
				bmsdata->max_ocv.chipIndex = c;
				bmsdata->max_ocv.cellNum = cell;
			}

			/* finds out the minimum cell voltage and location */
			if (bmsdata->chip_data[c].cell_voltages[cell] <
			    bmsdata->min_voltage.val) {
				bmsdata->min_voltage.val =
					bmsdata->chip_data[c]
						.cell_voltages[cell];
				bmsdata->min_voltage.chipIndex = c;
				bmsdata->min_voltage.cellNum = cell;
			}

			if (bmsdata->chip_data[c].open_cell_voltage[cell] <
			    bmsdata->min_ocv.val) {
				bmsdata->min_ocv.val =
					bmsdata->chip_data[c]
						.open_cell_voltage[cell];
				bmsdata->min_ocv.chipIndex = c;
				bmsdata->min_ocv.cellNum = cell;
			}

			total_volt += bmsdata->chip_data[c].cell_voltages[cell];
			total_ocv +=
				bmsdata->chip_data[c].open_cell_voltage[cell];
		}
	}

	/* calculate some voltage stats */
	// TODO: Make this based on total cells when actual segment is here
	bmsdata->avg_voltage = total_volt / NUM_CELLS;

	bmsdata->pack_voltage = total_volt;

	bmsdata->delt_voltage =
		bmsdata->max_voltage.val - bmsdata->min_voltage.val;

	bmsdata->avg_ocv = total_ocv / NUM_CELLS;
	bmsdata->pack_ocv = total_ocv;
	bmsdata->delt_ocv = bmsdata->max_ocv.val - bmsdata->min_ocv.val;

	send_acc_status_message(bmsdata->pack_voltage, bmsdata->pack_current,
				bmsdata->soc);
	send_cell_voltage_message(bmsdata->max_voltage, bmsdata->min_voltage,
				  bmsdata->avg_voltage);
}

void calc_cell_resistances(acc_data_t *bmsdata)
{
	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
		uint8_t num_cells = get_num_cells(&bmsdata->chip_data[c]);

		for (uint8_t cell = 0; cell < num_cells; cell++) {
			if (fabs(bmsdata->pack_current) >= 0.001) {
				bmsdata->chip_data[c].cell_resistance[cell] =
					(bmsdata->chip_data[c]
						 .open_cell_voltage[cell] -
					 bmsdata->chip_data[c]
						 .cell_voltages[cell]) /
					fabs(bmsdata->pack_current);
			} else {
				bmsdata->chip_data[c].cell_resistance[cell] =
					0.015; // default resistance from data sheet
			}
		}
	}
}

// TODO: Change to match P45Bs.
void calc_dcl(acc_data_t *bmsdata)
{
	static nertimer_t dcl_timer;

	static uint16_t prev_dcl;

	int16_t current_limit = 0x7FFF;

	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
		uint8_t num_cells = get_num_cells(&bmsdata->chip_data[c]);
		for (uint8_t cell = 0; cell < num_cells; cell++) {
			/* Apply equation */
			/* Multiplying resistance by 10 to convert from mOhm to Ohm and then to Ohm * 10000 to
			 * account for the voltage units */
			uint16_t tmpDCL =
				(bmsdata->chip_data[c].open_cell_voltage[cell] -
				 (MIN_VOLT + VOLT_SAG_MARGIN)) /
				(bmsdata->chip_data[c].cell_resistance[cell] *
				 10);

			/* Taking the minimum DCL of all the cells */
			if (tmpDCL < current_limit)
				current_limit = tmpDCL;
		}
	}

	/* ceiling for current limit */
	if (current_limit > MAX_DISCHG_CURR) {
		bmsdata->discharge_limit = MAX_DISCHG_CURR;
		return;
	}

	/* protection against being init to a high value */
	if (bmsdata->discharge_limit > MAX_DISCHG_CURR) {
		bmsdata->discharge_limit = 0;
		prev_dcl = 0;
	}

	/* State machine to prevent DCL from plummeting, copy over last DCL for 500ms */
	else if (!is_timer_active(&dcl_timer) && current_limit < 5) {
		bmsdata->discharge_limit = prev_dcl;
		start_timer(&dcl_timer, 500);
	}

	else if (is_timer_active(&dcl_timer)) {
		if (is_timer_expired(&dcl_timer)) {
			bmsdata->discharge_limit = current_limit;
			prev_dcl = current_limit;
		}
		if (current_limit > 5) {
			bmsdata->discharge_limit = current_limit;
			prev_dcl = current_limit;
			cancel_timer(&dcl_timer);
		}

		else {
			bmsdata->discharge_limit = prev_dcl;
		}
	} else {
		bmsdata->discharge_limit = current_limit;
		prev_dcl = current_limit;
	}

	if (bmsdata->discharge_limit > DCDC_CURRENT_DRAW) {
		bmsdata->discharge_limit -= DCDC_CURRENT_DRAW;
		prev_dcl -= DCDC_CURRENT_DRAW;
	}

	send_mc_discharge_message(bmsdata->discharge_limit);
}

//TODO: Fix for new cells and BMS
void calc_cont_dcl(acc_data_t *bmsdata)
{
	uint8_t min_res_index =
		(bmsdata->min_temp.val - MIN_TEMP) /
		5; /* resistance LUT increments by 5C for each index */
	uint8_t max_res_index = (bmsdata->max_temp.val - MIN_TEMP) / 5;

	if (TEMP_TO_DCL[min_res_index] < TEMP_TO_DCL[max_res_index]) {
		bmsdata->cont_DCL = TEMP_TO_DCL[min_res_index];
	} else {
		bmsdata->cont_DCL = TEMP_TO_DCL[max_res_index];
	}
}

//TODO: Change for P45B electrical characteristics.
void calcCCL(acc_data_t *bmsdata)
{
	int16_t currentLimit = 0x7FFF;

	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
		uint8_t num_cells = get_num_cells(&bmsdata->chip_data[c]);
		for (uint8_t cell = 0; cell < num_cells; cell++) {
			/* Apply equation */
			uint16_t tmpCCL =
				(MAX_VOLT -
				 (bmsdata->chip_data[c].open_cell_voltage[cell] +
				  VOLT_SAG_MARGIN)) /
				(bmsdata->chip_data[c].cell_resistance[cell] *
				 10);
			/* Multiplying resistance by 10 to convert from mOhm to Ohm and then to Ohm * 10000 to
			 * account for the voltage units */

			/* Taking the minimum CCL of all the cells */
			if (tmpCCL < currentLimit)
				currentLimit = tmpCCL;
		}
	}

	/* ceiling for current limit */
	if (currentLimit > MAX_CHG_CURR) {
		bmsdata->charge_limit = MAX_CHG_CURR;
	} else {
		bmsdata->charge_limit = currentLimit;
	}

	send_mc_charge_message(bmsdata->charge_limit);
}

//TODO: Change for P45B electrical characteristics.
void calc_cont_ccl(acc_data_t *bmsdata)
{
	uint8_t min_res_index =
		(bmsdata->min_temp.val - MIN_TEMP) /
		5; /* resistance LUT increments by 5C for each index */
	uint8_t max_res_index = (bmsdata->max_temp.val - MIN_TEMP) / 5;

	if (TEMP_TO_CCL[min_res_index] < TEMP_TO_CCL[max_res_index]) {
		bmsdata->cont_CCL = TEMP_TO_CCL[min_res_index];
	} else {
		bmsdata->cont_CCL = TEMP_TO_CCL[max_res_index];
	}

	if (bmsdata->cont_CCL > MAX_CHG_CURR) {
		bmsdata->cont_CCL = MAX_CHG_CURR;
	}
}

void calc_open_cell_voltage(acc_data_t *bmsdata)
{
	// If we are within the current threshold for open voltage measurments (1.5 mA)
	if (bmsdata->pack_current < OCV_CURR_THRESH &&
	    bmsdata->pack_current > -1 * OCV_CURR_THRESH) {
		// Timer expired or not active
		if (is_timer_expired(&ocvTimer) ||
		    !is_timer_active(&ocvTimer)) {
			for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
				// Number of cells in the chip
				uint8_t num_cells = get_num_cells(
					&bmsdata->chip_data[chip]);
				for (uint8_t cell = 0; cell < num_cells;
				     cell++) {
					// This is the actual OCV value in the cell
					uint16_t ocv_value =
						bmsdata->chip_data[chip]
							.cell_voltages[cell];

					// Set current OCV value
					bmsdata->chip_data[chip]
						.open_cell_voltage[cell] =
						ocv_value;
				}
			}
		} else {
			start_timer(&ocvTimer, 1000);
		}
	}
}

void calc_state_of_charge(acc_data_t *bmsdata)
{
	float volts = bmsdata->min_ocv.val;

	double soc = (-55.919476 * pow(16.1336555, volts)) +
		     (55.9296372 * pow(16.1330198, volts)) - 6.3330011;

	if (soc > 100) {
		soc = 100;
	}

	else if (soc < 0) {
		soc = 0;
	}

	bmsdata->soc = (float)soc;

	send_acc_status_message(bmsdata->pack_voltage, bmsdata->pack_current,
				bmsdata->soc);
}

//TODO: Make it actually calc OCVs. Revise algorithm and stuff.
//TODO: Change for new cells (probs not needed).
//void calc_open_cell_voltage(acc_data_t *bmsdata)
//{
//TODO: MAKE NOT SHIT :)

// static chipdata_t prev_chipdata[12];

// /* if there is no previous data point, set inital open cell voltage to current reading */
// if (is_first_reading_) {
// 	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
// 		uint8_t num_cells =
// 			get_num_cells(&bmsdata->chip_data[chip]);
// 		for (uint8_t cell = 0; cell < num_cells; cell++) {
// 			bmsdata->chip_data[chip]
// 				.open_cell_voltage[cell] =
// 				bmsdata->chip_data[chip]
// 					.cell_voltages[cell];
// 			prev_chipdata[chip].open_cell_voltage[cell] =
// 				bmsdata->chip_data[chip]
// 					.cell_voltages[cell];
// 		}
// 	}
// 	return;
// }
// /* If we are within the current threshold for open voltage measurments */
// else if (bmsdata->pack_current < (OCV_CURR_THRESH * 10) &&
// 	 bmsdata->pack_current > (-OCV_CURR_THRESH * 10)) {
// 	if (is_timer_expired(&ocvTimer) ||
// 	    !is_timer_active(&ocvTimer)) {
// 		for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
// 			uint8_t num_cells = get_num_cells(
// 				&bmsdata->chip_data[chip]);
// 			for (uint8_t cell = 0; cell < num_cells;
// 			     cell++) {
// 				/* Sets open cell voltage to a moving average of OCV_AVG values */
// 				bmsdata->chip_data[chip]
// 					.open_cell_voltage[cell] =
// 					((bmsdata->chip_data[chip].cell_voltages[cell];
// 					 ((prev_chipdata[chip].open_cell_voltage
// 							     [cell]) *
// 					  (OCV_AVG - 1))) /
// 					OCV_AVG;
// 				bmsdata->chip_data[chip]
// 					.open_cell_voltage[cell] =
// 					bmsdata->chips[chip]
// 						.fcell.fc_codes[cell];

// 				if (bmsdata->chip_data[chip]
// 					    .open_cell_voltage[cell] >
// 				    MAX_VOLT * 10000) {
// 					bmsdata->chip_data[chip]
// 						.open_cell_voltage[cell] =
// 						prev_chipdata[chip]
// 							.open_cell_voltage
// 								[cell];
// 				} else if (bmsdata->chip_data[chip]
// 						   .open_cell_voltage
// 							   [cell] <
// 					   MIN_VOLT * 10000) {
// 					bmsdata->chip_data[chip]
// 						.open_cell_voltage[cell] =
// 						prev_chipdata[chip]
// 							.open_cell_voltage
// 								[cell];
// 				}
// 			}
// 		}
// 		return;
// 	}
// } else {
// 	start_timer(&ocvTimer, 1000);
// }
// for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
// 	uint8_t num_cells = get_num_cells(&bmsdata->chip_data[chip]);
// 	for (uint8_t cell = 0; cell < num_cells; cell++) {
// 		/* Set OCV to the previous/existing OCV */
// 		bmsdata->chip_data[chip].open_cell_voltage[cell] =
// 			prev_chipdata[chip].open_cell_voltage[cell];
// 	}
// }
//}

//TODO: Change for new fans and cell temps
// uint8_t analyzer_calc_fan_pwm(acc_data_t *bmsdata)
// {
// 	/* Resistance LUT increments by 5C for each index, plus we account for negative minimum */
// 	uint8_t min_res_index = (bmsdata->max_temp.val - MIN_TEMP) / 5;
// 	/* Ints are roounded down, so this would be the value if rounded up */
// 	uint8_t max_res_index = (bmsdata->max_temp.val - MIN_TEMP) / 5 + 1;
// 	/* Determine how far into the 5C interval the temp is */
// 	uint8_t part_of_index = (bmsdata->max_temp.val - MIN_TEMP) % 5;

// 	/* Uses fan LUT and finds low and upper end. Then takes average, weighted to how far into the
// 	 * interval the exact temp is */
// 	return ((FAN_CURVE[max_res_index] * part_of_index) +
// 		(FAN_CURVE[min_res_index] * (5 - part_of_index))) /
// 	       (2 * 5);
// }

// void disable_therms(acc_data_t *bmsdata)
// {
// 	int8_t tmp_temp =
// 		25; /* Iniitalize to room temp (necessary to stabilize when the BMS first boots up/has null values) */
// 	if (!is_first_reading_)
// 		tmp_temp =
// 			bmsdata->avg_temp; /* Set to actual average temp of the pack */

// 	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
// 		for (uint8_t therm = 0; therm < NUM_THERMS_PER_CHIP; therm++) {
// 			/* If 2D LUT shows therm should be disable */
// 			if (THERM_DISABLE[c][therm]) {
// 				/* Nullify thermistor by setting to pack average */
// 				bmsdata->chip_data[c].thermistor_value[therm] =
// 					tmp_temp;
// 			} else {
// 				bmsdata->chip_data[c].thermistor_value[therm] =
// 					bmsdata->chip_data[c]
// 						.thermistor_reading[therm];
// 			}
// 		}
// 	}
// }

// NOTE: This function is broken or something.
// void calc_noise_volt_percent(acc_data_t *bmsdata)
// {
// 	int i = 0;
// 	for (uint8_t seg = 0; seg < NUM_SEGMENTS; seg++) {
// 		uint8_t count = 0;

// 		/* merge results from each of the two chips on a given segment */
// 		for (uint8_t cell = 0; cell < NUM_CELLS_SEG; cell++) {
// 			count = bmsdata->chip_data[seg + i].noise_reading[cell];
// 			count += bmsdata->chip_data[seg + i + 1]
// 					 .noise_reading[cell];
// 		}
// 		i++;

// 		/* turn into percentage */
// 		//printf("count: %d\r\n", count);
// 		bmsdata->segment_noise_percentage[seg] =
// 			(uint8_t)(100 * (count) / (NUM_CELLS_SEG * 2.0f));
// 	}
// }

// void high_curr_therm_check()
// {
// 	if (is_first_reading_)
// 		return;

// 	if (bmsdata->pack_current > 500) {
// 		for (uint8_t c = 0; c < NUM_CHIPS; c++) {
// 			for (uint8_t cell = 0; cell < NUM_CELLS_PER_CHIP;
// 			     cell++) {
// 				bmsdata->chip_data[c].thermistor_reading[cell] =
// 					prevbmsdata->chip_data[c]
// 						.thermistor_reading[cell];
// 				bmsdata->chip_data[c].thermistor_value[cell] =
// 					prevbmsdata->chip_data[c]
// 						.thermistor_value[cell];
// 			}
// 		}
// 	}
// }

// void diff_curr_therm_check()
// {
// 	if (prevbmsdata == NULL)
// 		return;

// 	if (abs(bmsdata->pack_current - prevbmsdata->pack_current) > 100) {
// 		for (uint8_t c = 0; c < NUM_CHIPS; c++) {
// 			for (uint8_t cell = 0; cell < NUM_CELLS_PER_CHIP;
// 			     cell++) {
// 				bmsdata->chip_data[c].thermistor_reading[cell] =
// 					prevbmsdata->chip_data[c]
// 						.thermistor_reading[cell];
// 				bmsdata->chip_data[c].thermistor_value[cell] =
// 					prevbmsdata->chip_data[c]
// 						.thermistor_value[cell];
// 			}
// 		}
// 	}
// }
