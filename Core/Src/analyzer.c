#include "analyzer.h"

#include <math.h>
#include <float.h>

#include "can_messages.h"
#include "serialPrintResult.h"

// the OCV timer
nertimer_t ocvTimer;

/**
 * @brief Map cells to therms (ra codes).  Note beta has only 6 therms.
 * 
 */
const int THERM_MAP[NUM_CELLS_ALPHA] = { 0, 0, 1, 1, 2, 2, 3,
					 3, 4, 4, 5, 5, 6, 6 };

uint8_t get_num_cells(chipdata_t *chip_data)
{
	if (chip_data->alpha) {
		return NUM_CELLS_ALPHA;
	} else {
		return NUM_CELLS_BETA;
	}
}

/**
 * @brief Calculate the cell temperature of a 10,000 ohm NTP resistor (model 103)
 * 
 * @param res The resistance of the resistor
 * @return float The temperature
 */
float calc_temp(float res)
{
	float coef = res / 10000.0;
	// achieved via passing ThermCalcs.xlsx into https://www.standardsapplied.com/nonlinear-curve-fitting-calculator.html
	return -1149.531863 * (pow(coef, 1.0 / 8)) +
	       658.9396848 * (pow(coef, 1.0 / 4)) +
	       -87.8102815 * (pow(coef, 1.0 / 2)) + 2.034216235 * coef +
	       601.008351;
}

/**
 * @brief Calculate a cell temperature based on the thermistor reading.
 * 
 * @param voltage The thremistor reading.
 * @return float The temperature in degrees Celsius.
 */
float calc_cell_temp(float voltage)
{
	float res = (5600 * (3 - voltage)) / voltage;
	return calc_temp(res);
}

/**
 * @brief Calculate a cell temperature of onboard therm
 * 
 * @param voltage the voltage read by ADC
 * @return float The temperature in degrees C
 */
float calc_cell_temp_onboard(float voltage)
{
	float res = (5600 * (5 - voltage)) / voltage;
	return calc_temp(res);
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
				(calc_cell_temp_onboard(getVoltage(
					 bmsdata->chips[chip].raux.ra_codes[6])) +
				 calc_cell_temp_onboard(getVoltage(
					 bmsdata->chips[chip]
						 .raux.ra_codes[7]))) /
				2;
		} else {
			//printf("\nONBOARD alpha %f\n\n",
			//       getVoltage(
			//	       bmsdata->chips[chip].raux.ra_codes[7]));
			bmsdata->chip_data[chip].on_board_temp =
				calc_cell_temp_onboard(getVoltage(
					bmsdata->chips[chip].raux.ra_codes[7]));
		}

		/* set the die temp */
		// conversion rate from datasheet, Table 105.  also in driver src
		bmsdata->chip_data[chip].die_temp =
			(getVoltage(bmsdata->chips[chip].stata.itmp) / 0.0075) -
			273;
	}
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
	float total_seg_volt = 0;

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

			total_seg_volt +=
				bmsdata->chip_data[c].cell_voltages[cell];

			if (c % 2 == 0) {
				bmsdata->segment_average_volts[c / 2] =
					total_seg_volt /
					((float)(NUM_CELLS_ALPHA +
						 NUM_CELLS_BETA));
				total_seg_volt = 0;
			}
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

void calc_cont_dcl(acc_data_t *bmsdata)
{
	float max_temp = bmsdata->max_temp.val;
	float min_temp = bmsdata->min_temp.val;
	float min_cell_voltage = bmsdata->min_voltage.val;

	float temp_derate_factor = 0.0f;
	float cell_volt_derate_factor = 0.0f;

	// All cell discharge limits were obtained from P45B Datasheet.

	if (min_temp <= MIN_DISCHG_TEMP || max_temp >= MAX_CELL_TEMP ||
	    min_cell_voltage <= MIN_VOLT) {
		bmsdata->cont_DCL = 0.0f;
		return;
	}

	/* Temperature Derating: 50–55°C ramp down
	   Derating begins at 50°C to limit stress as the pack heats up.
	   DCL drops to 30A (10A per cell) at 55°C and shuts off above MAX_CELL_TEMP. */
	if (max_temp >= 55.0f) {
		temp_derate_factor = MIN_DCL / (float)(MAX_PACK_DISCHG_CURR);
	} else if (max_temp > 50.0f) {
		temp_derate_factor =
			1.0f - ((max_temp - 50.0f) / 5.0f) *
				       (1.0f - (MIN_DCL /
						(float)(MAX_PACK_DISCHG_CURR)));
	} else {
		temp_derate_factor = 1.0f;
	}

	/* Cell Voltage Derating: 3.0–2.5V ramp down
	   Below 3.0V, the pack begins reducing DCL to avoid deep discharge.
	   DCL drops to 30A at 2.5V, and shuts off completely below MIN_VOLT. */
	if (min_cell_voltage < 3.0f && min_cell_voltage > 2.5f) {
		cell_volt_derate_factor =
			1.0f - ((3.0f - min_cell_voltage) / 0.5f) *
				       (1.0f - (MIN_DCL /
						(float)(MAX_PACK_DISCHG_CURR)));
	} else {
		cell_volt_derate_factor = 1.0f;
	}

	float scaled_dcl = MAX_PACK_DISCHG_CURR * temp_derate_factor *
			   cell_volt_derate_factor;

	if (scaled_dcl < MIN_DCL) {
		scaled_dcl = MIN_DCL;
	}

	bmsdata->cont_DCL = scaled_dcl;
}

void calc_cont_ccl(acc_data_t *bmsdata)
{
	float max_temp = bmsdata->max_temp.val;
	float max_cell_voltage = bmsdata->max_voltage.val;

	float temp_derate_factor = 0.0f;
	float cell_volt_derate_factor = 0.0f;

	// All cell charge limits were obtained from P45B Datasheet.

	/* Temperature Derating: 0–10°C ramp up, 45–60°C ramp down
	   10°C and 45°C chosen as safe margins from P45B charge temp limits. */
	if (max_temp <= MIN_CHG_TEMP || max_temp >= MAX_CELL_TEMP) {
		temp_derate_factor = 0.0f;
	} else if (max_temp < 10.0f) {
		temp_derate_factor =
			(max_temp - MIN_CHG_TEMP) / (10.0f - MIN_CHG_TEMP);
	} else if (max_temp <= 45.0f) {
		temp_derate_factor = 1.0f;
	} else {
		temp_derate_factor =
			(MAX_CELL_TEMP - max_temp) / (MAX_CELL_TEMP - 45.0f);
	}

	/* Cell Voltage Derating: 4.15–4.205V ramp down
	   4.15V was chosen to reduce current early and avoid overshooting the max limit. */
	if (max_cell_voltage >= MAX_CHARGE_VOLT) {
		cell_volt_derate_factor = 0.0f;
	} else if (max_cell_voltage > 4.15f) {
		cell_volt_derate_factor = (MAX_CHARGE_VOLT - max_cell_voltage) /
					  (MAX_CHARGE_VOLT - 4.15f);
	} else {
		cell_volt_derate_factor = 1.0f;
	}

	bmsdata->cont_CCL = MAX_PACK_CHG_CURR * temp_derate_factor *
			    cell_volt_derate_factor;
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