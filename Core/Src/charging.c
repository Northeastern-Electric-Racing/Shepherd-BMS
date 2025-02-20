#include "charging.h"
#include "stateMachine.h"
#include "segment.h"
#include "compute.h"
#include "BMSConfig.h"

// TODO: Do we need a minimum voltage threshold (BAL_MIN_V)?
/**
 * @brief Balance cells if necessary
 * 
 * @param bmsdata BMS data struct
 */
void balance_cells(acc_data_t *bmsdata)
{
	/* Get pack delta */
	float pack_delta = calc_pack_delta(bmsdata);
	if (pack_delta > MAX_DELTA_V || sm_balancing_check(bmsdata) == false) {
		/* No balancing, return */
		segment_disable_balancing(bmsdata);
		return;
	}

	bool balanceConfig[NUM_CHIPS][NUM_CELLS_ALPHA];

	/* Get cell voltage average and standard deviation */
	float avg = calc_cell_voltage_average(bmsdata);
	float std = calc_cell_voltage_std(bmsdata);

	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int cell = 0; cell < NUM_CELLS_ALPHA; cell++) {
			/* Check if cell voltage is above (average - standard deviation) */
			if (bmsdata->chip_data[chip].cell_voltages[cell] >
			    avg - std) {
				/* Balance cell */
				balanceConfig[chip][cell] = true;
			} else {
				/* Do not balance cell */
				balanceConfig[chip][cell] = false;
			}
		}
	}

	/* Configure balancing */
	segment_configure_balancing(bmsdata, balanceConfig);
}

/**
 * @brief Calculate the difference between the maximum votlage and 
 * minimum voltage in a cell of the whole pack
 * 
 * @param data BMS data struct
 * @return float 
 */
float calc_pack_delta(acc_data_t *data)
{
	float max = 0;
	float min = MAX_VOLT;
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int cell = 0; cell < NUM_CELLS_ALPHA; cell++) {
			if (data->chip_data[chip].cell_voltages[cell] > max) {
				max = data->chip_data[chip].cell_voltages[cell];
			}
			if (data->chip_data[chip].cell_voltages[cell] < min) {
				min = data->chip_data[chip].cell_voltages[cell];
			}
		}
	}
	return max - min;
}

/**
 * @brief Calculate the average cell voltage
 * 
 * @param data BMS data struct
 * @return float 
 */
float calc_cell_voltage_average(acc_data_t *data)
{
	float avg = 0;
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int cell = 0; cell < NUM_CELLS_ALPHA; cell++) {
			avg += data->chip_data[chip].cell_voltages[cell];
		}
	}
	avg /= NUM_CELLS_ALPHA * NUM_CHIPS;
}

/**
 * @brief Calculate the standard deviation of the cell voltages
 * 
 * @param data BMS data struct
 * @return float 
 */
float calc_cell_voltage_std(acc_data_t *data)
{
	float avg = calc_cell_voltage_average(data);

	/* Calculate mean squared error */
	float mse = 0;
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int cell = 0; cell < NUM_CELLS_ALPHA; cell++) {
			mse += pow(data->chip_data[chip].cell_voltages[cell] -
					   avg,
				   2);
		}
	}

	/* Calculate standard deviation */
	float std = pow(mse / (NUM_CELLS_ALPHA * NUM_CHIPS), 0.5);
	return std;
}