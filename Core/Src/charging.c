#include "charging.h"
#include "stateMachine.h"
#include "segment.h"
#include "compute.h"
#include "BMSConfig.h"

/* Private prototypes */
static float calc_cell_voltage_std(acc_data_t *data);

/* Find standard deviation from BMS data */
float calc_cell_voltage_std(acc_data_t *data)
{
	float avg = calc_cell_voltage_average(data);

	/* Calculate mean squared error */
	float mse = 0;
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int cell = 0; cell < NUM_CELLS_ALPHA; cell++) {
			float cell_voltage =
				data->chip_data[chip].cell_voltages[cell];
			mse += pow(cell_voltage - avg, 2);
		}
	}

	/* Calculate standard deviation */
	float std = pow(mse / (NUM_CELLS_ALPHA * NUM_CHIPS), 0.5);
	return std;
}

void handle_balance_cells(acc_data_t *bmsdata)
{
	if (bmsdata->delt_voltage > MAX_DELTA_V ||
	    sm_balancing_check(bmsdata) == false) {
		/* No balancing, return */
		segment_disable_balancing(bmsdata);
		return;
	}

	bool balanceConfig[NUM_CHIPS][NUM_CELLS_ALPHA];

	/* Get cell voltage average and standard deviation */
	float avg = bmsdata->avg_voltage;
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