#include "charging.h"
#include "stateMachine.h"
#include "segment.h"
#include "compute.h"
#include "bmsConfig.h"

#include <math.h>

/* Constants */
static const uint8_t STD_FACTOR = 1;

/* Find standard deviation from BMS data */
static float calc_cell_voltage_std(acc_data_t *data)
{
	/* Calculate mean squared error */
	float mse = 0;
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int cell = 0; cell < NUM_CELLS_ALPHA; cell++) {
			float cell_voltage =
				data->chip_data[chip].cell_voltages[cell];
			mse += pow(cell_voltage - data->avg_voltage, 2);
		}
	}

	/* Calculate standard deviation */
	float std = pow(mse / (NUM_CELLS_ALPHA * NUM_CHIPS), 0.5);
	return std;
}

/* Send cell balancing config to the segment */
void handle_balance_cells(acc_data_t *bmsdata)
{
	if (bmsdata->delt_voltage <= MAX_DELTA_V) {
		/* No balancing, return */
		// technically this should never be reached
		segment_disable_balancing(bmsdata);
		return;
	}

	bool balanceConfig[NUM_CHIPS][NUM_CELLS_ALPHA];

	/* Get cell voltage average and standard deviation */
	float avg = bmsdata->avg_voltage;
	float std = calc_cell_voltage_std(bmsdata);

	/* Set the threshold for balancing to (mu - sigma * STD_FACTOR) */
	float thresh = avg - (STD_FACTOR * std);

	/* Balance all cells above the threshold */
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int cell = 0; cell < NUM_CELLS_ALPHA; cell++) {
			/* Check if cell voltage is above (average - standard deviation) */
			if (bmsdata->chip_data[chip].cell_voltages[cell] >
			    thresh) {
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