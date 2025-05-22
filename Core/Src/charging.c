#include "charging.h"
#include "stateMachine.h"
#include "segment.h"
#include "compute.h"
#include "bmsConfig.h"

#include <math.h>

/* Constants */
static const uint8_t STD_FACTOR = 1;
nertimer_t bal_timer = { .active = false };


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
	static bool is_balancing = false;

	if (bmsdata->delt_ocv <= MAX_DELTA_V) {
		/* No balancing, return */
		// technically this should never be reached
		segment_disable_balancing(bmsdata);
		return;
	}

	
	bool balanceConfig[NUM_CHIPS][NUM_CELLS_ALPHA] = { 0 };

	/* Get cell voltage average and standard deviation */
	float low = bmsdata->min_ocv.val;
	float thresh = bmsdata->delt_ocv * 0.8;

	/* Set the threshold for balancing to (mu - sigma * STD_FACTOR) */

	/* Balance all cells above the threshold */
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int cell = 0; cell < NUM_CELLS_ALPHA; cell++) {
			/* Check if cell voltage is above (average - standard deviation) */
			if (bmsdata->chip_data[chip].open_cell_voltage[cell] >
			    (low + thresh)) {
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

	// enable balancing
	if (!is_timer_active(&bal_timer) || is_timer_expired(&bal_timer)) {
		if (is_balancing) {
			segment_disable_balancing(bmsdata);
			start_timer(&bal_timer, 160 * 1000);
		} else {
			printf("STARTING TO BALANCE.----------------------\n");
			segment_enable_balancing(bmsdata);
			start_timer(&bal_timer, 10 * 60 * 1000);
		}
		is_balancing = !is_balancing;	
	} 
}