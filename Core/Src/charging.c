#include "charging.h"
#include "stateMachine.h"
#include "segment.h"
#include "compute.h"
#include "bmsConfig.h"

#include <math.h>

/* Send cell balancing config to the segment */
void handle_balance_cells(acc_data_t *bmsdata)
{
	if (bmsdata->delt_ocv <= MAX_DELTA_V) {
		/* No balancing, return */
		// technically this should never be reached
		return;
	}

	/* Get cell voltage average and standard deviation */
	float low = bmsdata->min_ocv.val;
	float thresh = bmsdata->delt_ocv * 0.1;

	/* Set the threshold for balancing to (mu - sigma * STD_FACTOR) */

	/* Balance all cells above the threshold */
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		int num_cells = get_num_cells(&bmsdata->chip_data[chip]);
		for (int cell = 0; cell < num_cells; cell++) {
			/* Check if cell voltage is above (average - standard deviation) */
			if (bmsdata->chip_data[chip].open_cell_voltage[cell] >
			    (low + thresh)) {
				/* Balance cell */
				bmsdata->discharge_config[chip][cell] = true;
			} else {
				/* Do not balance cell */
				bmsdata->discharge_config[chip][cell] = false;
			}
		}
	}
}