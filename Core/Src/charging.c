#include "charging.h"
#include "stateMachine.h"
#include "segment.h"
#include "compute.h"
#include "bmsConfig.h"

#include <math.h>

#define min(a, b)                       \
	({                              \
		__typeof__(a) _a = (a); \
		__typeof__(b) _b = (b); \
		_a < _b ? _a : _b;      \
	})

/* Send cell balancing config to the segments */
void handle_balance_cells(acc_data_t *bmsdata)
{
	/* 
	 * The maximum number of cells to balance per chip, usually tuned
	 * for thermal reasons. With the lid off and cooling fans at maximum 
	 * power, we can sustain balancing 8 cells per chip.
	 */
	static const int MAX_BAL_CHIP = 8;

	/*
	 * A cell's voltage must be greater than this plus the low cell to balance. 
	 * This value is usually proportional to the delta.
	 */
	const float min_thresh = 0.010;

	for (size_t chip = 0; chip < NUM_CHIPS; chip++) {
		/* Number of cells in this chip. */
		uint8_t num_cells = get_num_cells(&bmsdata->chip_data[chip]);

		/* Clear balancing config. */
		for (uint8_t cell = 0; cell < num_cells; cell++) {
			bmsdata->discharge_config[chip][cell] = false;
		}

		/* Number of cells that can be balanced. */
		int cells_left = MAX_BAL_CHIP;

		/* Balance cells with an OCV above the threshold. */
		for (size_t cell = 0; (cell < num_cells) && (cells_left > 0);
		     cell++) {
			if (bmsdata->chip_data[chip].open_cell_voltage[cell] >
			    (bmsdata->min_ocv.val + min_thresh)) {
				bmsdata->discharge_config[chip][cell] = true;
				cells_left -= 1;
			}
		}
	}
}