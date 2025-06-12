#include "charging.h"
#include "stateMachine.h"
#include "segment.h"
#include "compute.h"
#include "bmsConfig.h"

#include <math.h>

typedef struct {
	uint8_t cell_num;
	float ocv_val;
} bal_cell_t;

#define min(a, b)                       \
	({                              \
		__typeof__(a) _a = (a); \
		__typeof__(b) _b = (b); \
		_a < _b ? _a : _b;      \
	})


static void merge_sort(uint8_t i, uint8_t j, bal_cell_t *src, bal_cell_t *dest) {
	
	if (i >= j) {
		return;
	}

    uint8_t mid = (i + j) / 2;
    merge_sort(i, mid, src, dest);
    merge_sort(mid + 1, j, src, dest);
	uint16_t p_left = i;
	uint16_t p_right = mid + 1;

	for (uint8_t k = i; k <= j; k++) {
		if (p_left > mid) {
			dest[k] = src[p_right++];
		} else if (p_right > j) {
			dest[k] = src[p_left++];
		}	
		else if (src[p_left].ocv_val >= src[p_right].ocv_val) {
			dest[k] = src[p_left++];
		} else {
			dest[k] = src[p_right++];
		}
	}

	for (uint8_t k = i; k <= j; k++) {
		src[k] = dest[k];
	}

	memcpy(&src[i], &dest[i], (j - i + 1) * sizeof(bal_cell_t));
}

/* Send cell balancing config to sthe segments */
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
		bal_cell_t ocv_vals[num_cells];

		/* Clear balancing config. */
		for (uint8_t cell = 0; cell < num_cells; cell++) {
			bmsdata->discharge_config[chip][cell] = false;
			ocv_vals[cell].cell_num = cell; 
			ocv_vals[cell].ocv_val = bmsdata->chip_data[chip].open_cell_voltage[cell];
		}
		
		bal_cell_t ocv_sorted[num_cells];
		// sort cell OCVs from greatest to least to know which
		// to prioritize balancing
		merge_sort(0, num_cells - 1, ocv_vals, ocv_sorted); 

		/* Number of cells that can be balanced. */
		int cells_left = MAX_BAL_CHIP;

		/* Balance cells with an OCV above the threshold. */
		for (size_t cell = 0; (cell < num_cells) && (cells_left > 0);
		     cell++) {
			if (ocv_sorted[cell].ocv_val > (bmsdata->min_ocv.val + min_thresh)) {
				bmsdata->discharge_config[chip][ocv_sorted[cell].cell_num] = true;
				cells_left -= 1;
			}
		}
	}
}