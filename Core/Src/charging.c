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

/// @brief A struct to hold the original float value and the index originally, as that holds meaning
typedef struct {
	float val;
	size_t idex;
} val_idexed_t;

/**
 * @brief selection sorts ocv into structs that remember values
 * @param arr 
 * @param n count
 */
void chipsSelectionSort(acc_data_t *bmsdata,
			val_idexed_t replaced_val[NUM_CHIPS][NUM_CELLS_ALPHA])
{
	for (size_t chip = 0; chip < NUM_CHIPS; chip++) {
		uint8_t cells = get_num_cells(bmsdata[chip].chip_data);
		// first fill the outer row
		for (int i = 0; i < cells; i++) {
			replaced_val[chip][i] = (val_idexed_t){
				.idex = i,
				.val = bmsdata->chip_data[chip]
					       .open_cell_voltage[i]
			};
		}

		// now actually sort it
		for (size_t i = 0; i < cells - 1; i++) {
			// Assume the current position holds
			// the minimum element
			size_t max_idx = i;

			// Iterate through the unsorted portion
			// to find the actual minimum
			for (size_t j = i + 1; j < cells; j++) {
				if (replaced_val[chip][j].val >
				    replaced_val[chip][max_idx].val) {
					// Update min_idx if a smaller element is found
					max_idx = j;
				}
			}

			// Move minimum element to its
			// correct position
			val_idexed_t temp = replaced_val[chip][i];
			replaced_val[chip][i] = replaced_val[chip][max_idx];
			replaced_val[chip][max_idx] = temp;
		}
	}
}

static PWM_DUTY calc_pwm_duty(acc_data_t* bmsdata, float curr_val) {
	// the low cell, eventually they all must get there
	float low = bmsdata->min_ocv.val;
	// the margin above the low cell to ignore, which is usually X% of the delta
	float coeff = 0.4;

	for (PWM_DUTY i = PWM_100_0_PCT; i > PWM_0_0_PCT; i--) {
		if (curr_val > (low + bmsdata->delt_ocv * (coeff + (i * (1 - coeff) / (1.0 * PWM_100_0_PCT))))) {
			return i;
		}
	} 
	return PWM_0_0_PCT;
}

/* Send cell balancing config to the segments */
void handle_balance_cells(acc_data_t *bmsdata)
{
	// the maximum number of cells to balance per chip, usually tuned for thermal reasons
	static const int MAX_BAL_CHIP = 7;
	val_idexed_t new_ocv_map[NUM_CHIPS][NUM_CELLS_ALPHA] = { 0 };

	// first, sort and cleanup everything
	chipsSelectionSort(bmsdata, new_ocv_map);

	/* Balance all cells above the threshold, using the sorted ocv map values but preserve the indexes*/
	for (size_t chip = 0; chip < NUM_CHIPS; chip++) {
		// ONLY iterate to MAX_BAL or the number of cells, whatever is lower.
		// this is OK because they are sorted greatest to least in delta
		int cell_max = min(get_num_cells(bmsdata[chip].chip_data),
				   MAX_BAL_CHIP);
		for (size_t cell = 0; cell < cell_max; cell++) {
			bmsdata->discharge_config
					[chip][new_ocv_map[chip][cell].idex] =
					calc_pwm_duty(bmsdata, new_ocv_map[chip][cell].val);
		}
	}
}