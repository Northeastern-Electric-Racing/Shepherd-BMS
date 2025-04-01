#ifndef CHARGING
#define CHARGING

#include "datastructs.h"

/**
 * @brief entrypoint for handling balancing of cells
 * 
 * @param bmsdata general BMS data struct
 */
void handle_balance_cells(acc_data_t *bmsdata);

#endif