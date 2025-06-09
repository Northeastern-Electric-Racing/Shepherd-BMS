#ifndef CHARGING
#define CHARGING

#include "datastructs.h"

/**
 * @brief entrypoint for handling balancing of cells.  DOES NOT ENABLE BALANCING, but does configure it.
 * 
 * @param bmsdata general BMS data struct
 */
void handle_balance_cells(acc_data_t *bmsdata);

#endif