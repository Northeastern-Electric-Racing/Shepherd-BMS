#ifndef ANALYZER_H
#define ANALYZER_H

#include "datastructs.h"

/**
 * @brief Get the number of cells on a chip.
 * 
 * @param chip_data Pointer to struct containing chip data.
 * @return uint8_t The number of cells in the chip.
 */
uint8_t get_num_cells(chipdata_t *chip_data);

/**
 * @brief Calculate thermistor values and cell temps using thermistors.
 * 
 */
void calc_cell_temps(acc_data_t *bmsdata);

/**
 * @brief Calculates pack temp, and avg, min, and max cell temperatures.
 * 
 */
void calc_pack_temps(acc_data_t *bmsdata);

/**
 * @brief Calclaute the voltage of every cell in the pack.
 * 
 * @param bmsdata Pointer to BMS data struct.
 */
void calc_cell_voltages(acc_data_t *bmsdata);

/**
 * @brief Calculate statistics about pack voltage, such as min and max cell volt, pack and avg voltage, pack and avg OCV, and deltas.
 * 
 */
void calc_pack_voltage_stats(acc_data_t *bmsdata);

/**
 * @brief Calculate open cell voltages based on cell voltages and previous open cell voltages.
 * 
 */
void calc_open_cell_voltage(acc_data_t *bmsdata);

/**
 * @brief Calculate cell resistances using Rin = ( Voc - V )/I
 * 
 */
void calc_cell_resistances(acc_data_t *bmsdata);

/**
 * @brief Calculate the continuous discharge current limit based on cell temperatures and a cell temp to DCL lookup table.
 * 
 */
void calc_cont_dcl(acc_data_t *bmsdata);

/**
 * @brief Calculate continuous charge current limit (CCL) using linear derating from max cell temperature and voltage.
 *
 * @param bmsdata Pointer to BMS data structure.
 */
void calc_cont_ccl(acc_data_t *bmsdata);

/**
 * @brief Calcuate state of charge by using a formula based on datasheet graph
 * 
 * @param bmsdata Pointer to BMS data struct.
 */
void calc_state_of_charge(acc_data_t *bmsdata);

#endif