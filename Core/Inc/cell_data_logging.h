/**
 * @file cell_data_logging.h
 * @brief Functionality for logging and retrieving cell voltage and temperature data.
 */

#ifndef CELL_DATA_LOGGING_H
#define CELL_DATA_LOGGING_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "datastructs.h"
#include "ringbuffer.h"
#include "cmsis_os.h"

// Number of stored cell data readings in ring buffer.
#define NUM_OF_READINGS 10

/**
 * @struct CellDataEntry_t
 * @brief Structure to store logged cell voltage and temperature data.
 *
 * This structure is used to store the timestamp, cell voltages, 
 * and cell temperatures for each measurement cycle. Each entry 
 * contains data for all chips and their respective cells.
 */
typedef struct {
	uint32_t cell_voltage_timestamp;
	uint32_t cell_temperature_timestamp;
	float cell_voltages[NUM_CHIPS][NUM_CELLS_ALPHA];
	float cell_temperatures[NUM_CHIPS][NUM_CELLS_ALPHA];
} CellDataEntry_t;

/**
 * @struct BMSLogger
 * @brief Structure to manage logging system
 */
struct BMSLogger;

/**
 * @brief Retrieves the global instance of the BMSLogger.
 * @return Pointer to the global `BMSLogger` instance.
 */
struct BMSLogger *get_logger(void);

/**
 * @brief Retrieves the current timestamp in microseconds from TIM2.
 * @return The current timestamp in microseconds.
 */
uint32_t get_us_timestamp(void);

/**
 * @brief Initializes a BMSLogger instance.
 * @param logger Pointer to the logger instance.
 * @return 0 on success, -1 on failure.
 */
int cell_data_logger_init(struct BMSLogger *logger);

/**
 * @brief Logs a new measurement and inserts it in the ring buffer.
 * @param logger Pointer to the logger instance.
 * @param bms_data Pointer to the BMS data containing chip cell voltages and temperatures.
 * @return 0 on success, -1 on failure.
 */
int cell_data_log_measurement(struct BMSLogger *logger, acc_data_t *bms_data);

/**
 * @brief Gets the most recent cell data log from the buffer.
 * @param logger Pointer to the logger instance.
 * @return Pointer to the most recent data entry, or NULL if the logger is empty.
 */
CellDataEntry_t *cell_data_log_get_last(const struct BMSLogger *logger);

/**
 * @brief Retrieves the last n cell data logs from the buffer.
 * @param logger Pointer to the logger instance.
 * @param n Number of previous logs to retrieve.
 * @param out_buffer Pointer to the buffer where the readings will be stored.
 * @return 0 on success, -1 on failure.
 */
int cell_data_log_get_last_n(const struct BMSLogger *logger, size_t n,
			     CellDataEntry_t *out_buffer);

/**
 * @brief Serial prints the last n cell data logs.
 * @param logger Pointer to the logger instance.
 * @param n Number of previous logs to print.
 * @return 0 on success, -1 on failure.
 */
int print_last_n_cell_data_logs(const struct BMSLogger *logger, size_t n);

#endif