/**
 * @file cell_data_logging.c
 * @brief Implementation of cell voltage and temperature data logging.
 */

#include "cell_data_logging.h"
#include "analyzer.h"
#include "bmsConfig.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

// Used to get microsecond timestamps.
extern TIM_HandleTypeDef htim2;

/**
 * @brief Prints a single cell data entry.
 * @param entry Pointer to the cell data entry to print.
 * @param entry_idx Index of the entry.
 */
static void print_cell_data(const CellDataEntry_t *entry, size_t entry_idx)
{
	assert(entry != NULL);

	printf("\r\n--- Log Entry %zu ---\r\n", entry_idx + 1);
	printf("Voltage Measurement Timestamp: %lu µs\r\n",
	       entry->cell_voltage_timestamp);
	printf("Temperature Measurement Timestamp: %lu µs\r\n",
	       entry->cell_temperature_timestamp);

	for (int chip_num = 0; chip_num < NUM_CHIPS; chip_num++) {
		int cell_count = (chip_num % 2 == 0) ? NUM_CELLS_ALPHA :
						       NUM_CELLS_BETA;
		printf("\r\nChip %d (%s):\r\n", chip_num,
		       (chip_num % 2 == 0) ? "Alpha" : "Beta");

		for (int cell = 0; cell < cell_count; cell++) {
			printf("  Cell %d: Voltage: %.3f V, Temperature: %.2f C\r\n",
			       cell + 1, entry->cell_voltages[chip_num][cell],
			       entry->cell_temperatures[chip_num][cell]);
		}
	}
}

/**
 * @brief Retrieves the current timestamp in microseconds from TIM2.
 * @return The current timestamp in microseconds.
 */
static uint32_t get_us_timestamp(void)
{
	return __HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief Initializes a BMSLogger instance.
 * @param logger Pointer to the logger instance.
 * @return 0 on success, -1 on failure.
 */
int cell_data_logger_init(struct BMSLogger *logger)
{
	assert(logger != NULL);

	memset(logger, 0, sizeof(struct BMSLogger));

	rb_init(&logger->ring_buff, logger->cell_data_storage, NUM_OF_READINGS,
		sizeof(CellDataEntry_t));

	logger->mutex = osMutexNew(NULL);

	if (logger->mutex == NULL) {
		printf("ERROR: Data Logger Mutex initialization failed!\r\n");
		return -1;
	}

	return 0;
}

/**
 * @brief Assigns a voltage timestamp to the current log entry. 
 * @param logger Pointer to the BMSLogger instance.
 * @return 0 on success, -1 on failure.
 */
int cell_data_logger_timestamp_voltage(struct BMSLogger *logger)
{
	assert(logger != NULL);

	if (osMutexAcquire(logger->mutex, osWaitForever) != osOK) {
		printf("ERROR: Failed to aquire data logging mutex!\r\n");
		return -1;
	}

	if (!logger->first_timestamp_set) {
		size_t next_idx = (logger->ring_buff.head_idx + 1) %
				  logger->ring_buff.size;

		logger->latest_entry = &logger->cell_data_storage[next_idx];

		logger->first_timestamp_set = true;
	}

	assert(logger->latest_entry != NULL);
	logger->latest_entry->cell_voltage_timestamp = get_us_timestamp();

	osMutexRelease(logger->mutex);
	return 0;
}

/**
 * @brief Assigns a temperature timestamp to the current log entry.
 * @param logger Pointer to the BMSLogger instance.
 * @return 0 on success, -1 on failure.
 */
int cell_data_logger_timestamp_therms(struct BMSLogger *logger)
{
	assert(logger != NULL);

	if (osMutexAcquire(logger->mutex, osWaitForever) != osOK) {
		printf("ERROR: Failed to aquire data logging mutex!\r\n");
		return -1;
	}

	if (!logger->first_timestamp_set) {
		size_t next_idx = (logger->ring_buff.head_idx + 1) %
				  logger->ring_buff.size;

		logger->latest_entry = &logger->cell_data_storage[next_idx];

		logger->first_timestamp_set = true;
	}

	assert(logger->latest_entry != NULL);
	logger->latest_entry->cell_temperature_timestamp = get_us_timestamp();

	osMutexRelease(logger->mutex);
	return 0;
}

/**
 * @brief Logs a new measurement and inserts it into the ring buffer.
 * 
 * @note The user is responsible for ensuring the timestamps are set before calling this function.
 * 
 * @param logger Pointer to the BMSLogger instance managing the ring buffer.
 * @param bms_data Pointer to the BMS data structure containing cell voltages and temperatures.
 * @return 0 on success, -1 on failure.
 */
int cell_data_log_measurement(struct BMSLogger *logger, acc_data_t *bms_data)
{
	int status = -1;
	assert(logger != NULL);
	assert(bms_data != NULL);

	if (osMutexAcquire(logger->mutex, osWaitForever) != osOK) {
		printf("ERROR: Failed to aquire data logging mutex!\r\n");
		goto exit;
	}

	if (logger->latest_entry == NULL) {
		printf("ERROR: No timestamped entry available!\r\n");
		osMutexRelease(logger->mutex);
		goto exit;
	}

	for (int chip_num = 0; chip_num < NUM_CHIPS; chip_num++) {
		int cell_count = get_num_cells(&bms_data->chip_data[chip_num]);

		for (int cell = 0; cell < cell_count; cell++) {
			logger->latest_entry->cell_voltages[chip_num][cell] =
				bms_data->chip_data[chip_num]
					.cell_voltages[cell];

			logger->latest_entry->cell_temperatures[chip_num][cell] =
				bms_data->chip_data[chip_num].cell_temp[cell];
		}
	}

	rb_insert(&logger->ring_buff, logger->latest_entry);
	logger->latest_entry = NULL;
	logger->first_timestamp_set = false;

	status = 0;
	osMutexRelease(logger->mutex);
	return status;

exit:
	return status;
}

/**
 * @brief Gets the most recent cell data log from the buffer.
 * @param logger Pointer to the logger instance.
 * @return Pointer to the most recent data entry, or NULL if the logger is empty.
 */
CellDataEntry_t *cell_data_log_get_last(const struct BMSLogger *logger)
{
	CellDataEntry_t *last_entry = NULL;
	assert(logger != NULL);

	if (logger->ring_buff.curr_elements == 0) {
		printf("ERROR: No logs available!\r\n");
		goto exit;
	}

	if (osMutexAcquire(logger->mutex, osWaitForever) != osOK) {
		printf("ERROR: Failed to aquire data logging mutex!\r\n");
		goto exit;
	}

	last_entry = rb_get_head(&logger->ring_buff);

	osMutexRelease(logger->mutex);

exit:
	return last_entry;
}

/**
 * @brief Retrieves the last n cell data logs from the buffer.
 * @param logger Pointer to the logger instance.
 * @param n Number of previous logs to retrieve.
 * @param out_buffer Pointer to the buffer where the readings will be stored.
 * @return 0 on success, -1 on failure.
 */
int cell_data_log_get_last_n(const struct BMSLogger *logger, size_t n,
			     CellDataEntry_t *out_buffer)
{
	int status = -1;
	assert(logger != NULL);

	if (n > logger->ring_buff.curr_elements) {
		printf("ERROR: Not enough logs available!\r\n");
		goto exit;
	}

	if (osMutexAcquire(logger->mutex, osWaitForever) != osOK) {
		printf("ERROR: Failed to aquire data logging mutex!\r\n");
		goto exit;
	}

	rb_get_last_n(&logger->ring_buff, out_buffer, n);

	status = 0;

	osMutexRelease(logger->mutex);

exit:
	return status;
}

/**
 * @brief Serial prints the last n cell data logs.
 * @param logger Pointer to the logger instance.
 * @param n Number of previous logs to print.
 * @return 0 on success, -1 on failure.
 */
int print_last_n_cell_data_logs(const struct BMSLogger *logger, size_t n)
{
	int status = -1;
	assert(logger != NULL);

	CellDataEntry_t log_entries[n];

	if (cell_data_log_get_last_n(logger, n, log_entries)) {
		printf("Error retrieving log entries!\r\n");
		goto exit;
	}

	printf("\r\nPrinting Last %zu Cell Data Logs:\r\n", n);
	for (size_t entry_idx = 0; entry_idx < n; entry_idx++) {
		print_cell_data(&log_entries[entry_idx], entry_idx);
	}

	status = 0;

exit:
	return status;
}