#include "cell_data_logging.h"
#include "analyzer.h"
#include "bmsConfig.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

extern TIM_HandleTypeDef htim2;

struct BMSLogger {
	ringbuf_t ring_buff;
	CellDataEntry_t cell_data_storage[NUM_OF_READINGS];
	osMutexId_t mutex;
};

uint32_t get_us_timestamp(void)
{
	return __HAL_TIM_GET_COUNTER(&htim2);
}

int cell_data_logger_init(BMSLogger *logger)
{
	assert(logger != NULL);

	memset(logger, 0, sizeof(BMSLogger));

	rb_init(&logger->ring_buff, logger->cell_data_storage, NUM_OF_READINGS,
		sizeof(CellDataEntry_t));

	logger->mutex = osMutexNew(NULL);

	if (logger->mutex == NULL) {
		printf("ERROR: Data Logger Mutex initialization failed!\r\n");
		return -1;
	}

	return 0;
}

int cell_data_log_measurement(BMSLogger *logger, acc_data_t *bms_data)
{
	int status = -1;

	assert(logger != NULL);
	assert(bms_data != NULL);

	if (osMutexAcquire(logger->mutex, LOGGER_MUTEX_WAIT_TIME) != osOK) {
		printf("ERROR: Failed to aquire data logging mutex!\r\n");
		goto exit;
	}

	CellDataEntry_t new_entry;

	memset(&new_entry, 0, sizeof(CellDataEntry_t));

	new_entry.cell_voltage_timestamp = bms_data->voltage_timestamp;
	new_entry.cell_temperature_timestamp = bms_data->temperature_timestamp;

	for (int chip_num = 0; chip_num < NUM_CHIPS; chip_num++) {
		int cell_count = get_num_cells(&bms_data->chip_data[chip_num]);

		for (int cell = 0; cell < cell_count; cell++) {
			new_entry.cell_voltages[chip_num][cell] =
				bms_data->chip_data[chip_num]
					.cell_voltages[cell];

			new_entry.cell_temperatures[chip_num][cell] =
				bms_data->chip_data[chip_num].cell_temp[cell];
		}
	}

	rb_insert(&logger->ring_buff, &new_entry);

	status = 0;

	osMutexRelease(logger->mutex);

exit:
	return status;
}

CellDataEntry_t *cell_data_log_get_last(const BMSLogger *logger)
{
	CellDataEntry_t *last_entry = NULL;

	assert(logger != NULL);

	if (logger->ring_buff.curr_elements == 0) {
		printf("ERROR: No logs available!\r\n");
		goto exit;
	}

	if (osMutexAcquire(logger->mutex, LOGGER_MUTEX_WAIT_TIME) != osOK) {
		printf("ERROR: Failed to aquire data logging mutex!\r\n");
		goto exit;
	}

	last_entry = rb_get_head(&logger->ring_buff);

	osMutexRelease(logger->mutex);

exit:
	return last_entry;
}

int cell_data_log_get_last_n(const BMSLogger *logger, size_t n,
			     CellDataEntry_t *out_buffer)
{
	int status = -1;

	assert(logger != NULL);

	if (n > logger->ring_buff.curr_elements) {
		printf("ERROR: Not enough logs available!\r\n");
		goto exit;
	}

	if (osMutexAcquire(logger->mutex, LOGGER_MUTEX_WAIT_TIME) != osOK) {
		printf("ERROR: Failed to aquire data logging mutex!\r\n");
		goto exit;
	}

	rb_get_last_n(&logger->ring_buff, out_buffer, n);

	status = 0;

	osMutexRelease(logger->mutex);

exit:
	return status;
}

int print_last_n_cell_data_logs(const BMSLogger *logger, size_t n)
{
	int status = -1;

	assert(logger != NULL);

	CellDataEntry_t log_entries[n];

	if (cell_data_log_get_last_n(logger, n, log_entries)) {
		printf("Error retrieving log entries!\r\n");
		goto exit;
	}

	printf("\r\nPrinting Last %u Cell Data Logs:\r\n", n);

	for (size_t entry_idx = 0; entry_idx < n; entry_idx++) {
		printf("\r\n--- Log Entry %u ---\r\n", entry_idx + 1);
		printf("Voltage Measurement Timestamp: %lu µs\r\n",
		       log_entries[entry_idx].cell_voltage_timestamp);
		printf("Temperature Measurement Timestamp: %lu µs\r\n",
		       log_entries[entry_idx].cell_temperature_timestamp);

		for (int chip_num = 0; chip_num < NUM_CHIPS; chip_num++) {
			int cell_count = (chip_num % 2 == 0) ? NUM_CELLS_ALPHA :
							       NUM_CELLS_BETA;
			printf("\r\nChip %d (%s):\r\n", chip_num,
			       (chip_num % 2 == 0) ? "Alpha" : "Beta");

			for (int cell = 0; cell < cell_count; cell++) {
				printf("  Cell %d: Voltage: %.3f V, Temperature: %.2f C\r\n",
				       cell + 1,
				       log_entries[entry_idx]
					       .cell_voltages[chip_num][cell],
				       log_entries[entry_idx]
					       .cell_temperatures[chip_num]
								 [cell]);
			}
		}
	}

	status = 0;

exit:
	return status;
}