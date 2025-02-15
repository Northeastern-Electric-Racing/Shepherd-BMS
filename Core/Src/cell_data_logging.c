#include "cell_data_logging.h"
#include "analyzer.h"
#include "bmsConfig.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

struct BMSLogger {
	ringbuf_t ring_buff;
	CellDataEntry_t cell_data_storage[NUM_OF_READINGS];
};

static BMSLogger bms_logger;

BMSLogger *getLogger(void)
{
	return &bms_logger;
}

bool cell_data_logger_init(BMSLogger *logger)
{
	if (logger == NULL) {
		return true;
	}

	memset(logger, 0, sizeof(BMSLogger));

	rb_init(&logger->ring_buff, logger->cell_data_storage, NUM_OF_READINGS,
		sizeof(CellDataEntry_t));

	return false;
}

bool cell_data_log_measurement(BMSLogger *logger, acc_data_t *bms_data)
{
	if (logger == NULL || bms_data == NULL) {
		printf("No BMS data to log!!\r\n");
		return true;
	}

	CellDataEntry_t new_entry;

	memset(&new_entry, 0, sizeof(CellDataEntry_t));

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

	return false;
}

CellDataEntry_t *cell_data_log_get_last(const BMSLogger *logger)
{
	if (logger == NULL) {
		return NULL;
	}

	return rb_get_head(&logger->ring_buff);
}

bool cell_data_log_get_last_n(const BMSLogger *logger, size_t n,
			      CellDataEntry_t *out_buffer)
{
	if (logger == NULL) {
		return true;
	}

	if (n > NUM_OF_READINGS) {
		n = NUM_OF_READINGS;
	}

	rb_get_last_n(&logger->ring_buff, out_buffer, n);
	return false;
}

void print_latest_cell_data_log(const BMSLogger *logger)
{
	if (logger == NULL) {
		printf("Logger not initialized!!");
		return;
	}

	const CellDataEntry_t *latest_entry =
		(const CellDataEntry_t *)cell_data_log_get_last(logger);

	if (latest_entry == NULL) {
		printf("No data available!!\r\n");
		return;
	}

	for (int chip_num = 0; chip_num < NUM_CHIPS; chip_num++) {
		int cell_count = (chip_num % 2 == 0) ? NUM_CELLS_ALPHA :
						       NUM_CELLS_BETA;
		printf("Chip %d (%s):\r\n", chip_num,
		       (chip_num % 2 == 0) ? "Alpha" : "Beta");

		for (int cell = 0; cell < cell_count; cell++) {
			printf("  Cell %d: Voltage: %.3f V, Temperature: %.2f C, Timestamp: %lu µs\r\n",
			       cell + 1,
			       latest_entry->cell_voltages[chip_num][cell],
			       latest_entry->cell_temperatures[chip_num][cell],
			       latest_entry->timestamp[chip_num][cell]);
		}
	}
}

void print_last_n_cell_data_logs(const BMSLogger *logger, size_t n)
{
	if (logger == NULL) {
		printf("Logger not initialized!!");
		return;
	}

	if (n > NUM_OF_READINGS) {
		n = NUM_OF_READINGS;
	}

	CellDataEntry_t log_entries[n];
	if (cell_data_log_get_last_n(logger, n, log_entries)) {
		printf("Error retrieving log entries!\r\n");
		return;
	}

	printf("Printing Last %zu Cell Data Logs:\r\n", n);

	for (size_t entry_idx = 0; entry_idx < n; entry_idx++) {
		printf("\r\n--- Log Entry %zu ---\r\n", entry_idx + 1);

		for (int chip_num = 0; chip_num < NUM_CHIPS; chip_num++) {
			int cell_count = (chip_num % 2 == 0) ? NUM_CELLS_ALPHA :
							       NUM_CELLS_BETA;
			printf("Chip %d (%s):\r\n", chip_num,
			       (chip_num % 2 == 0) ? "Alpha" : "Beta");

			for (int cell = 0; cell < cell_count; cell++) {
				printf("  Cell %d: Voltage: %.3f V, Temperature: %.2f C, Timestamp: %lu µs\r\n",
				       cell + 1,
				       log_entries[entry_idx]
					       .cell_voltages[chip_num][cell],
				       log_entries[entry_idx]
					       .cell_temperatures[chip_num]
								 [cell],
				       log_entries[entry_idx]
					       .timestamp[chip_num][cell]);
			}
		}
	}
}