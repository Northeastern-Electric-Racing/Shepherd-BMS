#include "cell_data_logging.h"
#include "analyzer.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

// For timestamps
extern TIM_HandleTypeDef htim2;

// Ring buffer
static ringbuf_t cell_log_ring_buff;

// Storage buffer for cell data
static CellDataEntry_t cell_data_storage[NUM_OF_READINGS] = { 0 };

static uint32_t get_us_timestamp(void)
{
	return __HAL_TIM_GET_COUNTER(&htim2);
}

void cell_data_logger_init(void)
{
	rb_init(&cell_log_ring_buff, cell_data_storage, NUM_OF_READINGS,
		sizeof(CellDataEntry_t));
}

void cell_data_log_measurement(acc_data_t *bms_data)
{
	if (bms_data == NULL) {
		printf("No BMS data to log!!\r\n");
		return;
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

			new_entry.timestamp[chip_num][cell] =
				get_us_timestamp();
		}
	}

	rb_insert(&cell_log_ring_buff, &new_entry);
}

CellDataEntry_t *cell_data_log_get_last(void)
{
	return (CellDataEntry_t *)rb_get_head(&cell_log_ring_buff);
}

void cell_data_log_get_last_n(size_t n, CellDataEntry_t *out_buffer)
{
	if (n > NUM_OF_READINGS) {
		n = NUM_OF_READINGS;
	}

	rb_get_last_n(&cell_log_ring_buff, out_buffer, n);
}