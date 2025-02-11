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