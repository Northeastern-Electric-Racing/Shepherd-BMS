/**
 * @file cell_data_logging.h
 * @brief Functionality for logging and retrieving cell voltage and temperature data.
 */

 #ifndef CELL_DATA_LOGGING_H
 #define CELL_DATA_LOGGING_H
 
 #include <stdint.h>
 #include <stddef.h>
 #include "bmsConfig.h"
 #include "datastructs.h"
 #include "ringbuffer.h"
 
 #define NUM_OF_READINGS 10
 
 typedef struct {
     uint32_t timestamp[NUM_CHIPS][NUM_CELLS_ALPHA];
     float cell_voltages[NUM_CHIPS][NUM_CELLS_ALPHA];
     float cell_temperatures[NUM_CHIPS][NUM_CELLS_ALPHA];
 } CellDataEntry_t;
 
 /**
  * @brief Gets the current microsecond timestamp from TIM2.
  */
 uint32_t get_us_timestamp(void);
 
 /**
  * @brief Sets up the ring buffer for cell data logging.
  */
 void cell_data_logger_init(void);
 
 /**
  * @brief Logs a new measurement and inserts it in the ring buffer.
  * @param bms_data Pointer to the BMS data containing chip cell voltages and temperatures.
  */
 void cell_data_log_measurement(acc_data_t *bms_data);
 
 /**
  * @brief Gets the most recent cell data log from the buffer.
  * @return Pointer to the most recent CellDataEntry_t data.
  */
 CellDataEntry_t *cell_data_log_get_last(void);
 
 /**
  * @brief Retrieves the last n cell data logs from the buffer.
  * @param n Number of previous cell data logs to retrieve.
  * @param out_buffer Pointer to the buffer where the readings will be stored.
  */
 void cell_data_log_get_last_n(size_t n, CellDataEntry_t *out_buffer);
 
 /**
  * @brief Serial prints the most recent data log.
  */
 void print_latest_cell_data_log(void);
 
 /**
  * @brief Serial prints the last n cell data logs.
  * @param n Number of previous cell data logs to retrieve to print.
  */
 void print_last_n_cell_data_logs(size_t n);
 
 #endif 