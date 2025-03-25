/**
 * @file eepromdirectory.h
 * @brief Functionality for EEPROM partition management, data read/write operations, and fault logging.
 * 
 * @note Per M24C32 datasheet, ensure a minimum 5 ms delay after each EEPROM write to avoid data corruption (tWR = 5 ms max).
 * It is the user's responsibility to maintain this delay.
 *
 * Functions return `eeprom_status_t` error codes, which are defined in `eeprom_status.h`.
 */

#ifndef EEPROMDIRECTORY_H
#define EEPROMDIRECTORY_H

#include "eeprom_status.h"
#include "eeprom_directory.h"
#include <stdint.h>

/**
 * @brief Initializes the EEPROM directory with given partitions.
 *
 * @param directory Pointer to the EEPROM directory structure.
 * @param partitions Array of partition configurations.
 * @param num_partitions Number of partitions in the array.
 * @return eeprom_status_t EEPROM_OK on success, error code otherwise.
 */
eeprom_status_t eepromInit(eeprom_directory_t *directory,
			   const struct partition_cfg *partitions,
			   size_t num_partitions);

/**
 * @brief Reads data from EEPROM using a key.
 *
 * @param directory Pointer to the EEPROM directory.
 * @param key Key representing the EEPROM partition.
 * @param data Pointer to buffer where data will be stored.
 * @param size Size of data to be read.
 * @return eeprom_status_t EEPROM_OK on success, error code otherwise.
 */
eeprom_status_t eeprom_read_data_key(eeprom_directory_t *directory,
				     const char *key, void *data,
				     uint16_t size);

/**
 * @brief Reads data from EEPROM using a memory address.
 *
 * @param directory Pointer to the EEPROM directory.
 * @param address EEPROM memory address to read from.
 * @param data Pointer to buffer where data will be stored.
 * @param size Size of data to be read.
 * @return eeprom_status_t EEPROM_OK on success, error code otherwise.
 */
eeprom_status_t eeprom_read_data_address(eeprom_directory_t *directory,
					 const char *key, uint16_t address,
					 void *data, uint16_t size);

/**
 * @brief Writes data to EEPROM using a key.
 *
 * @param directory Pointer to the EEPROM directory.
 * @param key Key representing the EEPROM partition.
 * @param data Pointer to the data to be written.
 * @param size Size of data to be written.
 * @return eeprom_status_t EEPROM_OK on success, error code otherwise.
 */
eeprom_status_t eeprom_write_data_key(eeprom_directory_t *directory,
				      const char *key, void *data,
				      uint16_t size);

/**
 * @brief Writes data to EEPROM using a memory address.
 *
 * @param directory Pointer to the EEPROM directory.
 * @param address EEPROM memory address to write to.
 * @param data Pointer to the data to be written.
 * @param size Size of data to be written.
 * @return eeprom_status_t EEPROM_OK on success, error code otherwise.
 */
eeprom_status_t eeprom_write_data_address(eeprom_directory_t *directory,
					  const char *key, uint16_t address,
					  void *data, uint16_t size);

/**
 * @brief Logs a fault code into the EEPROM faults partition.
 *
 * @param directory Pointer to the EEPROM directory.
 * @param fault_code The fault code to log.
 * @return eeprom_status_t EEPROM_OK on success, error code otherwise.
 */
eeprom_status_t log_fault(eeprom_directory_t *directory, uint32_t fault_code);

/**
 * @brief Retrieves the latest stored faults from the EEPROM faults partition.
 *
 * @param directory Pointer to the EEPROM directory.
 * @param faults Pointer to an array where fault codes will be stored.
 * @param n Number of latest faults to retrieve.
 * @param valid_count Pointer where the number of valid faults will be returned.
 * @return eeprom_status_t EEPROM_OK on success, error code otherwise.
 */
eeprom_status_t get_faults(eeprom_directory_t *directory, uint32_t *faults,
			   uint16_t n, uint16_t *valid_count);

#endif // EEPROMDIRECTORY_H