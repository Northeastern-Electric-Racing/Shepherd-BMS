#include "eepromdirectory.h"
#include "stm32xx_hal.h"
#include "m24c32_eeprom_directory.h"
#include "m24c32.h"

#define M24C32_DEVICE_ADDR (0x50U << 1)

extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Writes data to the EEPROM over I2C.
 *
 * @param address EEPROM memory address to write to.
 * @param data Pointer to the data to be written.
 * @param size Number of bytes to write.
 * @return EEPROM_OK on success, EEPROM_ERROR_COMMS on communication failure.
 */
static eeprom_status_t I2C_write(uint16_t address, uint8_t *data, uint16_t size)
{
	uint8_t buffer[2 + size];
	buffer[0] = (address >> 8) & 0xFF;
	buffer[1] = address & 0xFF;
	memcpy(&buffer[2], data, size);

	if (HAL_I2C_Master_Transmit(&hi2c1, M24C32_DEVICE_ADDR, buffer,
				    size + 2, 1000) == HAL_OK) {
		return EEPROM_OK;
	}
	return EEPROM_ERROR_COMMS;
}

/**
 * @brief Reads data from the EEPROM over I2C.
 *
 * @param address EEPROM memory address to read from.
 * @param data Pointer to the buffer to store the read data.
 * @param size Number of bytes to read.
 * @return EEPROM_OK on success, EEPROM_ERROR_COMMS on communication failure.
 */
static eeprom_status_t I2C_read(uint16_t address, uint8_t *data, uint16_t size)
{
	uint8_t addr[2] = { (address >> 8) & 0xFF, address & 0xFF };

	if (HAL_I2C_Master_Transmit(&hi2c1, M24C32_DEVICE_ADDR, addr, 2,
				    1000) != HAL_OK) {
		return EEPROM_ERROR_COMMS;
	}

	if (HAL_I2C_Master_Receive(&hi2c1, M24C32_DEVICE_ADDR, data, size,
				   1000) == HAL_OK) {
		return EEPROM_OK;
	}
	return EEPROM_ERROR_COMMS;
}

m24c32_t eeprom_dev = { .write = I2C_write, .read = I2C_read };

/**
 * @brief Calculates a XOR checksum for a 32-bit fault code.
 *
 * @param fault_code The fault code to calculate checksum for.
 * @return 8-bit checksum result.
 */
static uint8_t calculate_checksum(uint32_t fault_code)
{
	uint8_t *bytes = (uint8_t *)&fault_code;
	uint8_t sum = 0;
	for (int i = 0; i < 4; i++)
		sum ^= bytes[i]; // XOR checksum
	return sum;
}

eeprom_status_t eepromInit(eeprom_directory_t *directory,
			   const struct partition_cfg *partitions,
			   size_t num_partitions)
{
	eeprom_status_t status =
		directory_init(directory, partitions, num_partitions);

	if (status != EEPROM_OK)
		return status;

	return EEPROM_OK;
}

eeprom_status_t eeprom_read_data_key(eeprom_directory_t *directory,
				     const char *key, void *data, uint16_t size)
{
	eeprom_status_t status = m24c32_directory_read(directory, &eeprom_dev,
						       key, data, size, 0);

	if (status != EEPROM_OK)
		return status;

	return EEPROM_OK;
}

eeprom_status_t eeprom_read_data_address(eeprom_directory_t *directory,
					 const char *key, uint16_t address,
					 void *data, uint16_t size)
{
	uint16_t base;
	eeprom_status_t status = eeprom_get_base_address(directory, key, &base);

	if (status != EEPROM_OK)
		return status;

	if (address < base) {
		return EEPROM_ERROR_OUT_OF_BOUNDS;
	}

	status = m24c32_directory_read(directory, &eeprom_dev, key, data, size,
				       address - base);

	if (status != EEPROM_OK)
		return status;

	return EEPROM_OK;
}

eeprom_status_t eeprom_write_data_key(eeprom_directory_t *directory,
				      const char *key, void *data,
				      uint16_t size)
{
	eeprom_status_t status = m24c32_directory_write(
		directory, &eeprom_dev, key, (uint8_t *)data, size, 0);

	if (status != EEPROM_OK)
		return status;

	return EEPROM_OK;
}

eeprom_status_t eeprom_write_data_address(eeprom_directory_t *directory,
					  const char *key, uint16_t address,
					  void *data, uint16_t size)
{
	uint16_t base;
	eeprom_status_t status = eeprom_get_base_address(directory, key, &base);

	if (status != EEPROM_OK)
		return status;

	if (address < base) {
		return EEPROM_ERROR_OUT_OF_BOUNDS;
	}

	status = m24c32_directory_write(directory, &eeprom_dev, key,
					(uint8_t *)data, size, address - base);

	if (status != EEPROM_OK)
		return status;

	return EEPROM_OK;
}

eeprom_status_t log_fault(eeprom_directory_t *directory, uint32_t fault_code)
{
	uint8_t fault_index = 0;
	eeprom_status_t status;

	status = m24c32_directory_read(directory, &eeprom_dev, "FAULTS",
				       &fault_index, 1, 0);
	if (status != EEPROM_OK)
		return status;

	uint16_t partition_size;
	status = eeprom_get_size(directory, "FAULTS", &partition_size);
	if (status != EEPROM_OK)
		return status;

	uint8_t max_faults = (partition_size - 1) / 5;

	if (fault_index >= max_faults)
		fault_index = 0;

	uint16_t relative_offset = 1 + (fault_index * 5);

	uint8_t fault_packet[5];
	memcpy(fault_packet, &fault_code, 4);
	fault_packet[4] = calculate_checksum(fault_code);

	status = m24c32_directory_write(directory, &eeprom_dev, "FAULTS",
					fault_packet, 5, relative_offset);
	if (status != EEPROM_OK)
		return status;

	fault_index = (fault_index + 1) % max_faults;
	return m24c32_directory_write(directory, &eeprom_dev, "FAULTS",
				      &fault_index, 1, 0);
}

eeprom_status_t get_faults(eeprom_directory_t *directory, uint32_t *faults,
			   uint16_t n, uint16_t *valid_count)
{
	uint8_t fault_index = 0;
	eeprom_status_t status;

	status = m24c32_directory_read(directory, &eeprom_dev, "FAULTS",
				       &fault_index, 1, 0);
	if (status != EEPROM_OK)
		return status;

	uint16_t partition_size;
	status = eeprom_get_size(directory, "FAULTS", &partition_size);
	if (status != EEPROM_OK)
		return status;

	uint8_t max_faults = (partition_size - 1) / 5;

	if (n > max_faults)
		return EEPROM_ERROR_OUT_OF_BOUNDS;

	int start_index = (fault_index == 0) ? (max_faults - 1) :
					       (fault_index - 1);

	*valid_count = 0;

	for (int i = 0; i < n; i++) {
		uint8_t fault_packet[5];
		uint16_t relative_offset = 1 + (start_index * 5);

		status = m24c32_directory_read(directory, &eeprom_dev, "FAULTS",
					       fault_packet, 5,
					       relative_offset);
		if (status != EEPROM_OK)
			return status;

		uint32_t fault;
		memcpy(&fault, fault_packet, 4);
		uint8_t checksum = fault_packet[4];

		if (calculate_checksum(fault) == checksum) {
			faults[(*valid_count)++] = fault;
		}

		start_index = (start_index == 0) ? (max_faults - 1) :
						   (start_index - 1);
	}

	return EEPROM_OK;
}