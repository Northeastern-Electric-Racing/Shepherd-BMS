#include "compute.h"

#include <assert.h>

#include "datastructs.h"
#include "main.h"

// the buffer for ADC channel of current sensor
uint32_t channel_1_buf;

// ADC for use with current sensor
extern ADC_HandleTypeDef hadc1;

void compute_init()
{
	assert(!HAL_ADC_Start_DMA(&hadc1, &channel_1_buf,
				  sizeof(channel_1_buf) / sizeof(uint32_t)));
}

void compute_set_fault(bool fault_state)
{
	HAL_GPIO_WritePin(FAULT_MCU_GPIO_Port, FAULT_MCU_Pin, !fault_state);
}

float compute_get_pack_current()
{
	// For LA37S050S05KM Current Sensor

	static const float CURRENT_ADC_RESOLUTION = 3.3 / MAX_ADC_RESOLUTION;
	static const float SENSOR_V_REF = 2.5; // From LA37S050S05KM Data Sheet
	static const float VOLTAGE_DIVIDER = (3.0 / 5.0); // 2k and 3k resistors

	// Get ADC reading
	uint32_t adcValue;
	memcpy(&adcValue, &channel_1_buf,
	       sizeof(channel_1_buf)); // From the rank of ADC_CHANNEL_15

	// Convert ADC reading to volts and amps
	float volts =
		((float)adcValue * CURRENT_ADC_RESOLUTION / VOLTAGE_DIVIDER) -
		SENSOR_V_REF;
	float amps = volts / 0.0125; // Sensativity of 0.0125 Volts per Amp
	return amps;
}

void toggle_debug_led_1()
{
	HAL_GPIO_TogglePin(DEBUG_LED_1_GPIO_Port, DEBUG_LED_2_GPIO_Port);
}

void set_debug_led_2(int mode)
{
	HAL_GPIO_WritePin(DEBUG_LED_2_GPIO_Port, DEBUG_LED_2_Pin, mode);
}

void pet_watchdog()
{
	HAL_GPIO_WritePin(WATCHDOG_GPIO_Port, WATCHDOG_Pin, true);
	HAL_GPIO_WritePin(WATCHDOG_GPIO_Port, WATCHDOG_Pin, false);
}

bool read_shutdown()
{
	// If the pin is high, the shutdown circuit is closed. So, return false.
	// If the pin is low, the shutdown circuit is open. So, return true.
	return !HAL_GPIO_ReadPin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin);
}