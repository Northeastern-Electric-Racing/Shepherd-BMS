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
	HAL_GPIO_WritePin(GPIOA, Fault_Output_Pin, !fault_state);
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

void toggle_debug_led()
{
	HAL_GPIO_TogglePin(Debug_LED_GPIO_Port, Debug_LED_Pin);
}

void set_poll_led(int mode)
{
	HAL_GPIO_WritePin(Debug_LEDB11_GPIO_Port, Debug_LEDB11_Pin, mode);
}

bool read_shutdown()
{
	// If the pin is high, the shutdown circuit is closed. So, return false.
	// If the pin is low, the shutdown circuit is open. So, return true.
	return !HAL_GPIO_ReadPin(SHUTDOWN_3V3_GPIO_Port, SHUTDOWN_3V3_Pin);
}