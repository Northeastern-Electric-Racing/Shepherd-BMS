#include "compute.h"

#include <assert.h>

#include "datastructs.h"
#include "main.h"

#define REF_CHANNEL  0
#define VOUT_CHANNEL 1

// #define CHARGING_ENABLED

uint8_t fan_speed;
uint32_t channel_1_buf;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern ADC_HandleTypeDef hadc1;

TIM_OC_InitTypeDef pwm_config;
ADC_ChannelConfTypeDef adc_config;

const uint32_t fan_channels[6] = { TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_4,
				   TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1 };

uint8_t compute_init(acc_data_t *bmsdata)
{
	pwm_config.OCMode = TIM_OCMODE_PWM1;
	pwm_config.Pulse = 0;
	pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm_config,
				      fan_channels[FAN1]) != HAL_OK)
		return -1;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &pwm_config,
				      fan_channels[FAN2]) != HAL_OK)
		return -1;

	// HAL_TIM_PWM_Start(&htim1, fan_channels[FAN1]);
	// HAL_TIM_PWM_Start(&htim1, fan_channels[FAN2]);
	// HAL_TIM_PWM_Start(&htim8, fan_channels[FAN3]);
	// HAL_TIM_PWM_Start(&htim8, fan_channels[FAN4]);
	// HAL_TIM_PWM_Start(&htim8, fan_channels[FAN5]);
	// HAL_TIM_PWM_Start(&htim8, fan_channels[FAN6]);
	bmsdata->is_charger_connected = false;

	//DMA for first ADC channel -- raw_low_current and ref_5V
	assert(!HAL_ADC_Start_DMA(&hadc1, channel_1_buf,
				  sizeof(channel_1_buf) / sizeof(uint32_t)));

	return 0;
}

// TODO add this back
//  void compute_charger_callback(const CAN_message_t& msg)
//  {
//  	return;
//  }

uint8_t compute_set_fan_speed(TIM_HandleTypeDef *pwmhandle,
			      fan_select_t fan_select, uint8_t duty_cycle)
{
	if (!pwmhandle)
		return -1;
	if (fan_select >= FANMAX)
		return -1;
	if (duty_cycle > 100)
		return -1;

	uint32_t CCR_value = 0;
	uint32_t channel = fan_channels[fan_select];

	CCR_value = (pwmhandle->Instance->ARR * duty_cycle) / 100;
	__HAL_TIM_SET_COMPARE(pwmhandle, channel, CCR_value);

	return 0;
}

void compute_set_fault(int fault_state)
{
	HAL_GPIO_WritePin(GPIOA, Fault_Output_Pin, !fault_state);
	// if (true) digitalWrite(CHARGE_SAFETY_RELAY, 1);
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