#include "compute.h"

#include <assert.h>

#include "datastructs.h"
#include "main.h"

#define REF_CHANNEL  0
#define VOUT_CHANNEL 1

// #define CHARGING_ENABLED

uint8_t fan_speed;
bool is_charging_enabled;
enum { CHARGE_ENABLED, CHARGE_DISABLED };
uint32_t channel_1_buf[2];
uint32_t raw_high_current_buf;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

TIM_OC_InitTypeDef pwm_config;
ADC_ChannelConfTypeDef adc_config;

const uint32_t fan_channels[6] = { TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_4,
				   TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1 };

uint32_t adc_values[2] = { 0 };

/* private function defintions */
float read_ref_voltage();
float read_vout();
void change_adc1_channel(uint8_t channel);

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

	//DMA for second ADC channel -- raw_high_current
	assert(!HAL_ADC_Start_DMA(&hadc2, &raw_high_current_buf,
				  sizeof(raw_high_current_buf) /
					  sizeof(uint32_t)));

	return 0;
}

void compute_enable_charging(bool enable_charging)
{
	is_charging_enabled = enable_charging;
}

bool compute_charger_connected()
{
#ifdef CHARGING
	return true;
#endif
	//TODO need to set up CAN msg that actually toggles this bool
	return false; //bmsdata->is_charger_connected;
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
	static const float CURRENT_ADC_RESOLUTION =
		(5.0 / MAX_ADC_RESOLUTION) * (5.0 / 3.0);
	static const float SENSOR_V_REF = 2.5;

	// Get ADC reading
	uint32_t adcValue;
	memcpy(&adcValue, &channel_1_buf[0],
	       sizeof(channel_1_buf[0])); // From the rank of ADC_CHANNEL_15

	// Convert ADC reading to volts and amps
	float volts = ((float)adcValue * CURRENT_ADC_RESOLUTION) - SENSOR_V_REF;
	float amps = volts / 0.0125; // Sensativity of 0.0125 Volts per Amp

	return amps;
}

void change_adc1_channel(uint8_t channel)
{
	ADC_ChannelConfTypeDef sConfig = { 0 };

	if (channel == REF_CHANNEL)
		sConfig.Channel = ADC_CHANNEL_9;
	else if (channel == VOUT_CHANNEL)
		sConfig.Channel = ADC_CHANNEL_15;

	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}