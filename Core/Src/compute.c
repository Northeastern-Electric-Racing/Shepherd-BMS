#include "compute.h"
#include "c_utils.h"
#include "main.h"
#include "stm32f405xx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include "bmsConfig.h"
#include "can_handler.h"

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

extern can_msg_t bms_can_msgs[RL_MSG_COUNT];

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

int compute_send_charging_message(uint16_t voltage_to_set,
				  uint16_t current_to_set, acc_data_t *bms_data)
{
	struct __attribute__((__packed__)) {
		uint16_t charger_voltage; // Note the charger voltage sent over should be
			// 10*desired voltage
		uint16_t charger_current; // Note the charge current sent over should be
			// 10*desired current
		uint8_t charger_control;
		uint8_t reserved_1;
		uint16_t reserved_23;
	} charger_msg_data;

	charger_msg_data.charger_voltage = voltage_to_set * 10;
	charger_msg_data.charger_current = current_to_set * 10;

	if (is_charging_enabled) {
		charger_msg_data.charger_control = 0x00; // 0：Start charging.
	} else {
		charger_msg_data.charger_control =
			0xFF; // 1：battery protection, stop charging
	}

	charger_msg_data.reserved_1 = 0x00;
	charger_msg_data.reserved_23 = 0x0000;

	can_msg_t charger_msg;
	charger_msg.id = 0x1806E5F4;
	charger_msg.len = 8;
	memcpy(charger_msg.data, &charger_msg_data, sizeof(charger_msg_data));

	uint8_t temp = charger_msg.data[0];
	charger_msg.data[0] = charger_msg.data[1];
	charger_msg.data[1] = temp;
	temp = charger_msg.data[2];
	charger_msg.data[2] = charger_msg.data[3];
	charger_msg.data[3] = temp;

#ifdef CHARGING_ENABLED
	HAL_StatusTypeDef res = can_send_extended_msg(&can2, &charger_msg);
	if (res != HAL_OK) {
		printf("CAN ERROR CODE %X", res);
	}
#endif

	return 0;
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
	// TODO work with charger fw on this
	HAL_GPIO_WritePin(GPIOA, Fault_Output_Pin, !fault_state);
	// if (true) digitalWrite(CHARGE_SAFETY_RELAY, 1);
}

int16_t compute_get_pack_current()
{
	// static const float GAIN = 5.00; // mV/A
	// static const float OFFSET = 0.0; // mV
	// static const uint8_t num_samples = 10;
	// static int16_t current_accumulator = 0.0; // A

	// /* starting equation : Vout = Vref + Voffset  + (Gain * Ip) */
	// float ref_voltage = read_ref_voltage();
	// float vout = read_vout();

	// ref_voltage *= 1000;// convert to mV
	// vout *= 1000;

	// int16_t current = (vout - ref_voltage - OFFSET) / (GAIN); // convert to V

	// /* Low Pass Filter of Current*/
	// current = ((current_accumulator * (num_samples - 1)) + current) /
	// num_samples; current_accumulator = current;

	// return current;

	static const float CURRENT_LOWCHANNEL_MAX = 75.0; // Amps
	static const float CURRENT_LOWCHANNEL_MIN = -75.0; // Amps
	// static const float CURRENT_SUPPLY_VOLTAGE = 5.038;
	static const float CURRENT_ADC_RESOLUTION = 5.0 / MAX_ADC_RESOLUTION;

	static const float CURRENT_LOWCHANNEL_OFFSET =
		2.500; // Calibrated with current = 0A
	static const float CURRENT_HIGHCHANNEL_OFFSET =
		2.500; // Calibrated with current = 0A
	/*
	static const float HIGHCHANNEL_GAIN =
		1 / 0.0041; // Calibrated with  current = 5A, 10A, 20A
	static const float LOWCHANNEL_GAIN = 1 / 0.0267;
	*/

	uint32_t raw_high_current;
	uint32_t raw_low_current;
	uint32_t ref_5V;

	memcpy(&raw_high_current, &raw_high_current_buf,
	       sizeof(raw_high_current_buf));

	memcpy(&raw_low_current, &channel_1_buf[0],
	       sizeof(channel_1_buf[0])); //From the rank of ADC_CHANNEL_15

	memcpy(&ref_5V, &channel_1_buf[1],
	       sizeof(channel_1_buf[1])); //From the rank of ADC_CHANNEL_9

	int16_t ref_voltage_raw =
		(int16_t)(1000.0f * ((float)ref_5V * CURRENT_ADC_RESOLUTION));

	int16_t high_current_voltage_raw =
		(int16_t)(1000.0f *
			  ((float)raw_high_current * CURRENT_ADC_RESOLUTION));

	high_current_voltage_raw =
		(int16_t)(5000.0f * high_current_voltage_raw /
			  (float)ref_voltage_raw);

	int16_t high_current = (high_current_voltage_raw -
				(1000 * CURRENT_HIGHCHANNEL_OFFSET)) *
			       (1 / 4.0f); //* (HIGHCHANNEL_GAIN/100.0f))/1000;

	int16_t low_current_voltage_raw =
		(int16_t)(1000.0f *
			  ((float)raw_low_current * CURRENT_ADC_RESOLUTION));
	low_current_voltage_raw = (int16_t)(5000.0f * low_current_voltage_raw /
					    (float)ref_voltage_raw);

	int16_t low_current = (float)(low_current_voltage_raw -
				      (1000 * CURRENT_LOWCHANNEL_OFFSET)) *
			      (1 / 26.7); //* (LOWCHANNEL_GAIN/100.0f))/1000;

	// If the current is scoped within the range of the low channel, use the low
	// channel

	if ((low_current < CURRENT_LOWCHANNEL_MAX - 5.0 && low_current >= 0) ||
	    (low_current > CURRENT_LOWCHANNEL_MIN + 5.0 && low_current < 0)) {
		// printf("\rLow Current: %d\n", -low_current);
		return -low_current;
	}

	return -high_current;
}

void compute_send_mc_discharge_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		uint16_t max_discharge;
	} discharge_data;

	/* scale to A * 10 */
	discharge_data.max_discharge = 10 * bmsdata->discharge_limit;

	/* convert to big endian */
	endian_swap(&discharge_data.max_discharge,
		    sizeof(discharge_data.max_discharge));

	memcpy(bms_can_msgs[DISCHARGE].data, &discharge_data,
	       sizeof(discharge_data));

	queue_can_msg(bms_can_msgs[DISCHARGE]);
}

void compute_send_mc_charge_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		int16_t max_charge;
	} charge_data;

	/* scale to A * 10 */
	charge_data.max_charge = -10 * bmsdata->charge_limit;

	/* convert to big endian */
	endian_swap(&charge_data.max_charge, sizeof(charge_data.max_charge));

	memcpy(bms_can_msgs[CHARGE].data, &charge_data, sizeof(charge_data));

	queue_can_msg(bms_can_msgs[CHARGE]);
}

void compute_send_acc_status_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		uint16_t packVolt;
		uint16_t pack_current;
		uint16_t pack_ah;
		uint8_t pack_soc;
		uint8_t pack_health;
	} acc_status_msg_data;

	acc_status_msg_data.packVolt = bmsdata->pack_voltage;
	acc_status_msg_data.pack_current =
		(uint16_t)(bmsdata->pack_current); // convert with 2s complement
	acc_status_msg_data.pack_ah = 0;
	acc_status_msg_data.pack_soc = bmsdata->soc;
	acc_status_msg_data.pack_health = 0;

	/* convert to big endian */
	endian_swap(&acc_status_msg_data.packVolt,
		    sizeof(acc_status_msg_data.packVolt));
	endian_swap(&acc_status_msg_data.pack_current,
		    sizeof(acc_status_msg_data.pack_current));
	endian_swap(&acc_status_msg_data.pack_ah,
		    sizeof(acc_status_msg_data.pack_ah));

	memcpy(bms_can_msgs[ACC_STATUS].data, &acc_status_msg_data,
	       sizeof(acc_status_msg_data));

	queue_can_msg(bms_can_msgs[ACC_STATUS]);
}

void compute_send_fault_status_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		uint32_t fault_crit;
		uint32_t fault_noncrit;
	} fault_status_msg_data;

	/* convert to big endian */
	endian_swap(&fault_status_msg_data.fault_crit,
		    sizeof(fault_status_msg_data.fault_crit));
	endian_swap(&fault_status_msg_data.fault_noncrit,
		    sizeof(fault_status_msg_data.fault_noncrit));

	fault_status_msg_data.fault_crit = bmsdata->fault_code_crit;
	fault_status_msg_data.fault_noncrit = bmsdata->fault_code_noncrit;

	memcpy(bms_can_msgs[FAULT_STATUS].data, &fault_status_msg_data,
	       sizeof(fault_status_msg_data));

	queue_can_msg(bms_can_msgs[FAULT_STATUS]);
}

void compute_send_bms_status_message(acc_data_t *bmsdata, int bms_state,
				     bool balance)
{
	struct __attribute__((__packed__)) {
		uint8_t state;
		int8_t temp_avg;
		uint8_t temp_internal;
		uint8_t balance;
	} bms_status_msg_data;

	bms_status_msg_data.temp_avg = (int8_t)(bmsdata->avg_temp);
	bms_status_msg_data.state = (uint8_t)(bms_state);
	bms_status_msg_data.temp_internal = (uint8_t)(0);
	bms_status_msg_data.balance =
		(uint8_t)(balance); // segment_is_balancing()

	memcpy(bms_can_msgs[BMS_STATUS].data, &bms_status_msg_data,
	       sizeof(bms_status_msg_data));

	queue_can_msg(bms_can_msgs[BMS_STATUS]);
}

void compute_send_shutdown_ctrl_message(uint8_t mpe_state)
{
	struct __attribute__((__packed__)) {
		uint8_t mpeState;
	} shutdown_control_msg_data;

	shutdown_control_msg_data.mpeState = mpe_state;

	memcpy(bms_can_msgs[SHUTDOWN_CTRL].data, &shutdown_control_msg_data,
	       sizeof(shutdown_control_msg_data));

	queue_can_msg(bms_can_msgs[SHUTDOWN_CTRL]);
}

void compute_send_cell_data_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		uint16_t high_cell_voltage;
		uint8_t high_cell_id;
		uint16_t low_cell_voltage;
		uint8_t low_cell_id;
		uint16_t volt_avg;
	} cell_data_msg_data;

	cell_data_msg_data.high_cell_voltage = bmsdata->max_voltage.val;
	cell_data_msg_data.high_cell_id =
		(bmsdata->max_voltage.chipIndex << 4) |
		bmsdata->max_voltage.cellNum;
	cell_data_msg_data.low_cell_voltage = bmsdata->min_voltage.val;
	cell_data_msg_data.low_cell_id = (bmsdata->min_voltage.chipIndex << 4) |
					 bmsdata->min_voltage.cellNum;
	cell_data_msg_data.volt_avg = bmsdata->avg_voltage;

	/* convert to big endian */
	endian_swap(&cell_data_msg_data.high_cell_voltage,
		    sizeof(cell_data_msg_data.high_cell_voltage));
	endian_swap(&cell_data_msg_data.low_cell_voltage,
		    sizeof(cell_data_msg_data.low_cell_voltage));
	endian_swap(&cell_data_msg_data.volt_avg,
		    sizeof(cell_data_msg_data.volt_avg));

	memcpy(bms_can_msgs[CELL_DATA].data, &cell_data_msg_data,
	       sizeof(cell_data_msg_data));

	queue_can_msg(bms_can_msgs[CELL_DATA]);
}

void compute_send_cell_voltage_message(uint8_t cell_id,
				       uint16_t instant_voltage,
				       uint16_t internal_Res, uint8_t shunted,
				       uint16_t open_voltage)
{
	struct __attribute__((__packed__)) {
		uint8_t cellID;
		uint16_t instantVoltage;
		uint16_t internalResistance;
		uint8_t shunted;
		uint16_t openVoltage;
	} cell_voltage_msg_data;

	cell_voltage_msg_data.cellID = cell_id;
	cell_voltage_msg_data.instantVoltage = instant_voltage;
	cell_voltage_msg_data.internalResistance = internal_Res;
	cell_voltage_msg_data.shunted = shunted;
	cell_voltage_msg_data.openVoltage = open_voltage;

	/* convert to big endian */
	endian_swap(&cell_voltage_msg_data.instantVoltage,
		    sizeof(cell_voltage_msg_data.instantVoltage));
	endian_swap(&cell_voltage_msg_data.internalResistance,
		    sizeof(cell_voltage_msg_data.internalResistance));
	endian_swap(&cell_voltage_msg_data.openVoltage,
		    sizeof(cell_voltage_msg_data.openVoltage));

	memcpy(bms_can_msgs[CELL_VOLTAGE].data, &cell_voltage_msg_data,
	       sizeof(cell_voltage_msg_data));

	queue_can_msg(bms_can_msgs[CELL_VOLTAGE]);
}

void compute_send_current_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		uint16_t dcl;
		int16_t ccl;
		uint16_t pack_curr;
	} current_status_msg_data;

	current_status_msg_data.dcl = bmsdata->discharge_limit;
	current_status_msg_data.ccl = -1 * bmsdata->charge_limit;
	current_status_msg_data.pack_curr = bmsdata->pack_current;

	/* convert to big endian */
	endian_swap(&current_status_msg_data.dcl,
		    sizeof(current_status_msg_data.dcl));
	endian_swap(&current_status_msg_data.ccl,
		    sizeof(current_status_msg_data.ccl));
	endian_swap(&current_status_msg_data.pack_curr,
		    sizeof(current_status_msg_data.pack_curr));

	memcpy(bms_can_msgs[CURRENT].data, &current_status_msg_data,
	       sizeof(current_status_msg_data));

	queue_can_msg(bms_can_msgs[CURRENT]);
}

void compute_send_cell_temp_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		uint16_t max_cell_temp;
		uint8_t max_cell_id;
		uint16_t min_cell_temp;
		uint8_t min_cell_id;
		uint16_t average_temp;
	} cell_temp_msg_data;

	cell_temp_msg_data.max_cell_temp = bmsdata->max_temp.val;
	cell_temp_msg_data.max_cell_id = (bmsdata->max_temp.chipIndex << 4) |
					 (bmsdata->max_temp.cellNum - 17);
	cell_temp_msg_data.min_cell_temp = bmsdata->min_temp.val;
	cell_temp_msg_data.min_cell_id = (bmsdata->min_temp.chipIndex << 4) |
					 (bmsdata->min_temp.cellNum - 17);
	cell_temp_msg_data.average_temp = bmsdata->avg_temp;

	/* convert to big endian */
	endian_swap(&cell_temp_msg_data.max_cell_temp,
		    sizeof(cell_temp_msg_data.max_cell_temp));
	endian_swap(&cell_temp_msg_data.min_cell_temp,
		    sizeof(cell_temp_msg_data.min_cell_temp));
	endian_swap(&cell_temp_msg_data.average_temp,
		    sizeof(cell_temp_msg_data.average_temp));

	memcpy(bms_can_msgs[CELL_TEMP].data, &cell_temp_msg_data,
	       sizeof(cell_temp_msg_data));

	queue_can_msg(bms_can_msgs[CELL_TEMP]);
}

void compute_send_segment_temp_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		int8_t segment1_average_temp;
		int8_t segment2_average_temp;
		int8_t segment3_average_temp;
		int8_t segment4_average_temp;
		int8_t segment5_average_temp;
		int8_t segment6_average_temp;

	} segment_temp_msg_data;

	segment_temp_msg_data.segment1_average_temp =
		bmsdata->segment_average_temps[0];
	segment_temp_msg_data.segment2_average_temp =
		bmsdata->segment_average_temps[1];
	segment_temp_msg_data.segment3_average_temp =
		bmsdata->segment_average_temps[2];
	segment_temp_msg_data.segment4_average_temp =
		bmsdata->segment_average_temps[3];
	segment_temp_msg_data.segment5_average_temp =
		bmsdata->segment_average_temps[4];
	segment_temp_msg_data.segment6_average_temp =
		bmsdata->segment_average_temps[5];

	memcpy(bms_can_msgs[SEGMENT_TEMP].data, &segment_temp_msg_data,
	       sizeof(segment_temp_msg_data));

	queue_can_msg(bms_can_msgs[SEGMENT_TEMP]);
}

void compute_send_fault_message(uint8_t status, int16_t curr, int16_t in_dcl)
{
	struct __attribute__((__packed__)) {
		uint8_t status;
		int16_t pack_curr;
		int16_t dcl;
	} fault_msg_data;

	fault_msg_data.status = status;
	fault_msg_data.pack_curr = curr;
	fault_msg_data.dcl = in_dcl;

	endian_swap(&fault_msg_data.pack_curr,
		    sizeof(fault_msg_data.pack_curr));
	endian_swap(&fault_msg_data.dcl, sizeof(fault_msg_data.dcl));

	memcpy(bms_can_msgs[FAULT].data, &fault_msg_data,
	       sizeof(fault_msg_data));

	queue_can_msg(bms_can_msgs[FAULT]);
}

void compute_send_voltage_noise_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		uint8_t seg1_noise;
		uint8_t seg2_noise;
		uint8_t seg3_noise;
		uint8_t seg4_noise;
		uint8_t seg5_noise;
		uint8_t seg6_noise;
	} voltage_noise_msg_data;

	voltage_noise_msg_data.seg1_noise =
		bmsdata->segment_noise_percentage[0];
	voltage_noise_msg_data.seg2_noise =
		bmsdata->segment_noise_percentage[1];
	voltage_noise_msg_data.seg3_noise =
		bmsdata->segment_noise_percentage[2];
	voltage_noise_msg_data.seg4_noise =
		bmsdata->segment_noise_percentage[3];
	voltage_noise_msg_data.seg5_noise =
		bmsdata->segment_noise_percentage[4];
	voltage_noise_msg_data.seg6_noise =
		bmsdata->segment_noise_percentage[5];

	memcpy(bms_can_msgs[NOISE].data, &voltage_noise_msg_data,
	       sizeof(voltage_noise_msg_data));

	queue_can_msg(bms_can_msgs[NOISE]);
}

void compute_send_debug_message(uint8_t debug0, uint8_t debug1, uint16_t debug2,
				uint32_t debug3)
{
	struct __attribute__((__packed__)) {
		uint8_t debug0;
		uint8_t debug1;
		uint16_t debug2;
		uint32_t debug3;
	} debug_msg_data;

	debug_msg_data.debug0 = debug0;
	debug_msg_data.debug1 = debug1;
	debug_msg_data.debug2 = debug2;
	debug_msg_data.debug3 = debug3;

	endian_swap(&debug_msg_data.debug2, sizeof(debug_msg_data.debug2));
	endian_swap(&debug_msg_data.debug3, sizeof(debug_msg_data.debug3));

	memcpy(bms_can_msgs[DEBUG].data, &debug_msg_data, 8);

	queue_can_msg(bms_can_msgs[DEBUG]);
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
