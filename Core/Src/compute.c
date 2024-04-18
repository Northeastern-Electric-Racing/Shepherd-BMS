#include "compute.h"
#include "can_handler.h"
#include "can.h"
#include "main.h"
#include <assert.h>
#include "stm32f405xx.h"
#include <string.h>

#define MAX_CAN1_STORAGE 10
#define MAX_CAN2_STORAGE 10

uint8_t fan_speed;
bool is_charging_enabled;
enum { CHARGE_ENABLED, CHARGE_DISABLED };

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

TIM_OC_InitTypeDef pwm_config;
ADC_ChannelConfTypeDef adc_config;

const uint32_t fan_channels[6] = {TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_4, TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1};

can_t can1; // main can bus, used by most peripherals
can_t can2; // p2p can bus with charger

uint32_t adc_values[2] = {0};

/* private function defintions */
uint8_t calc_charger_led_state();
float read_ref_voltage();
float read_vout();

uint8_t compute_init()
{
	// TODO throw all of these objects into a compute struct
	can1.hcan = &hcan1;
	can1.id_list = can1_id_list;
	can1.id_list_len = sizeof(can1_id_list) / sizeof(can1_id_list[0]);
	//can1.callback = can_receive_callback;
	can1_rx_queue = ringbuffer_create(MAX_CAN1_STORAGE, sizeof(can_msg_t));
	can_init(&can1);

	can2.hcan = &hcan2;
	can2.id_list = can2_id_list;
	can2.id_list_len = sizeof(can2_id_list) / sizeof(can2_id_list[0]);
	//can2.callback = can_receive_callback;
	can2_rx_queue = ringbuffer_create(MAX_CAN2_STORAGE, sizeof(can_msg_t));
	can_init(&can2);

	pwm_config.OCMode = TIM_OCMODE_PWM1;
	pwm_config.Pulse = 0;
	pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm_config, fan_channels[FAN1]) != HAL_OK) return -1;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &pwm_config, fan_channels[FAN2]) != HAL_OK) return -1;

	HAL_TIM_PWM_Start(&htim1, fan_channels[FAN1]);
	HAL_TIM_PWM_Start(&htim1, fan_channels[FAN2]);
	HAL_TIM_PWM_Start(&htim8, fan_channels[FAN3]);
	HAL_TIM_PWM_Start(&htim8, fan_channels[FAN4]);
	HAL_TIM_PWM_Start(&htim8, fan_channels[FAN5]);
	HAL_TIM_PWM_Start(&htim8, fan_channels[FAN6]);

	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_Start_DMA(&hadc1, adc_values, 2);

	return 0;

}

void compute_enable_charging(bool enable_charging)
{
	is_charging_enabled = enable_charging;
}

int compute_send_charging_message(uint16_t voltage_to_set, acc_data_t* bms_data)
{
	struct __attribute__((packed)) {
		uint16_t charger_voltage; // Note the charger voltage sent over should be 10*desired voltage
		uint16_t charger_current; // Note the charge current sent over should be 10*desired current
        uint8_t charger_control;
        uint8_t reserved_1;
        uint16_t reserved_23;
	} charger_msg_data;

	uint16_t current_to_set = bms_data->charge_limit;
    if (current_to_set > 10) {
		current_to_set = 10;
	}

	charger_msg_data.charger_voltage = voltage_to_set * 10;
	charger_msg_data.charger_current = current_to_set * 10;

    if (is_charging_enabled) {
        charger_msg_data.charger_control = 0x00;  //0：Start charging.
    } else {
        charger_msg_data.charger_control = 0xFF;  // 1：battery protection, stop charging
    }

    charger_msg_data.reserved_1 = 0x00;
    charger_msg_data.reserved_23 = 0x0000;

	can_msg_t charger_msg;
	charger_msg.id = 0x1806E5F4;
	charger_msg.len = 8;
	memcpy(charger_msg.data, &charger_msg_data, sizeof(charger_msg_data));
	can_send_msg(&can2, &charger_msg);

	return 0;
}

bool compute_charger_connected()
{
	//TODO need to set up CAN msg that actually toggles this bool
	return bmsdata->is_charger_connected;
}

//TODO add this back
// void compute_charger_callback(const CAN_message_t& msg)
// {
// 	return;
// }

uint8_t compute_set_fan_speed(TIM_HandleTypeDef* pwmhandle, fan_select_t fan_select, uint8_t duty_cycle)
{
	if (!pwmhandle) return -1;
	if (fan_select >= FANMAX) return -1;
	if (duty_cycle > 100) return -1;

	uint32_t CCR_value = 0;
	uint32_t channel = fan_channels[fan_select];

	CCR_value = (pwmhandle->Instance->ARR * duty_cycle) / 100;
	__HAL_TIM_SET_COMPARE(pwmhandle, channel, CCR_value); 
	
	return 0;
}

void compute_set_fault(int fault_state)
{
	//TODO work with charger fw on this
	HAL_GPIO_WritePin(GPIOA, Fault_Output_Pin, !fault_state);
	 //if (true) digitalWrite(CHARGE_SAFETY_RELAY, 1);
}

int16_t compute_get_pack_current()
{
	static const float GAIN = 6.250; // mV/A
	static const OFFSET = 0.0; // mV


	/* starting equation : Vout = Vref + Voffset  + (Gain * Ip) */
	//float ref_voltage = -1;
	//float vout = -1;

	// if (!HAL_DMA_PollForTransfer(&hdma_adc1, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY))
	// {
	// 	ref_voltage = 2.5;//adc_values[1] * 2.5 / MAX_ADC_RESOLUTION;
	// 	vout = adc_values[0] * 3.3 / MAX_ADC_RESOLUTION;
	// }

	
	// remove once DMA verified working, along with functions themselves 

	float ref_voltage = read_ref_voltage();
	float vout = read_vout();
	vout *= 1000; // convert to mV

	//if (ref_voltage == -1 || vout == -1) return -1;

	int16_t current = (vout - ref_voltage - OFFSET) / (GAIN / 1000); // convert to V

	return vout;

	/* TEMP keep last years math until above is verified */

	// static const float CURRENT_LOWCHANNEL_MAX = 75.0;  // Amps
	// static const float CURRENT_LOWCHANNEL_MIN = -75.0; // Amps
	// // static const float CURRENT_SUPPLY_VOLTAGE = 5.038;
	// static const float CURRENT_ADC_RESOLUTION = 5.0 / MAX_ADC_RESOLUTION;

	// static const float CURRENT_LOWCHANNEL_OFFSET  = 2.517; // Calibrated with current = 0A
	// static const float CURRENT_HIGHCHANNEL_OFFSET = 2.520; // Calibrated with current = 0A

	// static const float HIGHCHANNEL_GAIN = 1 / 0.004; // Calibrated with  current = 5A, 10A, 20A
	// static const float LOWCHANNEL_GAIN	= 1 / 0.0267;

	// static const float REF5V_DIV  = 19.02 / (19.08 + 19.02); // Resistive divider in kOhm
	// static const float REF5V_CONV = 1 / REF5V_DIV; // Converting from reading to real value

	// //TODO ADD BACK THE COMMENTED OUT ANALOG READS
	// float ref_5V = /*analogRead(MEAS_5VREF_PIN) * */(3.3 / MAX_ADC_RESOLUTION) * REF5V_CONV;
	// int16_t high_current
	// 	= 10
	// 	 /* * (((5 / ref_5V) * /analogRead(CURRENT_SENSOR_PIN_L) * CURRENT_ADC_RESOLUTION))
	// 		 - CURRENT_HIGHCHANNEL_OFFSET) */
	// 	  * HIGHCHANNEL_GAIN; // Channel has a large range with low resolution
	// int16_t low_current
	// 	= 10
	// 	  /* * (((5 / ref_5V) * (analogRead(CURRENT_SENSOR_PIN_H) * CURRENT_ADC_RESOLUTION))
	// 		 - CURRENT_LOWCHANNEL_OFFSET) */
	// 	  * LOWCHANNEL_GAIN; // Channel has a small range with high resolution

	// // Serial.print("High: ");
	// // Serial.println(-high_current);
	// // Serial.print("Low: ");
	// // Serial.println(-low_current);
	// // Serial.print("5V: ");
	// // Serial.println(ref_5V);

	// // If the current is scoped within the range of the low channel, use the low channel
	// if (low_current < CURRENT_LOWCHANNEL_MAX - 5.0 || low_current > CURRENT_LOWCHANNEL_MIN + 5.0) {
	// 	return -low_current;
	// }

	// return -high_current;
}

void compute_send_mc_discharge_message(acc_data_t* bmsdata)
{

	struct __attribute__((packed)) {
		uint16_t max_discharge;
	} discharge_data;

	/* scale to A * 10 */
	discharge_data.max_discharge = 10 * bmsdata->discharge_limit;

	can_msg_t mc_msg = {0};
	mc_msg.id = 0x156;  // 0x0A is the dcl id, 0x22 is the device id set by us
	mc_msg.len = 8;
	memcpy(mc_msg.data, &discharge_data, sizeof(discharge_data));

	can_send_msg(&can1, &mc_msg);
}

void compute_send_mc_charge_message(acc_data_t* bmsdata)
{

	struct __attribute__((packed)) {
		uint16_t max_charge;
	} charge_data;

	/* scale to A * 10 */
	charge_data.max_charge = 10 * bmsdata->charge_limit;

	can_msg_t mc_msg = {0};
	mc_msg.id = 0x176;  // 0x0A is the dcl id, 0x157 is the device id set by us
	mc_msg.len = 8;
	memcpy(mc_msg.data, &charge_data, sizeof(charge_data));

	can_send_msg(&can1, &mc_msg);
}

void compute_send_acc_status_message(acc_data_t* bmsdata)
{

	struct __attribute__((packed)) {
		uint16_t packVolt;
		uint16_t pack_current;
		uint16_t pack_ah;
		uint8_t pack_soc;
		uint8_t pack_health;
	} acc_status_msg_data;

	acc_status_msg_data.packVolt	    = __builtin_bswap16(bmsdata->pack_voltage);
	acc_status_msg_data.pack_current = __builtin_bswap16((uint16_t)(bmsdata->pack_current)); // convert with 2s complement
	acc_status_msg_data.pack_ah	    = __builtin_bswap16(0);
	acc_status_msg_data.pack_soc	    = bmsdata->soc;
	acc_status_msg_data.pack_health  = 0;

	can_msg_t acc_msg;
	acc_msg.id = 0x00; // TODO replace with correct ID;
	acc_msg.len = sizeof(acc_status_msg_data);
	memcpy(acc_msg.data, &acc_status_msg_data, sizeof(acc_status_msg_data));

	can_send_msg(&can1, &acc_msg);
}

void compute_send_bms_status_message(acc_data_t* bmsdata, int bms_state, bool balance)
{
    struct __attribute__((packed)) {
        uint8_t state;
        uint32_t fault;
        int8_t temp_avg;
        uint8_t temp_internal;
        uint8_t balance;
    } bms_status_msg_data;

    bms_status_msg_data.temp_avg = (int8_t)(bmsdata->avg_temp);
    bms_status_msg_data.state = (uint8_t)(bms_state);
    bms_status_msg_data.fault = bmsdata->fault_code;
    bms_status_msg_data.temp_internal = (uint8_t)(0);
    bms_status_msg_data.balance = (uint8_t)(balance);

    can_msg_t acc_msg;
    acc_msg.id = 0x00; // TODO replace with correct ID;
    acc_msg.len = sizeof(bms_status_msg_data);
    memcpy(acc_msg.data, &bms_status_msg_data, sizeof(bms_status_msg_data));

    can_send_msg(&can1, &acc_msg);
}

void compute_send_shutdown_ctrl_message(uint8_t mpe_state)
{
    struct __attribute__((packed)) {
        uint8_t mpeState;
    } shutdown_control_msg_data;

    shutdown_control_msg_data.mpeState = mpe_state;

    can_msg_t acc_msg;
    acc_msg.id = 0x00; // TODO replace with correct ID;
    acc_msg.len = sizeof(shutdown_control_msg_data);
    memcpy(acc_msg.data, &shutdown_control_msg_data, sizeof(shutdown_control_msg_data));

    can_send_msg(&can1, &acc_msg);
}

void compute_send_cell_data_message(acc_data_t* bmsdata)
{
    struct __attribute__((packed)) {
        uint16_t high_cell_voltage;
        uint8_t high_cell_id;
        uint16_t low_cell_voltage;
        uint8_t low_cell_id;
        uint16_t volt_avg;
    } cell_data_msg_data;

    cell_data_msg_data.high_cell_voltage = bmsdata->max_voltage.val;
    cell_data_msg_data.high_cell_id = bmsdata->max_voltage.chipIndex;
    cell_data_msg_data.low_cell_voltage = bmsdata->min_voltage.val;
    cell_data_msg_data.low_cell_id = (bmsdata->min_voltage.chipIndex << 4) | bmsdata->min_voltage.cellNum;
    cell_data_msg_data.volt_avg = bmsdata->avg_voltage;

    can_msg_t acc_msg;
    acc_msg.id = 0x00; // TODO replace with correct ID;
    acc_msg.len = sizeof(cell_data_msg_data);
    memcpy(acc_msg.data, &cell_data_msg_data, sizeof(cell_data_msg_data));

    can_send_msg(&can1, &acc_msg);
}

void compute_send_cell_voltage_message(uint8_t cell_id, uint16_t instant_voltage,
                                       uint16_t internal_Res, uint8_t shunted,
                                       uint16_t open_voltage)
{
    struct __attribute__((packed)) {
        uint8_t cellID;
        uint16_t instantVoltage;
        uint16_t internalResistance;
        uint8_t shunted;
        uint16_t openVoltage;
    } cell_voltage_msg_data;

    cell_voltage_msg_data.cellID = cell_id;
    cell_voltage_msg_data.instantVoltage = __builtin_bswap16(instant_voltage);
    cell_voltage_msg_data.internalResistance = __builtin_bswap16(internal_Res);
    cell_voltage_msg_data.shunted = shunted;
    cell_voltage_msg_data.openVoltage = __builtin_bswap16(open_voltage);

    can_msg_t acc_msg;
    acc_msg.id = 0x00; // TODO replace with correct ID;
    acc_msg.len = sizeof(cell_voltage_msg_data);
    memcpy(acc_msg.data, &cell_voltage_msg_data, sizeof(cell_voltage_msg_data));

    can_send_msg(&can1, &acc_msg);
}

void compute_send_current_message(acc_data_t* bmsdata)
{
    struct __attribute__((packed)) {
        uint16_t dcl;
        uint16_t ccl;
        uint16_t pack_curr;
    } current_status_msg_data;

    current_status_msg_data.dcl = bmsdata->discharge_limit;
    current_status_msg_data.ccl = bmsdata->charge_limit;
    current_status_msg_data.pack_curr = bmsdata->pack_current;

    can_msg_t acc_msg;
    acc_msg.id = 0x00; // TODO replace with correct ID;
    acc_msg.len = sizeof(current_status_msg_data);
    memcpy(acc_msg.data, &current_status_msg_data, sizeof(current_status_msg_data));

    can_send_msg(&can1, &acc_msg);
}

// TODO ADD THIS BACK
// void compute_mc_callback(const CAN_message_t& currentStatusMsg)
// {
// 	return;
// }

void compute_send_cell_temp_message(acc_data_t* bmsdata)
{
    struct __attribute__((packed)) {
        uint16_t max_cell_temp;
        uint8_t max_cell_id;
        uint16_t min_cell_temp;
        uint8_t min_cell_id;
        uint16_t average_temp;
    } cell_temp_msg_data;

    cell_temp_msg_data.max_cell_temp = bmsdata->max_temp.val;
    cell_temp_msg_data.max_cell_id = (bmsdata->max_temp.chipIndex << 4) | (bmsdata->max_temp.cellNum - 17);
    cell_temp_msg_data.min_cell_temp = bmsdata->min_temp.val;
    cell_temp_msg_data.min_cell_id = (bmsdata->min_temp.chipIndex << 4) | (bmsdata->min_temp.cellNum - 17);
    cell_temp_msg_data.average_temp = bmsdata->avg_temp;

    can_msg_t acc_msg;
    acc_msg.id = 0x00; // TODO replace with correct ID;
    acc_msg.len = sizeof(cell_temp_msg_data);
    memcpy(acc_msg.data, &cell_temp_msg_data, sizeof(cell_temp_msg_data));

    can_send_msg(&can1, &acc_msg);
}

void compute_send_segment_temp_message(acc_data_t* bmsdata)
{
    struct __attribute__((packed)) {
        int8_t segment1_average_temp;
        int8_t segment2_average_temp;
        int8_t segment3_average_temp;
        int8_t segment4_average_temp;
    } segment_temp_msg_data;

    segment_temp_msg_data.segment1_average_temp = bmsdata->segment_average_temps[0];
    segment_temp_msg_data.segment2_average_temp = bmsdata->segment_average_temps[1];
    segment_temp_msg_data.segment3_average_temp = bmsdata->segment_average_temps[2];
    segment_temp_msg_data.segment4_average_temp = bmsdata->segment_average_temps[3];

    uint8_t buff[4] = { 0 };
    memcpy(buff, &segment_temp_msg_data, sizeof(segment_temp_msg_data));

    can_msg_t acc_msg;
    acc_msg.id = 0x00; // TODO replace with correct ID;
    acc_msg.len = sizeof(segment_temp_msg_data);
    memcpy(acc_msg.data, &segment_temp_msg_data, sizeof(segment_temp_msg_data));

    can_send_msg(&can1, &acc_msg);
}

uint8_t calc_charger_led_state(acc_data_t* bms_data)
{
	enum LED_state {
		RED_BLINKING	   = 0x00,
		RED_CONSTANT	   = 0x01,
		YELLOW_BLINKING	   = 0x02,
		YELLOW_CONSTANT	   = 0x03,
		GREEN_BLINKING	   = 0x04,
		GREEN_CONSTANT	   = 0x05,
		RED_GREEN_BLINKING = 0x06
	};

	if ((bms_data->soc < 80) && (bms_data->pack_current > .5 * 10)) {
		return RED_BLINKING;
	} else if ((bms_data->soc < 80) && (bms_data->pack_current <= .5 * 10)) {
		return RED_CONSTANT;
	} else if ((bms_data->soc >= 80 && bms_data->soc < 95) && (bms_data->pack_current > .5 * 10)) {
		return YELLOW_BLINKING;
	} else if ((bms_data->soc >= 80 && bms_data->soc < 95) && (bms_data->pack_current <= .5 * 10)) {
		return YELLOW_CONSTANT;
	} else if ((bms_data->soc >= 95) && (bms_data->pack_current > .5 * 10)) {
		return GREEN_BLINKING;
	} else if ((bms_data->soc >= 95) && (bms_data->pack_current <= .5 * 10)) {
		return GREEN_CONSTANT;
	} else {
		return RED_GREEN_BLINKING;
	}
}

float read_ref_voltage()
{
	adc_config.Channel = ADC_CHANNEL_9;
	if (HAL_ADC_ConfigChannel(&hadc1, &adc_config) != HAL_OK) return -1;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	/* scaled to 2.5 as per datasheet */
	float ref_voltage = HAL_ADC_GetValue(&hadc1) * 2.5 / MAX_ADC_RESOLUTION;

	return ref_voltage;
}

float read_vout()
{
	adc_config.Channel = ADC_CHANNEL_15;
	if (HAL_ADC_ConfigChannel(&hadc1, &adc_config) != HAL_OK) return -1;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	/* scaled to 3.3 */
	float vout = HAL_ADC_GetValue(&hadc1) * 3.3 / MAX_ADC_RESOLUTION;

	return vout;
}

