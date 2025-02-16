#include "can_messages.h"

#include <math.h>

#include "can.h"
#include "can_handler.h"

int send_charging_message(uint16_t voltage_to_set, uint16_t current_to_set,
			  acc_data_t *bms_data, bool charging_enabled)
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

	if (charging_enabled) {
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

void send_mc_discharge_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		uint16_t max_discharge;
	} discharge_data;

	/* scale to A * 10 */
	discharge_data.max_discharge = 10 * bmsdata->discharge_limit;

	/* convert to big endian */
	endian_swap(&discharge_data.max_discharge,
		    sizeof(discharge_data.max_discharge));

	can_msg_t msg;
	msg.id = DISCHARGE_CANID;
	msg.len = DISCHARGE_SIZE;

	memcpy(msg.data, &discharge_data, sizeof(discharge_data));

	queue_can_msg(msg);
}

void send_mc_charge_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		int16_t max_charge;
	} charge_data;

	/* scale to A * 10 */
	charge_data.max_charge = -10 * bmsdata->charge_limit;

	/* convert to big endian */
	endian_swap(&charge_data.max_charge, sizeof(charge_data.max_charge));

	can_msg_t msg;
	msg.id = CHARGE_CANID;
	msg.len = CHARGE_SIZE;

	memcpy(msg.data, &charge_data, sizeof(charge_data));

	queue_can_msg(msg);
}

void send_acc_status_message(acc_data_t *bmsdata)
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

	can_msg_t msg;
	msg.id = ACC_STATUS_CANID;
	msg.len = ACC_STATUS_SIZE;

	memcpy(msg.data, &acc_status_msg_data, sizeof(acc_status_msg_data));

	queue_can_msg(msg);
}

void send_fault_status_message(acc_data_t *bmsdata)
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

	can_msg_t fault_msg;
	fault_msg.id = FAULT_STATUS_CANID;
	fault_msg.len = FAULT_STATUS_SIZE;

	memcpy(fault_msg.data, &fault_status_msg_data,
	       sizeof(fault_status_msg_data));

	queue_can_msg(fault_msg);
}

void send_bms_status_message(acc_data_t *bmsdata, int bms_state, bool balance)
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
	bms_status_msg_data.balance = (uint8_t)(balance);

	can_msg_t msg;
	msg.id = BMS_STATUS_CANID;
	msg.len = BMS_STATUS_SIZE;

	memcpy(msg.data, &bms_status_msg_data, sizeof(bms_status_msg_data));

	queue_can_msg(msg);
}

void send_shutdown_ctrl_message(uint8_t mpe_state)
{
	struct __attribute__((__packed__)) {
		uint8_t mpeState;
	} shutdown_control_msg_data;

	shutdown_control_msg_data.mpeState = mpe_state;

	can_msg_t msg;
	msg.id = SHUTDOWN_CTRL_CANID;
	msg.len = SHUTDOWN_CTRL_SIZE;

	memcpy(msg.data, &shutdown_control_msg_data,
	       sizeof(shutdown_control_msg_data));

	queue_can_msg(msg);
}

void send_cell_voltage_message(acc_data_t *bmsdata)
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

	can_msg_t msg;
	msg.id = CELL_DATA_CANID;
	msg.len = CELL_DATA_SIZE;

	memcpy(msg.data, &cell_data_msg_data, sizeof(cell_data_msg_data));

	queue_can_msg(msg);
}

void send_current_message(acc_data_t *bmsdata)
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

	can_msg_t msg;
	msg.id = CURRENT_CANID;
	msg.len = CURRENT_SIZE;

	memcpy(msg.data, &current_status_msg_data,
	       sizeof(current_status_msg_data));

	queue_can_msg(msg);
}

void send_cell_temp_message(acc_data_t *bmsdata)
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

	can_msg_t msg;
	msg.id = CELL_TEMP_CANID;
	msg.len = CELL_TEMP_SIZE;

	memcpy(msg.data, &cell_temp_msg_data, sizeof(cell_temp_msg_data));

	queue_can_msg(msg);
}

void send_segment_temp_message(acc_data_t *bmsdata)
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

	can_msg_t msg;
	msg.id = SEGMENT_TEMP_CANID;
	msg.len = SEGMENT_TEMP_SIZE;

	memcpy(msg.data, &segment_temp_msg_data, sizeof(segment_temp_msg_data));

	queue_can_msg(msg);
}

void send_fault_message(uint8_t status, int16_t curr, int16_t in_dcl)
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

	can_msg_t msg;
	msg.id = FAULT_CANID;
	msg.len = FAULT_SIZE;

	memcpy(msg.data, &fault_msg_data, sizeof(fault_msg_data));

	queue_can_msg(msg);
}

void send_fault_timer_message(uint8_t start_stop, uint32_t fault_code,
			      uint16_t data_1)
{
	struct __attribute__((__packed__)) {
		uint8_t start_stop;
		uint8_t fault_code;
		int16_t data_1;
	} fault_timer_msg_data;

	fault_timer_msg_data.start_stop = start_stop;
	fault_timer_msg_data.fault_code = log2(fault_code);
	fault_timer_msg_data.data_1 = data_1;

	endian_swap(&fault_timer_msg_data.fault_code,
		    sizeof(fault_timer_msg_data.fault_code));
	endian_swap(&fault_timer_msg_data.data_1,
		    sizeof(fault_timer_msg_data.data_1));

	can_msg_t msg;
	msg.id = FAULT_TIMER_CANID;
	msg.len = FAULT_TIMER_SIZE;

	memcpy(msg.data, &fault_timer_msg_data, sizeof(fault_timer_msg_data));

	queue_can_msg(msg);
}

void send_voltage_noise_message(acc_data_t *bmsdata)
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

	can_msg_t msg;
	msg.id = NOISE_CANID;
	msg.len = NOISE_SIZE;

	memcpy(msg.data, &voltage_noise_msg_data,
	       sizeof(voltage_noise_msg_data));

	queue_can_msg(msg);
}

void send_debug_message(uint8_t debug0, uint8_t debug1, uint16_t debug2,
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

	can_msg_t msg;
	msg.id = DEBUG_CANID;
	msg.len = DEBUG_SIZE;

	memcpy(msg.data, &debug_msg_data, 8);

	queue_can_msg(msg);
}

void send_cell_data_message(bool alpha, uint16_t temperature,
			    uint16_t voltage_a, uint16_t voltage_b,
			    uint8_t chip_ID, uint8_t cell_a, uint8_t cell_b,
			    bool discharging_a, bool discharging_b)
{
	endian_swap(&temperature, sizeof(temperature));
	endian_swap(&voltage_a, sizeof(voltage_a));
	endian_swap(&voltage_b, sizeof(voltage_b));

	struct __attribute__((__packed__)) {
		uint16_t temperature : 10;
		uint16_t voltage_a : 13;
		uint16_t voltage_b : 13;
		uint8_t chip_ID : 4;
		uint8_t cell_a : 4;
		uint8_t cell_b : 4;
		uint8_t discharging_a : 1;
		uint8_t discharging_b : 1;
	} cell_data_msg_data;

	cell_data_msg_data.temperature = temperature;
	cell_data_msg_data.voltage_a = voltage_a;
	cell_data_msg_data.voltage_b = voltage_b;
	cell_data_msg_data.chip_ID = chip_ID;
	cell_data_msg_data.cell_a = cell_a;
	cell_data_msg_data.cell_b = cell_b;
	cell_data_msg_data.discharging_a = discharging_a;
	cell_data_msg_data.discharging_b = discharging_b;

	can_msg_t msg;
	if (alpha) {
		msg.id = ALPHA_CELL_CANID;
	} else {
		msg.id = BETA_CELL_CANID;
	}
	msg.len = CELL_MSG_SIZE;

	memcpy(msg.data, &cell_data_msg_data, CELL_MSG_SIZE);

	queue_can_msg(msg);
}

void send_beta_status_a_message(uint16_t cell_temperature, uint16_t voltage,
				bool discharging, uint8_t chip,
				uint16_t segment_temperature,
				uint16_t die_temperature, uint16_t vpv)
{
	endian_swap(&cell_temperature, sizeof(cell_temperature));
	endian_swap(&voltage, sizeof(voltage));
	endian_swap(&segment_temperature, sizeof(segment_temperature));
	endian_swap(&die_temperature, sizeof(die_temperature));
	endian_swap(&vpv, sizeof(vpv));

	struct __attribute__((__packed__)) {
		uint16_t cell_temperature : 10;
		uint16_t voltage : 13;
		uint8_t discharging : 1;
		uint8_t chip : 4;
		uint16_t segment_temperature : 10;
		uint16_t die_temperature : 13;
		uint16_t vpv : 13;
	} beta_status_a_data;

	beta_status_a_data.cell_temperature = cell_temperature;
	beta_status_a_data.voltage = voltage;
	beta_status_a_data.discharging = discharging;
	beta_status_a_data.chip = chip;
	beta_status_a_data.segment_temperature = segment_temperature;
	beta_status_a_data.die_temperature = die_temperature;
	beta_status_a_data.vpv = vpv;

	can_msg_t msg;
	msg.id = BETA_STAT_A_CANID;
	msg.len = BETA_STAT_A_SIZE;

	memcpy(msg.data, &beta_status_a_data, BETA_STAT_A_SIZE);

	queue_can_msg(msg);
}

void send_beta_status_b_message(uint16_t vref2, uint16_t v_analog,
				uint16_t v_digital, uint8_t chip,
				uint16_t v_res, uint16_t vmv)
{
	endian_swap(&vref2, sizeof(vref2));
	endian_swap(&v_analog, sizeof(v_analog));
	endian_swap(&v_digital, sizeof(v_digital));
	endian_swap(&v_res, sizeof(v_res));
	endian_swap(&vmv, sizeof(vmv));

	struct __attribute__((__packed__)) {
		uint16_t vref2 : 13;
		uint16_t v_analog : 10;
		uint16_t v_digital : 10;
		uint8_t chip : 4;
		uint16_t v_res : 13;
		uint16_t vmv : 13;
	} beta_status_b_data;

	beta_status_b_data.vref2 = vref2;
	beta_status_b_data.v_analog = v_analog;
	beta_status_b_data.v_digital = v_digital;
	beta_status_b_data.chip = chip;
	beta_status_b_data.v_res = v_res;
	beta_status_b_data.vmv = vmv;

	can_msg_t msg;
	msg.id = BETA_STAT_B_CANID;
	msg.len = BETA_STAT_B_SIZE;

	memcpy(msg.data, &beta_status_b_data, BETA_STAT_B_SIZE);

	queue_can_msg(msg);
}

void send_beta_status_c_message(uint8_t chip, stc_ *flt_reg)
{
	struct __attribute__((__packed__)) {
		uint8_t chip : 4;
		uint8_t va_ov : 1;
		uint8_t va_uv : 1;
		uint8_t vd_ov : 1;
		uint8_t vd_uv : 1;
		uint8_t vde : 1;
		uint8_t vdel : 1;
		uint8_t spiflt : 1;
		uint8_t sleep : 1;
		uint8_t thsd : 1;
		uint8_t tmodchk : 1;
		uint8_t oscchk : 1;
		uint8_t otp1_med : 1;
		uint8_t otp2_med : 1;
	} beta_status_b_data;

	beta_status_b_data.chip = chip;

	beta_status_b_data.va_ov = flt_reg->va_ov;
	beta_status_b_data.va_uv = flt_reg->va_uv;
	beta_status_b_data.vd_ov = flt_reg->vd_ov;
	beta_status_b_data.vd_uv = flt_reg->vd_uv;
	beta_status_b_data.vde = flt_reg->vde;
	beta_status_b_data.vdel = flt_reg->vdel;
	beta_status_b_data.spiflt = flt_reg->spiflt;
	beta_status_b_data.sleep = flt_reg->sleep;
	beta_status_b_data.thsd = flt_reg->thsd;
	beta_status_b_data.tmodchk = flt_reg->tmodchk;
	beta_status_b_data.oscchk = flt_reg->oscchk;
	beta_status_b_data.otp1_med = flt_reg->otp1_med;
	beta_status_b_data.otp1_med = flt_reg->otp1_med;

	can_msg_t msg;
	msg.id = BETA_STAT_C_CANID;
	msg.len = BETA_STAT_C_SIZE;

	memcpy(msg.data, &beta_status_b_data, BETA_STAT_C_SIZE);

	queue_can_msg(msg);
}

void send_alpha_status_a_message(uint16_t segment_temp, uint8_t chip,
				 uint16_t die_temperature, uint16_t vpv,
				 uint16_t vmv, stc_ *flt_reg)
{
	endian_swap(&segment_temp, sizeof(segment_temp));
	endian_swap(&die_temperature, sizeof(die_temperature));
	endian_swap(&vpv, sizeof(vpv));
	endian_swap(&vmv, sizeof(vmv));

	struct __attribute__((__packed__)) {
		uint16_t segment_temp : 10;
		uint8_t chip : 4;
		uint16_t die_temperature : 13;
		uint16_t vpv : 13;
		uint16_t vmv : 13;
		uint8_t va_ov : 1;
		uint8_t va_uv : 1;
		uint8_t vd_ov : 1;
		uint8_t vd_uv : 1;
		uint8_t vde : 1;
		uint8_t vdel : 1;
		uint8_t spiflt : 1;
		uint8_t sleep : 1;
		uint8_t thsd : 1;
		uint8_t tmodchk : 1;
		uint8_t oscchk : 1;
	} alpha_status_a_data;

	alpha_status_a_data.va_ov = flt_reg->va_ov;
	alpha_status_a_data.va_uv = flt_reg->va_uv;
	alpha_status_a_data.vd_ov = flt_reg->vd_ov;
	alpha_status_a_data.vd_uv = flt_reg->vd_uv;
	alpha_status_a_data.vde = flt_reg->vde;
	alpha_status_a_data.vdel = flt_reg->vdel;
	alpha_status_a_data.spiflt = flt_reg->spiflt;
	alpha_status_a_data.sleep = flt_reg->sleep;
	alpha_status_a_data.thsd = flt_reg->thsd;
	alpha_status_a_data.tmodchk = flt_reg->tmodchk;
	alpha_status_a_data.oscchk = flt_reg->oscchk;

	alpha_status_a_data.segment_temp = segment_temp;
	alpha_status_a_data.chip = chip;
	alpha_status_a_data.die_temperature = die_temperature;
	alpha_status_a_data.vpv = vpv;
	alpha_status_a_data.vmv = vmv;

	can_msg_t msg;
	msg.id = ALPHA_STAT_A_CANID;
	msg.len = ALPHA_STAT_A_SIZE;

	memcpy(msg.data, &alpha_status_a_data, ALPHA_STAT_A_SIZE);

	queue_can_msg(msg);
}

void send_alpha_status_b_message(uint16_t v_res, uint8_t chip, uint16_t vref2,
				 uint16_t v_analog, uint16_t v_digital,
				 stc_ *flt_reg)
{
	endian_swap(&v_res, sizeof(v_res));
	endian_swap(&vref2, sizeof(vref2));
	endian_swap(&v_analog, sizeof(v_analog));
	endian_swap(&v_digital, sizeof(v_digital));

	struct __attribute__((__packed__)) {
		uint16_t v_res : 13;
		uint8_t chip : 4;
		uint16_t vref2 : 13;
		uint16_t v_analog : 13;
		uint16_t v_digital : 13;
		uint8_t otp1_med : 1;
		uint8_t otp2_med : 1;
	} alpha_status_b_data;

	alpha_status_b_data.otp1_med = flt_reg->otp1_med;
	alpha_status_b_data.otp2_med = flt_reg->otp2_med;

	alpha_status_b_data.v_res = v_res;
	alpha_status_b_data.chip = chip;
	alpha_status_b_data.vref2 = vref2;
	alpha_status_b_data.v_analog = v_analog;
	alpha_status_b_data.v_digital = v_digital;

	can_msg_t msg;
	msg.id = ALPHA_STAT_B_CANID;
	msg.len = ALPHA_STAT_B_SIZE;

	memcpy(msg.data, &alpha_status_b_data, ALPHA_STAT_B_SIZE);

	queue_can_msg(msg);
}