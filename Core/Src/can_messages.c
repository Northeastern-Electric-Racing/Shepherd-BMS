#include "can_messages.h"

#include <math.h>

#include "can.h"
#include "can_handler.h"
#include "bitstream.h"

#define BYTE_TO_BITS 8
#define THERM_BITS   10
#define VOLT_BITS    13
#define AUX_ADC_BITS 13
#define CHIP_ID_BTIS 4
#define CELL_ID_BITS 4
#define VA_VD_BITS   10 /* Vanalog and Vdigital internal references */

extern is_charging_enabled;

static unsigned short reverse_short(unsigned short val);

static unsigned short reverse_short(unsigned short val)
{
	return reverse_bits(val) >> 4;
}

/// @brief A helper which sends appropriate error to stdout and CAN if a bistream overflows
/// @param bitstream_res The bitstream to check for overflow
/// @param can_id The CAN ID this bistream data is intended for
/// @return 0 if success
static const bool handle_bitstream_overflow(bitstream_t *bitstream_res,
					    uint32_t can_id)
{
	if (!bitstream_res->overflow) {
		return 0;
	}

	//printf("CAN MESSAGE %ld overflowed!\n", can_id);

	static uint16_t overflow_cnt = 0;
	overflow_cnt++;
	struct __attribute__((__packed__)) {
		uint32_t can_id;
		uint16_t overflow_cnt;
	} overflow_data;

	endian_swap(&can_id, sizeof(can_id));
	overflow_data.can_id = can_id;
	overflow_data.overflow_cnt = overflow_cnt;

	can_msg_t overflow_msg = { .id = OVERFLOW_CANID,
				   .len = 6,
				   .data = { 0 } };

	memcpy(&overflow_msg.data, &overflow_data, sizeof(overflow_data));

	queue_can_msg(overflow_msg);

	return 0;
}

int send_charging_message(uint16_t voltage_to_set, uint16_t current_to_set,
			  acc_data_t *bms_data)
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

	can_msg_t charger_msg = { .id = 0x1806E5F4,
				  .id_is_extended = true,
				  .len = 8,
				  .data = { 0 } };
	memcpy(charger_msg.data, &charger_msg_data, sizeof(charger_msg_data));

	uint8_t temp = charger_msg.data[0];
	charger_msg.data[0] = charger_msg.data[1];
	charger_msg.data[1] = temp;
	temp = charger_msg.data[2];
	charger_msg.data[2] = charger_msg.data[3];
	charger_msg.data[3] = temp;

#ifdef CHARGING_ENABLED
	HAL_StatusTypeDef res = can_send_msg(&can2, &msg);
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

	can_msg_t msg = { .id = DISCHARGE_CANID,
			  .len = DISCHARGE_SIZE,
			  .data = { 0 } };

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

	can_msg_t msg = { .id = CHARGE_CANID,
			  .len = CHARGE_SIZE,
			  .data = { 0 } };

	memcpy(msg.data, &charge_data, sizeof(charge_data));

	queue_can_msg(msg);
}

void send_acc_status_message(acc_data_t *bmsdata)
{
	struct __attribute__((__packed__)) {
		uint16_t packVolt;
		int16_t pack_current;
		uint16_t pack_ah;
		uint8_t pack_soc;
		uint8_t pack_health;
	} acc_status_msg_data;

	acc_status_msg_data.packVolt = bmsdata->pack_voltage;
	acc_status_msg_data.pack_current =
		(int16_t)(bmsdata->pack_current *
			  10); // converted to signed int and scaled by 10
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

	can_msg_t msg = { .id = ACC_STATUS_CANID,
			  .len = ACC_STATUS_SIZE,
			  .data = { 0 } };

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

	can_msg_t fault_msg = { .id = FAULT_STATUS_CANID,
				.len = FAULT_STATUS_SIZE,
				.data = { 0 } };

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

	can_msg_t msg = { .id = BMS_STATUS_CANID,
			  .len = BMS_STATUS_SIZE,
			  .data = { 0 } };

	memcpy(msg.data, &bms_status_msg_data, sizeof(bms_status_msg_data));

	queue_can_msg(msg);
}

void send_shutdown_ctrl_message(uint8_t mpe_state)
{
	struct __attribute__((__packed__)) {
		uint8_t mpeState;
	} shutdown_control_msg_data;

	shutdown_control_msg_data.mpeState = mpe_state;

	can_msg_t msg = { .id = SHUTDOWN_CTRL_CANID,
			  .len = SHUTDOWN_CTRL_SIZE,
			  .data = { 0 } };

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

	can_msg_t msg = { .id = CELL_DATA_CANID,
			  .len = CELL_DATA_SIZE,
			  .data = { 0 } };

	memcpy(msg.data, &cell_data_msg_data, sizeof(cell_data_msg_data));

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

	can_msg_t msg = { .id = CELL_TEMP_CANID,
			  .len = CELL_TEMP_SIZE,
			  .data = { 0 } };

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

	can_msg_t msg = { .id = SEGMENT_TEMP_CANID,
			  .len = SEGMENT_TEMP_SIZE,
			  .data = { 0 } };

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

	can_msg_t msg = { .id = FAULT_CANID, .len = FAULT_SIZE, .data = { 0 } };

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

	can_msg_t msg = { .id = FAULT_TIMER_CANID,
			  .len = FAULT_TIMER_SIZE,
			  .data = { 0 } };

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

	can_msg_t msg = { .id = NOISE_CANID, .len = NOISE_SIZE, .data = { 0 } };

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

	can_msg_t msg = { .id = DEBUG_CANID, .len = DEBUG_SIZE, .data = { 0 } };

	memcpy(msg.data, &debug_msg_data, 8);

	queue_can_msg(msg);
}

// verified by Jack on 3/17/2025 for beta and alpha chip 0
void send_cell_data_message(bool alpha, float temperature, float voltage_a,
			    float voltage_b, uint8_t chip_ID, uint8_t cell_a,
			    uint8_t cell_b, bool discharging_a,
			    bool discharging_b)
{
	// clang-format off
	can_msg_t msg = { .len = CELL_MSG_SIZE, .data = { 0 } };
	if (alpha) {
		msg.id = ALPHA_CELL_CANID;
	} else {
		msg.id = BETA_CELL_CANID;
	}

	// patch bc 0 to 4
	chip_ID /= 2;
	

	// if (alpha) {
	// 	printf("ALPHA: c%d\n", chip_ID);
	// } else {
	// 	printf("BETA: c%d\n", chip_ID);
	// }
	// printf("TEMP %d: %f\n", cell_a, temperature);
	// printf("VOLT %d: %f  b%d\n", cell_a, voltage_a, discharging_a);
	// printf("VOLT %d: %f  b%d\n", cell_b, voltage_b, discharging_b);

	/* Multiply data by scaling factor before converuting to int */
	temperature *= 10;
	voltage_a *= 1000;
	voltage_b *= 1000;

	bitstream_t cell_data_message;
	uint8_t bitstream_data[7];
	bitstream_init(&cell_data_message, bitstream_data, 7); // Create 7-byte bitstream

	bitstream_add(&cell_data_message, temperature, 10); 			// Cell temperature (10 bits)
	bitstream_add(&cell_data_message, voltage_a, 13);   			// Voltage A (13 bits)
	bitstream_add(&cell_data_message, voltage_b, 13);   			// Voltage B (13 bits)
	bitstream_add(&cell_data_message, chip_ID, 4);   // Chip ID (4 bits)
	bitstream_add(&cell_data_message, cell_a, 4);    // Cell A (4 bits)
	bitstream_add(&cell_data_message, cell_b, 4);    // Cell B (4 bits)
	bitstream_add(&cell_data_message, discharging_a, 1); 			// Discharging A (1 bit)
	bitstream_add(&cell_data_message, discharging_b, 1); 			// Discharging B (1 bit)
	bitstream_add(&cell_data_message, 0, 6);             			// Extra (6 bits)

	memcpy(msg.data, &bitstream_data, CELL_MSG_SIZE);

	handle_bitstream_overflow(&cell_data_message, msg.id);

	queue_can_msg(msg);
}

// TODO confirm cell 10 vs 11?. Jack verified VPV wil chip 0 on 3/17/2025
void send_beta_status_a_message(float cell_temperature, float voltage,
				bool discharging, uint8_t chip,
				float segment_temperature,
				float die_temperature, float vpv)
{
	can_msg_t msg = { .id = BETA_STAT_A_CANID, .len = BETA_STAT_A_SIZE, .data = { 0 } };

	// patch bc 0 to 4
	chip /= 2;

	// printf("BETA Chip: %d\n", chip);
	// printf("VOLT 10: %f, b%d\n", voltage, discharging);
	// printf("SegTemp: %f\n", segment_temperature);
	// printf("DieTemp: %f\n", die_temperature);
	// printf("VPV: %f\n", vpv);

	cell_temperature *= 10;
	voltage *= 1000;
	segment_temperature *= 10;
	die_temperature *= 100;
	vpv *= 100;

	bitstream_t beta_status_a_message;
	uint8_t bitstream_data[8];
	bitstream_init(&beta_status_a_message, bitstream_data,
		       8); // Create 8-byte bitstream

	bitstream_add(&beta_status_a_message, cell_temperature,
		      10); // Cell temperature (10 bits)
	bitstream_add(&beta_status_a_message, voltage, 13); // Voltage (13 bits)
	bitstream_add(&beta_status_a_message, discharging,
		      1); // Discharging (1 bit)
	bitstream_add(&beta_status_a_message, chip,
		      4); // Chip ID (4 bits)
	bitstream_add(&beta_status_a_message, segment_temperature,
		      10); // Segment temperature (10 bits)
	bitstream_add(&beta_status_a_message, die_temperature,
		      13); // Die temperature (13 bits)
	bitstream_add(&beta_status_a_message, vpv, 13); // Vpv (12 bits)

	memcpy(msg.data, &bitstream_data, BETA_STAT_A_SIZE);

	handle_bitstream_overflow(&beta_status_a_message, msg.id);

	queue_can_msg(msg);
}

// verified by Jack on chip 0, 3/12/2025.
void send_beta_status_b_message(float vref2, float v_analog, float v_digital,
				uint8_t chip, float v_res, float vmv)
{
	can_msg_t msg = { .id = BETA_STAT_B_CANID, .len = BETA_STAT_B_SIZE, .data = { 0 } };

	// patch bc 0 to 4
	chip /= 2;

	// printf("BETA Chip: %d\n", chip);
	// printf("Vref2 %f\n", vref2);
	// printf("v_analog %f\n", v_analog);
	// printf("v_digital %f\n", v_digital);
	// printf("v_res %f\n", v_res);
	// printf("vmv %f\n", vmv);

	vref2 *= 1000;
	v_analog *= 100;
	v_digital *= 100;
	v_res *= 1000;
	vmv *= 1000;

	bitstream_t beta_status_b_message;
	uint8_t bitstream_data[8];
	bitstream_init(&beta_status_b_message, bitstream_data, 8); // Create 8-byte bitstream

	bitstream_add(&beta_status_b_message, vref2, 13); 				// Vref2 (13 bits)
	bitstream_add(&beta_status_b_message, v_analog, 10); 			// Vanalog (10 bits)
	bitstream_add(&beta_status_b_message, v_digital, 10); 			// Vdigital (10 bits)
	bitstream_add(&beta_status_b_message, chip, 4); 	// Chip ID (4 bits)
	bitstream_add(&beta_status_b_message, v_res, 13); 				// Vres (13 bits)
	bitstream_add(&beta_status_b_message, vmv, 13); 				// Vmv (13 bits)
	bitstream_add(&beta_status_b_message, 0, 1); 					// Extra (1 bit)

	memcpy(msg.data, &bitstream_data, BETA_STAT_B_SIZE);

	handle_bitstream_overflow(&beta_status_b_message, msg.id);

	queue_can_msg(msg);
}

// verified by Jack on chip 0 3/12/2025.  For some reason OTP1_MED triggering without print?
void send_beta_status_c_message(uint8_t chip, stc_ *flt_reg)
{
	can_msg_t msg = { .id = BETA_STAT_C_CANID, .len = BETA_STAT_C_SIZE, .data = { 0 } };

	// patch bc 0 to 4
	chip /= 2;

	bitstream_t beta_status_c_message;
	uint8_t bitstream_data[3];
	bitstream_init(&beta_status_c_message, bitstream_data, 3); // Create 3-byte bitstream

	bitstream_add(&beta_status_c_message, chip, 4);	// Chip ID (4 bits)
	bitstream_add(&beta_status_c_message, flt_reg->va_ov, 1);		// VA_OV (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->va_uv, 1);		// VA_UV (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->vd_ov, 1);		// VD_OV (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->vd_uv, 1);		// VD_OV (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->vde, 1);			// VDE (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->vdel, 2);		// VDEL (2 bits)
	bitstream_add(&beta_status_c_message, flt_reg->spiflt, 1);		// SPIFLT (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->sleep, 1);		// SLEEP (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->thsd, 1);		// THSD (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->tmodchk, 1);		// TMODCHK (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->oscchk, 1);		// OSCCHK (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->otp1_med, 1);	// OTP1_MED (1 bit)
	bitstream_add(&beta_status_c_message, flt_reg->otp2_med, 1);	// OTP2_MED (1 bit)
	bitstream_add(&beta_status_c_message, 0, 7);					// Extra (7 bits)

	memcpy(msg.data, &bitstream_data, BETA_STAT_C_SIZE);

	handle_bitstream_overflow(&beta_status_c_message, msg.id);

	queue_can_msg(msg);
}

// verified 3/17/2025 for chip 0 by Jack, EXCLUDING VMV (see TODO)
void send_alpha_status_a_message(float segment_temp, uint8_t chip,
				 float die_temperature, float vpv, float vmv,
				 stc_ *flt_reg)
{
	can_msg_t msg = { .id = ALPHA_STAT_A_CANID, .len = ALPHA_STAT_A_SIZE, .data = { 0 } };


	// printf("SegTemp %f\n", segment_temp);
	// printf("DieTemp %f\n", die_temperature);
	// printf("VPV %f\n", vpv);
	// printf("VMV %f\n", vmv);

	segment_temp *= 10;

	die_temperature *= 100;
	vpv *= 100;
	vmv *= 1000;


	bitstream_t alpha_status_a_message;
	uint8_t bitstream_data[8];
	bitstream_init(&alpha_status_a_message, bitstream_data, 8);	// Create 8-byte bitstream

	bitstream_add(&alpha_status_a_message, segment_temp, 10);		// Segment Temp (10 bits)
	bitstream_add(&alpha_status_a_message, reverse_short(chip), 4);	// Chip ID (4 bits)
	bitstream_add(&alpha_status_a_message, die_temperature, 13);	// Die Temp (13 bits)
	bitstream_add(&alpha_status_a_message, vpv, 13);				// Vpv (13 bits)
	// TODO : VMV could be negative, how is that gonna work?
	bitstream_add(&alpha_status_a_message, vmv, 13);					// Vmv (8 bits)			// Vpv (5 bits)
	bitstream_add(&alpha_status_a_message, flt_reg->va_ov, 1);		// VA_OV (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->va_uv, 1);		// VA_UV (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->vd_ov, 1);		// VD_OV (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->vd_uv, 1);		// VD_UV (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->vde, 1);		// VDE (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->vdel, 1);		// VDEL (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->spiflt, 1);		// SPIFLT (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->sleep, 1);		// SLEEP (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->thsd, 1);		// THSD (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->tmodchk, 1);	// TMODCHK (1 bit)
	bitstream_add(&alpha_status_a_message, flt_reg->oscchk, 1);	 	// OSCCHK (1 bit)
	
	memcpy(msg.data, &bitstream_data, ALPHA_STAT_A_SIZE);

	handle_bitstream_overflow(&alpha_status_a_message, msg.id);

	queue_can_msg(msg);
}

// verified 3/17/2025 for chip 0 by Jack. mostly faults too
void send_alpha_status_b_message(float v_res, uint8_t chip, float vref2,
				 float v_analog, float v_digital, stc_ *flt_reg)
{
	can_msg_t msg = { .id = ALPHA_STAT_B_CANID, .len = ALPHA_STAT_B_SIZE, .data = { 0 } };

	// printf("Vres %f\n", v_res);
	// printf("Vref2 %f\n", vref2);
	// printf("Vanalog %f\n", v_analog);
	// printf("Vdigital %f\n", v_digital);

	v_res *= 1000;
	vref2 *= 1000;
	v_analog *= 1000;
	v_digital *= 1000;

	bitstream_t alpha_status_b_message;
	uint8_t bitstream_data[8];
	bitstream_init(&alpha_status_b_message, bitstream_data, 8);	// Create 8-byte bitstream

	bitstream_add(&alpha_status_b_message, v_res, 13);				// Vres (13 bits)
	bitstream_add(&alpha_status_b_message, reverse_short(chip), 4);	// Chip ID (4 bits)
	bitstream_add(&alpha_status_b_message, vref2, 13);				// Vref2 (13 bits)
	bitstream_add(&alpha_status_b_message, v_analog, 13);			// Vanalog (13 bits)
	bitstream_add(&alpha_status_b_message, v_digital, 13);			// Vdigital (13 bits)
	bitstream_add(&alpha_status_b_message, flt_reg->otp1_med, 1);	// OTP1_MED (1 bit)
	bitstream_add(&alpha_status_b_message, flt_reg->otp2_med, 1);	// OTP2_MED (1 bit)
	bitstream_add(&alpha_status_b_message, 0, 6);					// Extra (6 bits)

	memcpy(msg.data, &bitstream_data, ALPHA_STAT_B_SIZE);

	handle_bitstream_overflow(&alpha_status_b_message, msg.id);

	queue_can_msg(msg);
	// clang-format on
}

/**
 * @brief Sends a CAN message containing the PEC error count for a specific chip.
 *
 * @param chip_num The index of the chip that reported PEC errors.
 * @param pec_count The total number of PEC errors detected for the specified chip.
 */
void send_pec_error_message(uint8_t chip_num, uint16_t pec_count)
{
	struct __attribute__((__packed__)) {
		uint8_t chip_number;
		uint16_t pec_error_count;
	} pec_data;

	pec_data.chip_number = chip_num;
	pec_data.pec_error_count = pec_count;

	endian_swap(&pec_data.pec_error_count,
		    sizeof(pec_data.pec_error_count));

	can_msg_t msg = { .id = PEC_ERROR_CANID,
			  .len = PEC_ERROR_SIZE,
			  .data = { 0 } };

	memcpy(&msg.data, &pec_data, sizeof(pec_data));

	queue_can_msg(msg);
}