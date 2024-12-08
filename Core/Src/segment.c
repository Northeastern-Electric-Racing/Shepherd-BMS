#include "segment.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// TEMPORARY

#include "common.h"
#include "adBms6830CmdList.h"
#include "adBms6830GenericType.h"
#include "serialPrintResult.h"
#include "adBms6830ParseCreate.h"
#include "mcuWrapper.h"
#include "cmsis_os.h"

#define ALL_GPIOS_ARE_INPUTS 0x3FF
#define T_READY		     10 /* microseconds*/
#define T_IDLE		     4.3 /* milliseconds, minimum. typ is 5.5, max is 6.7 */
#define T_WAKE		     200 /* microseconds */
#define T_SLEEP		     1.8 /* seconds minimum, typ is 2, max is 2.2 */
#define T_REFUP		     2.7 /* milliseconds minimum, typ is 3.5, max is 4.4 */

typedef enum {
	DISCHARGE_ENABLED = 0,
	MUTE_ACTIVATED_DISCHARGE_DISABLED = 1
} MUTE_ST;

#define TOTAL_IC 1
cell_asic IC[TOTAL_IC];

RD REDUNDANT_MEASUREMENT = RD_OFF;
CH AUX_CH_TO_CONVERT = AUX_ALL;
CONT CONTINUOUS_MEASUREMENT = SINGLE;
OW_C_S CELL_OPEN_WIRE_DETECTION = OW_OFF_ALL_CH;
OW_AUX AUX_OPEN_WIRE_DETECTION = AUX_OW_OFF;
PUP OPEN_WIRE_CURRENT_SOURCE = PUP_DOWN;
DCP DISCHARGE_PERMITTED = DCP_OFF;
RSTF RESET_FILTER = RSTF_OFF;
ERR INJECT_ERR_SPI_READ = WITHOUT_ERR;

/*Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS*/
LOOP_MEASURMENT MEASURE_CELL =
	ENABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_AVG_CELL =
	ENABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_F_CELL =
	ENABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_S_VOLTAGE =
	DISABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_AUX =
	DISABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_RAUX =
	DISABLED; /*   This is ENABLED or DISABLED       */
LOOP_MEASURMENT MEASURE_STAT =
	ENABLED; /*   This is ENABLED or DISABLED       */

// END

#define THERM_WAIT_TIME	   500 /* ms */
#define VOLTAGE_WAIT_TIME  500 /* ms */
#define THERM_AVG	   15 /* Number of values to average */
#define MAX_VOLT_DELTA	   2500
#define MAX_CONSEC_NOISE   10
#define GPIO_EXPANDER_ADDR 0x40
#define GPIO_REGISTER_ADDR 0x09

// TODO ensure spi 1 is correct for talking to segs
extern SPI_HandleTypeDef hspi1;

uint8_t local_config[NUM_CHIPS][6] = {};
uint8_t therm_avg_counter = 0;

chipdata_t *segment_data = NULL;
chipdata_t previous_data[NUM_CHIPS] = {};
uint16_t discharge_commands[NUM_CHIPS] = {};

nertimer_t therm_timer;
nertimer_t voltage_reading_timer;
nertimer_t variance_timer;

int voltage_error = 0; // not faulted
int therm_error = 0; // not faulted
uint16_t crc_error_check = 0;

/* our segments are mapped backwards and in pairs, so they are read in 1,0 then
 * 3,2, etc*/
const int mapping_correction[12] = { 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10 };

uint16_t therm_settle_time_ = 0;

const uint32_t VOLT_TEMP_CONV[106] = {
	157300, 148800, 140300, 131800, 123300, 114800, 108772, 102744, 96716,
	90688,	84660,	80328,	75996,	71664,	67332,	63000,	59860,	56720,
	53580,	50440,	47300,	45004,	42708,	40412,	38116,	35820,	34124,
	32428,	30732,	29036,	27340,	26076,	24812,	23548,	22284,	21020,
	20074,	19128,	18182,	17236,	16290,	15576,	14862,	14148,	13434,
	12720,	12176,	11632,	11088,	10544,	10000,	9584,	9168,	8753,
	8337,	7921,	7600,	7279,	6957,	6636,	6315,	6065,	5816,
	5566,	5317,	5067,	4872,	4676,	4481,	4285,	4090,	3936,
	3782,	3627,	3473,	3319,	3197,	3075,	2953,	2831,	2709,
	2612,	2514,	2417,	2319,	2222,	2144,	2066,	1988,	1910,
	1832,	1769,	1706,	1644,	1581,	1518,	1467,	1416,	1366,
	1315,	1264,	1223,	1181,	1140,	1098,	1057
};

const int32_t VOLT_TEMP_CALIB_OFFSET = 0;

/* private function prototypes */
void serialize_i2c_msg(uint8_t data_to_write[][3], uint8_t comm_output[][6]);
int8_t steinhart_est(uint16_t V);
void variance_therm_check(void);
void discard_neutrals(void);
void pull_chip_configuration(void);
int16_t calc_average(void);
int8_t calc_therm_standard_dev(int16_t avg_temp);
void init_chip(cell_asic *chip);
void set_cell_discharge(cell_asic *chip, uint8_t cell, bool discharge);

/**
 * @brief Set a bit in a uint16
 *  
 * @param number uint16 to change.
 * @param n Nth bit to change.
 * @param x true sets, false clears.
 * @return uint16_t New uint16.
 */
inline uint16_t set_uint16_bit(uint16_t number, uint16_t n, bool x)
{
	return (number & ~((uint16_t)1 << n)) | ((uint16_t)x << n);
}

void adbms_wake()
{
	adBmsWakeupIc(NUM_CHIPS);
}

/**
 * @brief Initialize a chip with default values. 
 * 
 * @param chip Pointer to chip to initialize.
 */
void init_chip(cell_asic *chip)
{
	chip->tx_cfga.refon = PWR_UP;
	chip->tx_cfga.cth = CVT_8_1mV;
	chip->tx_cfga.flag_d = 0;

	// No soak on AUX ADCs
	chip->tx_cfga.soakon = SOAKON_CLR;

	// short soak time by default
	chip->tx_cfga.owrng = TIME_32US_TO_4_1MS;

	chip->tx_cfga.owa = OWA0;

	// All GPIOs are inputs by default
	chip->tx_cfga.gpo = ALL_GPIOS_ARE_INPUTS;

	// Registers are unfrozen
	chip->tx_cfga.snap = SNAP_OFF;

	// Charging is deactivated
	chip->tx_cfga.mute_st = MUTE_ACTIVATED_DISCHARGE_DISABLED;

	// Not an endpoint in the daisy chain
	chip->tx_cfga.comm_bk = false;

	// IIR filter disabled
	chip->tx_cfga.fc = IIR_FPA_OFF;

	// Init config B
	chip->tx_cfgb.vov = SetOverVoltageThreshold(4.2);
	chip->tx_cfgb.vuv = SetUnderVoltageThreshold(3.0);

	// Discharge timer monitor off
	chip->tx_cfgb.dtmen = DTMEN_OFF;

	// Set discharge timer range to 0 to 63 minutes with 1 minute increments
	chip->tx_cfgb.dtrng = RANG_0_TO_63_MIN;

	// Disable discharge timer
	chip->tx_cfgb.dcto = DCTO_TIMEOUT;

	// Disable discharge for all cells
	chip->tx_cfgb.dcc = 0;
}

/**
 * @brief Set the status of the REFON bit.
 * 
 * @param chip Pointer to the chip to modify.
 * @param state New state of the REFON bit.
 */
void set_REFON(cell_asic *chip, REFON state)
{
	chip->tx_cfga.refon = state;
}

/**
 * @brief Set the C-ADC vs. S-ADC comparison voltage threshold 
 * 
 * @param chip Pointer to the chip to modify.
 * @param threshold Threshold to set.
 */
void set_volt_adc_comp_thresh(cell_asic *chip, CTH threshold)
{
	chip->tx_cfga.cth = threshold;
}

/**
 * @brief Set the discharge state of a cell.
 * 
 * @param chip Pointer to chip with cell to modify.
 * @param cell ID of cell to modify.
 * @param discharge Cell discharge state. true to discharge, false to disable discharge.
 */
void set_cell_discharge(cell_asic *chip, uint8_t cell, bool discharge)
{
	chip->tx_cfgb.dcc = set_uint16_bit(chip->tx_cfgb.dcc, cell, discharge);
}

/**
 * @brief Write data to all chips.
 * 
 * @param chip Array of chips to write data to.
 * @param command Command to issue to the chip.
 * @param type Register type to write to.
 * @param group Group of registers to write to.
 */
void write_adbms_data(cell_asic chips[NUM_CHIPS], uint8_t command[2], TYPE type,
		      GRP group)
{
	adBmsWriteData(NUM_CHIPS, chips, command, type, group);
}

/**
 * @brief Read data from all chips.
 * 
 * @param chips Array of chips to read data to.
 * @param command Command to issue to the chip.
 * @param type Register type to write to.
 * @param group Group of registers to write to.
 */
void read_adbms_data(cell_asic chips[NUM_CHIPS], uint8_t command[2], TYPE type,
		     GRP group)
{
	adBmsReadData(NUM_CHIPS, chips, command, type, group);
}

/**
 * @brief Write config registers. Wakes chips before writing.
 * 
 * @param chips Array of chips to write config registers of.
 */
inline void write_config_regs(cell_asic chips[NUM_CHIPS])
{
	adbms_wake();
	write_adbms_data(chips, WRCFGA, Config, A);
	write_adbms_data(chips, WRCFGB, Config, B);
}

/**
 * @brief Initialize chips with default values.
 * 
 */
void segment_init()
{
	printf("Initializing Segments...");
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		init_chip(&IC[chip]);
	}
	write_config_regs(IC);
}

/**
 * @brief Get voltage readings from the C-ADCs.
 * 
 * @param chips Array of chips to get voltage readings from.
 */
void get_c_adc_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);

	adbms_wake();
	read_adbms_data(chips, RDCVALL, Rdcvall, ALL_GRP);
	// read_adbms_data(chip, RDCVA, Cell, A);
	// read_adbms_data(chip, RDCVB, Cell, B);
	// read_adbms_data(chip, RDCVC, Cell, C);
	// read_adbms_data(chip, RDCVD, Cell, D);
	// read_adbms_data(chip, RDCVE, Cell, E);
	// read_adbms_data(chip, RDCVF, Cell, F);
}

/**
 * @brief Get voltages from the S-ADCs.
 * 
 * @param chip Array of chips to get voltage readings from.
 */
void get_s_adc_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake();
	adBms6830_Adsv(CONTINUOUS, DCP_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLSADC);

	adbms_wake();
	read_adbms_data(chips, RDSALL, Rdsall, ALL_GRP);
	// read_adbms_data(chip, RDSVA, S_volt, A);
	// read_adbms_data(chip, RDSVB, S_volt, B);
	// read_adbms_data(chip, RDSVC, S_volt, C);
	// read_adbms_data(chip, RDSVD, S_volt, D);
	// read_adbms_data(chip, RDSVE, S_volt, E);
	// read_adbms_data(chip, RDSVF, S_volt, F);
}

/**
 * @brief Get the avgeraged cell voltages.
 * 
 * @param chip Array of chips to get voltage readings of.
 */
void get_avgd_cell_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);

	adbms_wake();
	read_adbms_data(chips, RDACALL, Rdacall, ALL_GRP);
}

/**
 * @brief Get the filtered cell volrages.
 * 
 * @param chip Array of chips to get voltage readings of.
 */
void get_filtered_cell_volrages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);

	adbms_wake();
	read_adbms_data(chips, RDFCALL, Rdfcall, ALL_GRP);
}

/**
 * @brief Get the c and s adc voltages. Does this with RDCSALL command.
 * 
 * @param chips Array of chips to get voltage readings of.
 */
void get_c_and_s_adc_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);

	adbms_wake();
	read_adbms_data(chips, RDCSALL, Rdcsall, ALL_GRP);
}

/**
 * @brief Read every register connected to the AUX ADC.
 * 
 * @param chips Array of chips to get voltage readings of.
 */
void read_aux_registers(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
	adBmsPollAdc(PLAUX1);

	adbms_wake();
	read_adbms_data(chips, RDAUXA, Aux, A);
	read_adbms_data(chips, RDAUXB, Aux, B);
	read_adbms_data(chips, RDAUXC, Aux, C);
	read_adbms_data(chips, RDAUXD, Aux, D);
}

/**
 * @brief Read voltages in every register connected to AUX2 ADC.
 * 
 * @param chips Array of chips to get voltages of.
 */
void read_aux2_registers(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adBms6830_Adax2(AUX_ALL);
	adBmsPollAdc(PLAUX2);

	adbms_wake();
	read_adbms_data(chips, RDRAXA, RAux, A);
	read_adbms_data(chips, RDRAXB, RAux, B);
	read_adbms_data(chips, RDRAXC, RAux, C);
	read_adbms_data(chips, RDRAXD, RAux, D);
}

/**
 * @brief Read status registers.
 * 
 * @param chips Array of chips to read voltages of.
 */
void adBms6830_read_status_registers(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
	adBmsPollAdc(PLAUX1);

	read_adbms_data(chips, RDSTATA, Status, A);
	read_adbms_data(chips, RDSTATB, Status, B);
	read_adbms_data(chips, RDSTATC, Status, C);
	read_adbms_data(chips, RDSTATD, Status, D);
	read_adbms_data(chips, RDSTATE, Status, E);
}

int pull_voltages()
{
	/**
   * If we haven't waited long enough between pulling voltage data
   * just copy over the contents of the last good reading and the fault status
   * from the most recent attempt
   */
	if (!is_timer_expired(&voltage_reading_timer) &&
	    voltage_reading_timer.active) {
		for (uint8_t i = 0; i < NUM_CHIPS; i++) {
			memcpy(segment_data[i].voltage,
			       previous_data[i].voltage,
			       sizeof(segment_data[i].voltage));
		}
		return voltage_error;
	}

	get_c_adc_voltages()

	/*

	OLD CODE THAT DID WORK FOR ADBMS

	uint16_t raw_voltages[NUM_CHIPS][NUM_CELLS_PER_CHIP];

	// printVoltages(TOTAL_IC, &IC[0], Cell);

	float voltage;
	uint8_t ic = 0;
	int16_t temp;
	// number of cells
	uint8_t channel = 16;
	uint8_t type = Cell;
	for (uint8_t index = 0; index < channel; index++) {
		if (type == Cell) {
			temp = IC[ic].cell.c_codes[index];
		} else if (type == AvgCell) {
			temp = IC[ic].acell.ac_codes[index];
		} else if (type == F_volt) {
			temp = IC[ic].fcell.fc_codes[index];
		} else if (type == S_volt) {
			temp = IC[ic].scell.sc_codes[index];
		} else if (type == Aux) {
			temp = IC[ic].aux.a_codes[index];
		} else if (type == RAux) {
			temp = IC[ic].raux.ra_codes[index];
		}

		voltage = getVoltage(temp);

		raw_voltages[ic][index] = (uint16_t)temp;

		segment_data[ic].voltage[index] = raw_voltages[ic][index];

		if (type == Cell) {
			// printf("C%d=%fV= %d raw\r\n", (index + 1), voltage,
			//        segment_data[ic].voltage[index]);
			if (index == (channel - 1)) {
				// printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
				// printf("PECError:%d", IC[ic].cccrc.cell_pec);
			}
		}
	}
	// printf("\n\n");

	*/

	/*
	float total_volts = 0;
	for (int i = 0; i < 16; i++) {
		printf("Raw %d: %f\n", i, getVoltage(segment_data[ic].voltage[i]));
		total_volts += getVoltage(segment_data[ic].voltage[i]);
	}
	printf("TOTAL VOLTAGE: %f", total_volts);*/

	/*
	if (MEASURE_AVG_CELL == ENABLED) {
		adBmsReadData(TOTAL_IC, &IC[0], RDACA, AvgCell, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDACB, AvgCell, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDACC, AvgCell, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDACD, AvgCell, D);
		adBmsReadData(TOTAL_IC, &IC[0], RDACE, AvgCell, E);
		adBmsReadData(TOTAL_IC, &IC[0], RDACF, AvgCell, F);
		printVoltages(TOTAL_IC, &IC[0], AvgCell);
	}

	if (MEASURE_F_CELL == ENABLED) {
		adBmsReadData(TOTAL_IC, &IC[0], RDFCA, F_volt, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCB, F_volt, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCC, F_volt, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCD, F_volt, D);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCE, F_volt, E);
		adBmsReadData(TOTAL_IC, &IC[0], RDFCF, F_volt, F);
		printVoltages(TOTAL_IC, &IC[0], F_volt);
	} */

	/*
	if (MEASURE_S_VOLTAGE == ENABLED) {
		adBmsWakeupIc(TOTAL_IC);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVA, S_volt, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVB, S_volt, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVC, S_volt, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVD, S_volt, D);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVE, S_volt, E);
		adBmsReadData(TOTAL_IC, &IC[0], RDSVF, S_volt, F);
		printVoltages(TOTAL_IC, &IC[0], S_volt);
	} */

	/*
	if (MEASURE_AUX == ENABLED) {
		adBms6830_Adax(AUX_OPEN_WIRE_DETECTION,
			       OPEN_WIRE_CURRENT_SOURCE, AUX_CH_TO_CONVERT);
		adBmsPollAdc(PLAUX1);
		adBmsReadData(TOTAL_IC, &IC[0], RDAUXA, Aux, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDAUXB, Aux, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDAUXC, Aux, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDAUXD, Aux, D);
		printVoltages(TOTAL_IC, &IC[0], Aux);
	} */

	/*
	if (MEASURE_RAUX == ENABLED) {
		adBmsWakeupIc(TOTAL_IC);
		adBms6830_Adax2(AUX_CH_TO_CONVERT);
		adBmsPollAdc(PLAUX2);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXA, RAux, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXB, RAux, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXC, RAux, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXD, RAux, D);
		printVoltages(TOTAL_IC, &IC[0], RAux);
	}	*/

	/*
	if (MEASURE_STAT == ENABLED) {
		adBms6830_Adax(AUX_OPEN_WIRE_DETECTION,
			       OPEN_WIRE_CURRENT_SOURCE, AUX_CH_TO_CONVERT);
		adBmsPollAdc(PLAUX1);
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATA, Status, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATB, Status, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATC, Status, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATD, Status, D);
		adBmsReadData(TOTAL_IC, &IC[0], RDSTATE, Status, E);
		printStatus(TOTAL_IC, &IC[0], Status, ALL_GRP);
	}*/

	/* Start the timer between readings if successful */
	start_timer(&voltage_reading_timer, VOLTAGE_WAIT_TIME);

	return 0;
}

int pull_thermistors()
{
	/* If polled too soon, just copy existing values from memory */
	if (!is_timer_expired(&therm_timer)) {
		for (uint8_t i = 0; i < NUM_CHIPS; i++) {
			memcpy(segment_data[i].thermistor_reading,
			       previous_data[i].thermistor_reading,
			       sizeof(segment_data[i].thermistor_reading));
			memcpy(segment_data[i].thermistor_value,
			       previous_data[i].thermistor_value,
			       sizeof(segment_data[i].thermistor_value));
		}
		return voltage_error;
	}

	uint16_t raw_temp_voltages[NUM_CHIPS][6];

	static uint8_t current_therm = 1;
	if (current_therm > 16) {
		current_therm = 1;
	}

	/* Sets multiplexors to select thermistors */
	select_therm(current_therm);
	HAL_Delay(200);
	// push_chip_configuration();
	HAL_Delay(3);
	// LTC6804_rdaux(ltc68041, 0, NUM_CHIPS, raw_temp_voltages);
	/* Rotate through all thermistor pairs (we can poll two at once) */
	for (uint8_t therm = 1; therm <= (NUM_THERMS_PER_CHIP / 2); therm++) {
		for (uint8_t c = 0; c < NUM_CHIPS; c++) {
			int corrected_index = mapping_correction[c];
			/*
       * Get current temperature LUT. Voltage is adjusted to account for 5V reg
       * fluctuations (index 2 is a reading of the ADC 5V ref)
       */
			if (therm == current_therm) {
				/* see "thermister decoding" in confluence in shepherd software 22A */
				uint16_t steinhart_input_low =
					10000 *
					(float)(((float)raw_temp_voltages[c][2]) /
							(raw_temp_voltages[c]
									  [0]) -
						1);
				uint16_t steinhart_input_high =
					10000 *
					(float)(((float)raw_temp_voltages[c][2]) /
							(raw_temp_voltages[c]
									  [1]) -
						1);

				segment_data[corrected_index]
					.thermistor_reading[therm - 1] =
					steinhart_est(steinhart_input_low);
				segment_data[corrected_index]
					.thermistor_reading[therm + 15] =
					steinhart_est(steinhart_input_high);

				/* Directly update for a set time from start up due to therm voltages
         * needing to settle */
				segment_data[corrected_index]
					.thermistor_value[therm - 1] =
					segment_data[corrected_index]
						.thermistor_reading[therm - 1];
				segment_data[corrected_index]
					.thermistor_value[therm + 15] =
					segment_data[corrected_index]
						.thermistor_reading[therm + 15];

				if (raw_temp_voltages[c][0] == LTC_BAD_READ ||
				    raw_temp_voltages[c][1] == LTC_BAD_READ ||
				    segment_data[corrected_index]
						    .thermistor_value[therm - 1] >
					    (MAX_CELL_TEMP + 5) ||
				    segment_data[corrected_index]
						    .thermistor_value[therm +
								      15] >
					    (MAX_CELL_TEMP + 5) ||
				    segment_data[corrected_index]
						    .thermistor_value[therm - 1] <
					    (MIN_CELL_TEMP - 5) ||
				    segment_data[corrected_index]
						    .thermistor_value[therm +
								      15] <
					    (MIN_CELL_TEMP - 5)) {
					memcpy(segment_data[corrected_index]
						       .thermistor_reading,
					       previous_data[c]
						       .thermistor_reading,
					       sizeof(segment_data[corrected_index]
							      .thermistor_reading));
					memcpy(segment_data[corrected_index]
						       .thermistor_value,
					       previous_data[c].thermistor_value,
					       sizeof(segment_data[corrected_index]
							      .thermistor_value));
				}
			} else {
				segment_data[corrected_index]
					.thermistor_reading[therm - 1] =
					previous_data[corrected_index]
						.thermistor_reading[therm - 1];
				segment_data[corrected_index]
					.thermistor_reading[therm + 15] =
					previous_data[corrected_index]
						.thermistor_reading[therm + 15];

				segment_data[corrected_index]
					.thermistor_value[therm - 1] =
					segment_data[corrected_index]
						.thermistor_reading[therm - 1];
				segment_data[corrected_index]
					.thermistor_value[therm + 15] =
					segment_data[corrected_index]
						.thermistor_reading[therm + 15];
			}
		}
	}
	current_therm++;
	start_timer(&therm_timer,
		    100 /*THERM_WAIT_TIME*/); /* Start timer for next reading */

	/* the following algorithms were used to eliminate noise on Car 17D - keep
   * them off if possible */
	// variance_therm_check();
	// standard_dev_therm_check();
	// averaging_therm_check();
	// discard_neutrals();

	return 0; /* Read successfully */
}

void segment_retrieve_data(chipdata_t databuf[NUM_CHIPS])
{
	segment_data = databuf;

	/* Pull voltages and thermistors and indiacate if there was a problem during
   * retrieval */
	voltage_error = pull_voltages();
	therm_error = pull_thermistors();

	/* Save the contents of the reading so that we can use it to fill in missing
   * data */
	memcpy(previous_data, segment_data, sizeof(chipdata_t) * NUM_CHIPS);

	segment_data = NULL;
}

void configure_discharge(uint8_t chip, uint16_t cells)
{
	/*
   * chipConfigurations[chip][4] == chipConfigurations[Literally what chip you
   * want][register] 4 and 5 are registers to discharge chips
   */
	local_config[chip][4] = (uint8_t)(cells & 0x00FF);

	/*
   * Register 5 is split in half, so we maintain the upper half and add in the
   * bottom half to discharge cells
   */
	local_config[chip][5] =
		(local_config[chip][5] & 0xF0) + (uint8_t)(cells >> 8);
}

void segment_enable_balancing(bool balance_enable)
{
	/*
   * Discharging all cells in series
   * Making the discharge command all 1's for all cells per chip
   */
	static const uint16_t DICHARGE_ALL_COMMAND = 0xFFFF >>
						     (16 - NUM_CELLS_PER_CHIP);

	if (balance_enable) {
		for (int c = 0; c < NUM_CHIPS; c++) {
			configure_discharge(c, DICHARGE_ALL_COMMAND);
			discharge_commands[c] = DICHARGE_ALL_COMMAND;
		}
		// push_chip_configuration();
	} else {
		for (int c = 0; c < NUM_CHIPS; c++) {
			configure_discharge(c, 0);
			discharge_commands[c] = 0;
		}
		// push_chip_configuration();
	}
}

// @todo Revisit after testing
void cell_enable_balancing(uint8_t chip_num, uint8_t cell_num,
			   bool balance_enable)
{
	pull_chip_configuration();

	if (balance_enable)
		discharge_commands[chip_num] |= (1 << cell_num);
	else
		discharge_commands[chip_num] &= ~(1 << cell_num);

	configure_discharge(chip_num, discharge_commands[chip_num]);

	// push_chip_configuration();
}

void segment_configure_balancing(
	bool discharge_config[NUM_CHIPS][NUM_CELLS_PER_CHIP])
{
	for (int c = 0; c < NUM_CHIPS; c++) {
		for (int cell = 0; cell < NUM_CELLS_PER_CHIP; cell++) {
			if (discharge_config[mapping_correction[c]][cell])
				discharge_commands[mapping_correction[c]] |=
					1 << cell;
			else
				discharge_commands[mapping_correction[c]] &=
					~(1 << cell);
		}

		configure_discharge(c, discharge_commands[c]);
	}
	// push_chip_configuration();
}

bool cell_is_balancing(uint8_t chip_num, uint8_t cell_num)
{
	/* If the cell is one of the first 8, check the 4th register */
	if (cell_num < 8) {
		return local_config[chip_num][4] & (1 << cell_num);
	}
	/* If the cell number is greater than 8, check the 5th register */
	else {
		return local_config[chip_num][5] & (1 << (cell_num - 8));
	}

	return false; /* default case */
}

bool segment_is_balancing()
{
	for (int c = 0; c < NUM_CHIPS; c++) {
		/* Reading from the 4th config register */
		for (int cell = 0; cell < 8; cell++) {
			if (local_config[c][4] & (1 << cell))
				return true;
		}

		/* Reading from the 5th config register */
		for (int cell = 0; cell < 4; cell++) {
			if (local_config[c][5] & (1 << (cell)))
				return true;
		}
	}

	return false;
}

void pull_chip_configuration()
{
	uint8_t remote_config[NUM_CHIPS][8];
	// LTC6804_rdcfg(ltc68041, NUM_CHIPS, remote_config);

	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int index = 0; index < 6; index++) {
			local_config[chip][index] = remote_config[chip][index];
		}
	}
}

int8_t steinhart_est(uint16_t V)
{
	/* min temp - max temp with buffer on both */
	for (int i = -25; i < 80; i++) {
		if (V > VOLT_TEMP_CONV[i + 25]) {
			return i;
		}
	}

	return 80;
}
void disable_gpio_pulldowns()
{
	HAL_Delay(1000);
	/* Turn OFF GPIO 1 & 2 pull downs */
	pull_chip_configuration();
	for (int c = 0; c < NUM_CHIPS; c++) {
		local_config[c][0] |= 0x18;
	}
	// push_chip_configuration();

	pull_chip_configuration();
	printf("Chip CFG:\n");
	for (int c = 0; c < NUM_CHIPS; c++) {
		for (int byte = 0; byte < 6; byte++) {
			printf("%x", local_config[c][byte]);
			printf("\t");
		}
		printf("\n");
	}
	printf("Done\n");
}

void serialize_i2c_msg(uint8_t data_to_write[][3], uint8_t comm_output[][6])
{
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		comm_output[chip][0] = 0x60 | (data_to_write[chip][0] >>
					       4); /* START + high side of B0 */
		comm_output[chip][1] = (data_to_write[chip][0] << 4) |
				       0x00; /* low side of B0 + ACK */
		comm_output[chip][2] = 0x00 | (data_to_write[chip][1] >>
					       4); /* BLANK + high side of B1 */
		comm_output[chip][3] = (data_to_write[chip][1] << 4) |
				       0x00; /* low side of B1 + ACK */
		comm_output[chip][4] = 0x00 | (data_to_write[chip][2] >>
					       4); /* BLANK + high side of B2 */
		comm_output[chip][5] = (data_to_write[chip][2] << 4) |
				       0x09; /* low side of B2 + STOP & NACK */
	}
}

void averaging_therm_check()
{
	for (int therm = 1; therm <= 16; therm++) {
		for (int c = 0; c < NUM_CHIPS; c++) {
			/* Directly update for a set time from start up due to therm voltages
       * needing to settle */
			if (therm_avg_counter < THERM_AVG * 10) {
				segment_data[c].thermistor_value[therm - 1] =
					segment_data[c]
						.thermistor_reading[therm - 1];
				segment_data[c].thermistor_value[therm + 15] =
					segment_data[c]
						.thermistor_reading[therm + 15];
				therm_avg_counter++;
			} else {
				/* We need to investigate this. Very sloppy */
				/* Discard if reading is 33C */
				if (segment_data[c]
					    .thermistor_reading[therm - 1] !=
				    33) {
					/* If measured value is larger than current "averaged" value,
           * increment value */
					if (segment_data[c]
						    .thermistor_reading[therm -
									1] >
					    segment_data[c]
						    .thermistor_value[therm -
								      1]) {
						segment_data[c]
							.thermistor_value[therm -
									  1]++;
						/* If measured value is smaller than current "averaged" value,
             * decrement value */
					} else if (segment_data[c]
							   .thermistor_reading
								   [therm - 1] <
						   segment_data[c]
							   .thermistor_value
								   [therm - 1]) {
						segment_data[c]
							.thermistor_value[therm -
									  1]--;
					}
				}

				/* See comments above. Identical but for the upper 16 therms */
				if (segment_data[c]
					    .thermistor_reading[therm + 15] !=
				    33) {
					if (segment_data[c]
						    .thermistor_reading[therm +
									15] >
					    segment_data[c]
						    .thermistor_value[therm +
								      15]) {
						segment_data[c]
							.thermistor_value[therm +
									  15]++;
					} else if (segment_data[c]
							   .thermistor_reading
								   [therm + 15] <
						   segment_data[c].thermistor_value
							   [therm + 15]) {
						segment_data[c]
							.thermistor_value[therm +
									  15]--;
					}
				}
			}
		}
	}
}

void standard_dev_therm_check()
{
	if (previous_data == NULL)
		return;
	int16_t avg_temp = calc_average();
	uint8_t standard_dev = calc_therm_standard_dev(avg_temp);
	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
		for (uint8_t therm = 17; therm < 28; therm++) {
			/*
       * If difference between thermistor and average is more than
       * MAX_STANDARD_DEV set the therm to pack average
       */
			if (abs(segment_data[c].thermistor_value[therm] -
				avg_temp) > (MAX_STANDARD_DEV * standard_dev)) {
				/* Nullify thermistor by setting to pack average */
				segment_data[c].thermistor_value[therm] =
					previous_data[c].thermistor_value[therm];
			}
		}
	}
}

int8_t calc_therm_standard_dev(int16_t avg_temp)
{
	uint16_t sum_diff_sqrd = 0;
	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
		for (uint8_t therm = 17; therm < 28; therm++) {
			uint16_t sum_diff =
				abs(segment_data[chip].thermistor_value[therm] -
				    avg_temp);
			sum_diff_sqrd += sum_diff * sum_diff;
		}
	}

	uint8_t standard_dev = sqrt(sum_diff_sqrd / 88);
	if (standard_dev < 8) {
		standard_dev = 8;
	}
	return standard_dev;
}

int16_t calc_average()
{
	int16_t avg = 0;
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int therm = 17; therm < 28; therm++) {
			avg += segment_data[chip].thermistor_value[therm];
		}
	}

	avg = avg / (NUM_CHIPS * 11);
	return avg;
}

void variance_therm_check()
{
	if (previous_data == NULL) {
		start_timer(&variance_timer, 1000);
		return;
	}

	if (is_timer_expired(&variance_timer)) {
		for (uint8_t c = 0; c < NUM_CHIPS; c++) {
			for (uint8_t therm = 17; therm < 28; therm++) {
				if (abs(segment_data[c]
						.thermistor_reading[therm] -
					previous_data[c]
						.thermistor_reading[therm]) >
					    5 &&
				    (segment_data[c].thermistor_reading[therm] <
					     10 ||
				     segment_data[c].thermistor_reading[therm] >
					     30)) {
					segment_data[c]
						.thermistor_reading[therm] =
						previous_data[c]
							.thermistor_reading
								[therm];
					segment_data[c].thermistor_value[therm] =
						previous_data[c]
							.thermistor_value[therm];
				}
			}
		}
	}
}

void discard_neutrals()
{
	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
		for (uint8_t therm = 17; therm < 28; therm++) {
			if (segment_data[c].thermistor_reading[therm] == 33) {
				segment_data[c].thermistor_reading[therm] = 25;
				segment_data[c].thermistor_value[therm] = 25;
			}
		}
	}
}
