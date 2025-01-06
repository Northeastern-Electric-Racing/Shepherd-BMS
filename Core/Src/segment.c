#include "segment.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

#define THERM_WAIT_TIME	   500 /* ms */
#define VOLTAGE_WAIT_TIME  500 /* ms */
#define THERM_AVG	   15 /* Number of values to average */
#define MAX_VOLT_DELTA	   2500
#define MAX_CONSEC_NOISE   10
#define GPIO_EXPANDER_ADDR 0x40
#define GPIO_REGISTER_ADDR 0x09

// TODO ensure spi 1 is correct for talking to segs
extern SPI_HandleTypeDef hspi1;

uint8_t therm_avg_counter = 0;

chipdata_t previous_data[NUM_CHIPS] = {};
uint16_t discharge_commands[NUM_CHIPS] = {};

nertimer_t therm_timer;
nertimer_t voltage_reading_timer;
nertimer_t variance_timer;

int therm_error = 0; // not faulted
uint16_t crc_error_check = 0;

/* our segments are mapped backwards and in pairs, so they are read in 1,0 then
 * 3,2, etc*/
const int mapping_correction[12] = { 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10 };

uint16_t therm_settle_time_ = 0;

// TODO: replace for new thermistors
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
int8_t steinhart_est(uint16_t V);
void variance_therm_check(void);
void discard_neutrals(chipdata_t segment_data[NUM_CHIPS]);
void pull_chip_configuration(void);
int16_t calc_average(chipdata_t segment_data[NUM_CHIPS]);
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

/**
 * @brief Wake every ADBMS6830 IC in the daisy chain. Takes NUM_CHIPS * 8 ms to finish.
 * 
 */
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

	// If the corresponding fault bits are sent high, it does not affect the IC
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
 * @param cell ID of cell to modify. Cell indexes start are from 1-16 (NOT ZERO INDEXED).
 * @param discharge Cell discharge state. true to discharge, false to disable discharge.
 */
void set_cell_discharge(cell_asic *chip, uint8_t cell, bool discharge)
{
	chip->tx_cfgb.dcc = set_uint16_bit(chip->tx_cfgb.dcc, cell, discharge);
}

/**
 * @brief Set the state of the SOAKON bit to either enable or disable soak times.
 * 
 * @param chip Pointer to chip to configure
 * @param state Enable or disable SOAKON
 */
void set_soak_on(cell_asic *chip, SOAKON state)
{
	chip->tx_cfga.soakon = state;
}

/**
 * @brief Set the open wire detection soak time range.
 * 
 * @param chip Pointer to chip to configure
 * @param range The range of time over which to soak for open wire detection
 */
void set_open_wire_soak_range(cell_asic *chip, OWRNG range)
{
	chip->tx_cfga.owrng = range;
}

/**
 * @brief Set the open wire soak time. See data sheet for formula.
 * 
 * @param chip Pointer to chip configuration
 * @param time The amount of time to soak for. Higher OWA is a higher soak time.
 */
void set_open_wire_soak_time(cell_asic *chip, OWA time)
{
	chip->tx_cfga.owa = time;
}

/**
 * @brief Set the mode of a GPIO pin on an ADBMS8630.
 * 
 * @param chip ADBMS6830 chip
 * @param gpio Number of the GPIO pin to change (1-10)
 * @param mode Whether the pin should be an input or an output. True is input, False is output.
 */
void set_gpio_mode(cell_asic *chip, uint8_t gpio, bool mode)
{
	if (gpio > 10 || gpio < 1) {
		printf("ERROR: Invalid GPIO pin %d\n", gpio);
		return;
	}
	chip->tx_cfga.gpo = set_uint16_bit(chip->tx_cfga.gpo, gpio - 1, mode);
}

/**
 * @brief Set the corner frequency of the IIR filter.
 * 
 * @param chip Pointer to chip config
 * @param freq Corner frequency (see IIR_FPA enum for frequencies)
 */
void set_iir_corner_freq(cell_asic *chip, IIR_FPA freq)
{
	chip->tx_cfga.fc = freq;
}

/**
 * @brief Configure a chip as a break in the isoSPI daisy chain.
 * 
 * @param chip Pointer to chip config
 * @param is_break True if chip is break, false if chip is not break
 */
void set_comm_break(cell_asic *chip, bool is_break)
{
	chip->tx_cfga.comm_bk = is_break;
}

/**
 * @brief Set whether or not this chip is taking a snapshot. The chip will not begin reading new values unless the snapshot bit is cleared.
 * 
 * @param chip Pointer to chip config
 * @param take_snapshot True to take a snapshot, false to end the snapshot
 */
void set_snapshot(cell_asic *chip, bool take_snapshot)
{
	chip->tx_cfga.snap = take_snapshot;
}

/**
 * @brief Enable/disable the discharge timer monitor.
 * 
 * @param chip Pointer to chip config
 * @param enabled True if discharge timer monitor is enabled, false if otherwise
 */
void set_discharge_timer_monitor(cell_asic *chip, bool enabled)
{
	chip->tx_cfgb.dtmen = enabled;
}

/**
 * @brief Configure the discharge timer range, which affects the resolution.
 * 
 * @param chip Pointer to chip config
 * @param large True for large range, False for small range
 */
void set_discharge_timer_range(cell_asic *chip, bool large)
{
	chip->tx_cfgb.dtrng = large;
}

/**
 * @brief Set the discharge monitor timeout, which is dependent on the discharge timer range.
 * 
 * @param chip Pointer to chip config
 * @param timeout Base for timeout multiplicaiton. Must be below six bits.
 */
void set_discharge_timeout(cell_asic *chip, uint8_t timeout)
{
	if (timeout >> 6 > 0) {
		printf("Invalid discharge time\n");
		return;
		//TODO: Non-critical fault
	}
	chip->tx_cfgb.dcto = timeout;
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
void segment_init(acc_data_t *bmsdata)
{
	printf("Initializing Segments...");
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		init_chip(&bmsdata->chips[chip]);
		// TODO: Make sure this is accurate
		bmsdata->chip_data->alpha = chip % 2 == 0;
	}
	write_config_regs(bmsdata->chips);
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
void get_filtered_cell_voltages(cell_asic chips[NUM_CHIPS])
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

void pull_voltages(acc_data_t *bmsdata)
{
	/**
   * If we haven't waited long enough between pulling voltage data
   * just copy over the contents of the last good reading and the fault status
   * from the most recent attempt
   */

	// TODO: should probably get rid of this
	if (!is_timer_expired(&voltage_reading_timer) &&
	    voltage_reading_timer.active) {
		for (uint8_t i = 0; i < NUM_CHIPS; i++) {
			memcpy(&bmsdata->chips[i], &previous_data[i],
			       sizeof(bmsdata->chips[i]));
		}
	}

	get_c_adc_voltages(bmsdata->chips);

	/*

	OLD CODE THAT DID WORK FOR ADBMS. KEEPING IN FOR DEBUGGING.

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
		adBms6830_Adax(AUX_OW_OFF,
			       PUP_DOWN, AUX_ALL);
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
		adBms6830_Adax2(AUX_ALL);
		adBmsPollAdc(PLAUX2);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXA, RAux, A);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXB, RAux, B);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXC, RAux, C);
		adBmsReadData(TOTAL_IC, &IC[0], RDRAXD, RAux, D);
		printVoltages(TOTAL_IC, &IC[0], RAux);
	}	*/

	/*
	if (MEASURE_STAT == ENABLED) {
		adBms6830_Adax(AUX_OW_OFF,
			       PUP_DOWN, AUX_ALL);
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
}

void segment_retrieve_data(acc_data_t *bmsdata)
{
	get_c_adc_voltages(bmsdata->chips);

	// get_s_adc_voltages(bmsdata->chips);

	// The GPIOs in the AUX registers contain voltage readings from the therms.
	read_aux_registers(bmsdata->chips);
	// If you want redundant Thermistor readings, uncomment the following.
	read_aux2_registers(bmsdata->chips);

	/* Save the contents of the reading so that we can use it to fill in missing
   * data */
	memcpy(previous_data, bmsdata->chip_data,
	       sizeof(chipdata_t) * NUM_CHIPS);
}

bool segment_is_balancing(cell_asic chips[NUM_CHIPS])
{
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		if (chips[chip].tx_cfga.mute_st !=
		    MUTE_ACTIVATED_DISCHARGE_DISABLED) {
			return true;
		}
	}
	return false;
}

void segment_disable_balancing(cell_asic chips[NUM_CHIPS])
{
	// Initializes all array elements to zero
	bool discharge_config[NUM_CHIPS][NUM_CELLS_PER_CHIP] = { 0 };
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		chips[chip].tx_cfga.mute_st = MUTE_ACTIVATED_DISCHARGE_DISABLED;
	}
	segment_configure_balancing(chips, discharge_config);
}

/**
 * @brief Configure which cells should discharge, and send configuration to ICs.
 * 
 * @param discharge_config Array containing the discharge configuration. true = discharge, false = do not discharge.
 */
void segment_configure_balancing(
	cell_asic chips[NUM_CHIPS],
	bool discharge_config[NUM_CHIPS][NUM_CELLS_PER_CHIP])
{
	// TODO: Test
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		for (int cell = 0; cell < NUM_CELLS_PER_CHIP; cell++) {
			set_cell_discharge(&chips[chip], cell + 1,
					   discharge_config[chip][cell]);

			// Enable balancing for a chip if a cell is to be discharged
			if (chips[chip].tx_cfga.mute_st ==
				    MUTE_ACTIVATED_DISCHARGE_DISABLED &&
			    discharge_config[chip][cell]) {
				chips[chip].tx_cfga.mute_st = DISCHARGE_ENABLED;
			}
		}
	}
	write_config_regs(chips);
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

// void averaging_therm_check(chipdata_t segment_data[NUM_CHIPS])
// {
// 	for (int therm = 1; therm <= 16; therm++) {
// 		for (int c = 0; c < NUM_CHIPS; c++) {
// 			/* Directly update for a set time from start up due to therm voltages
//        * needing to settle */
// 			if (therm_avg_counter < THERM_AVG * 10) {
// 				segment_data[c].thermistor_value[therm - 1] =
// 					segment_data[c]
// 						.thermistor_reading[therm - 1];
// 				segment_data[c].thermistor_value[therm + 15] =
// 					segment_data[c]
// 						.thermistor_reading[therm + 15];
// 				therm_avg_counter++;
// 			} else {
// 				/* We need to investigate this. Very sloppy */
// 				/* Discard if reading is 33C */
// 				if (segment_data[c]
// 					    .thermistor_reading[therm - 1] !=
// 				    33) {
// 					/* If measured value is larger than current "averaged" value,
//            * increment value */
// 					if (segment_data[c]
// 						    .thermistor_reading[therm -
// 									1] >
// 					    segment_data[c]
// 						    .thermistor_value[therm -
// 								      1]) {
// 						segment_data[c]
// 							.thermistor_value[therm -
// 									  1]++;
// 						/* If measured value is smaller than current "averaged" value,
//              * decrement value */
// 					} else if (segment_data[c]
// 							   .thermistor_reading
// 								   [therm - 1] <
// 						   segment_data[c]
// 							   .thermistor_value
// 								   [therm - 1]) {
// 						segment_data[c]
// 							.thermistor_value[therm -
// 									  1]--;
// 					}
// 				}

// 				/* See comments above. Identical but for the upper 16 therms */
// 				if (segment_data[c]
// 					    .thermistor_reading[therm + 15] !=
// 				    33) {
// 					if (segment_data[c]
// 						    .thermistor_reading[therm +
// 									15] >
// 					    segment_data[c]
// 						    .thermistor_value[therm +
// 								      15]) {
// 						segment_data[c]
// 							.thermistor_value[therm +
// 									  15]++;
// 					} else if (segment_data[c]
// 							   .thermistor_reading
// 								   [therm + 15] <
// 						   segment_data[c].thermistor_value
// 							   [therm + 15]) {
// 						segment_data[c]
// 							.thermistor_value[therm +
// 									  15]--;
// 					}
// 				}
// 			}
// 		}
// 	}
// }

// void standard_dev_therm_check(chipdata_t segment_data[NUM_CHIPS])
// {
// 	if (previous_data == NULL)
// 		return;
// 	int16_t avg_temp = calc_average(segment_data);
// 	uint8_t standard_dev = calc_therm_standard_dev(avg_temp);
// 	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
// 		for (uint8_t therm = 17; therm < 28; therm++) {
// 			/*
//        * If difference between thermistor and average is more than
//        * MAX_STANDARD_DEV set the therm to pack average
//        */
// 			if (abs(segment_data[c].thermistor_value[therm] -
// 				avg_temp) > (MAX_STANDARD_DEV * standard_dev)) {
// 				/* Nullify thermistor by setting to pack average */
// 				segment_data[c].thermistor_value[therm] =
// 					previous_data[c].thermistor_value[therm];
// 			}
// 		}
// 	}
// }

// int8_t calc_therm_standard_dev(chipdata_t segment_data[NUM_CHIPS], int16_t avg_temp)
// {
// 	uint16_t sum_diff_sqrd = 0;
// 	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
// 		for (uint8_t therm = 17; therm < 28; therm++) {
// 			uint16_t sum_diff =
// 				abs(segment_data[chip].thermistor_value[therm] -
// 				    avg_temp);
// 			sum_diff_sqrd += sum_diff * sum_diff;
// 		}
// 	}

// 	uint8_t standard_dev = sqrt(sum_diff_sqrd / 88);
// 	if (standard_dev < 8) {
// 		standard_dev = 8;
// 	}
// 	return standard_dev;
// }

int16_t calc_average(chipdata_t segment_data[NUM_CHIPS])
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

// void variance_therm_check()
// {
// 	if (previous_data == NULL) {
// 		start_timer(&variance_timer, 1000);
// 		return;
// 	}

// 	if (is_timer_expired(&variance_timer)) {
// 		for (uint8_t c = 0; c < NUM_CHIPS; c++) {
// 			for (uint8_t therm = 17; therm < 28; therm++) {
// 				if (abs(segment_data[c]
// 						.thermistor_reading[therm] -
// 					previous_data[c]
// 						.thermistor_reading[therm]) >
// 					    5 &&
// 				    (segment_data[c].thermistor_reading[therm] <
// 					     10 ||
// 				     segment_data[c].thermistor_reading[therm] >
// 					     30)) {
// 					segment_data[c]
// 						.thermistor_reading[therm] =
// 						previous_data[c]
// 							.thermistor_reading
// 								[therm];
// 					segment_data[c].thermistor_value[therm] =
// 						previous_data[c]
// 							.thermistor_value[therm];
// 				}
// 			}
// 		}
// 	}
// }

// void discard_neutrals(chipdata_t segment_data[NUM_CHIPS])
// {
// 	for (uint8_t c = 0; c < NUM_CHIPS; c++) {
// 		for (uint8_t therm = 17; therm < 28; therm++) {
// 			if (segment_data[c].thermistor_reading[therm] == 33) {
// 				segment_data[c].thermistor_reading[therm] = 25;
// 				segment_data[c].thermistor_value[therm] = 25;
// 			}
// 		}
// 	}
// }
