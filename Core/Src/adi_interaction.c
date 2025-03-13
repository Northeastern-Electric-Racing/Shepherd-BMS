#include "adi_interaction.h"
#include "adBms6830CmdList.h"
#include "adBms6830GenericType.h"
#include "mcuWrapper.h"
#include "can_messages.h"

/**
 * @brief Count and reset PEC errors for all chips, then send a CAN message if needed.
 *
 * This function iterates through all chips, accumulates the PEC (Packet Error Code) 
 * error count, resets the PEC error counter and Command counter, then sends a CAN message if any errors exist.
 *
 * @param chips Array of chips containing PEC error data.
 */
static void count_pec_errors(cell_asic chips[NUM_CHIPS])
{
	for (uint8_t chip = 0U; chip < NUM_CHIPS; chip++) {
		uint16_t pec_error_count =
			(uint16_t)(chips[chip].cccrc.cfgr_pec +
				   chips[chip].cccrc.cell_pec +
				   chips[chip].cccrc.acell_pec +
				   chips[chip].cccrc.scell_pec +
				   chips[chip].cccrc.fcell_pec +
				   chips[chip].cccrc.aux_pec +
				   chips[chip].cccrc.raux_pec +
				   chips[chip].cccrc.stat_pec +
				   chips[chip].cccrc.comm_pec +
				   chips[chip].cccrc.pwm_pec +
				   chips[chip].cccrc.sid_pec);

		if (pec_error_count > 0) {
			//printf("PEC Error: Chip %u, Count: %u\n", chip + 1, pec_error_count);

			send_pec_error_message(chip + 1, pec_error_count);
		}

		memset(&(chips[chip].cccrc), 0, sizeof(chips[chip].cccrc));
	}
}

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

// --- BEGIN SET HELPERS ---

void set_REFON(cell_asic *chip, REFON state)
{
	chip->tx_cfga.refon = state;
}

void set_volt_adc_comp_thresh(cell_asic *chip, CTH threshold)
{
	chip->tx_cfga.cth = threshold;
}

void set_diagnostic_flags(cell_asic *chip, FLAG_D config)
{
	chip->tx_cfga.flag_d =
		(uint8_t)set_uint16_bit(chip->tx_cfga.flag_d, config, true);
}

void set_cell_discharge(cell_asic *chip, uint8_t cell, bool discharge)
{
	chip->tx_cfgb.dcc = set_uint16_bit(chip->tx_cfgb.dcc, cell, discharge);
}

void set_soak_on(cell_asic *chip, SOAKON state)
{
	chip->tx_cfga.soakon = state;
}

void set_aux_soak_range(cell_asic *chip, OWRNG range)
{
	chip->tx_cfga.owrng = range;
}

void set_open_wire_soak_time(cell_asic *chip, OWA time)
{
	chip->tx_cfga.owa = time;
}

void set_gpio_pull(cell_asic *chip, uint8_t gpio, bool input)
{
	if (gpio > 10 || gpio < 1) {
		printf("ERROR: Invalid GPIO pin %d\n", gpio);
		return;
	}
	chip->tx_cfga.gpo = set_uint16_bit(chip->tx_cfga.gpo, gpio - 1, input);
}

void set_iir_corner_freq(cell_asic *chip, IIR_FPA freq)
{
	chip->tx_cfga.fc = freq;
}

void set_comm_break(cell_asic *chip, bool is_break)
{
	chip->tx_cfga.comm_bk = is_break;
}

void set_mute_state(cell_asic *chip, bool disable_discharge)
{
	chip->tx_cfga.mute_st = disable_discharge;
}

void set_snapshot(cell_asic *chip, bool take_snapshot)
{
	chip->tx_cfga.snap = take_snapshot;
}

void set_discharge_timer_monitor(cell_asic *chip, bool enabled)
{
	chip->tx_cfgb.dtmen = enabled;
}

void set_discharge_timer_range(cell_asic *chip, bool large)
{
	chip->tx_cfgb.dtrng = large;
}

void set_discharge_timeout(cell_asic *chip, uint8_t timeout)
{
	if (timeout >> 6 > 0) {
		printf("Invalid discharge time\n");
		return;
		// TODO: Non-critical fault
	}
	chip->tx_cfgb.dcto = timeout;
}

// --- END SET HELPERS ---

// void start_cell_voltages_adc(cell_asic chips[NUM_CHIPS])
// {
// 	adbms_wake_isospi();
// 	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
// 	adBmsPollAdc(PLCADC);
// }

// --- BEGIN RW ---

extern TIM_HandleTypeDef htim2;

/**
 * @brief Delays a certain number of microseconds
 * 
 * Approximately +50% error as seen in logic analyzer
 * 
 * @param us the number of us to delay
 */
inline void delay_us(uint16_t us)
{
	uint16_t tickstart = __HAL_TIM_GET_COUNTER(&htim2);
	uint16_t wait = us;

	while ((__HAL_TIM_GET_COUNTER(&htim2) - tickstart) < wait) {
	}
}

/**
 * @brief Wake the isoSPI of every ADBMS6830 IC in the daisy chain. Blocking wait for around 30us * NUM_CHIPS.
 * 
 */
void adbms_wake_isospi()
{
	for (uint8_t ic = 0; ic < NUM_CHIPS; ic++) {
		adBmsCsLow();
		adBmsCsHigh();
		delay_us(20);
	}
}

/**
 * @brief Wake the chip of every ADBMS6830 IC.  Blocking wait about 1ms * NUM_CHIPS
 * 
 */
void adbms_wake_core()
{
	for (uint8_t ic = 0; ic < NUM_CHIPS; ic++) {
		adBmsCsLow();
		adBmsCsHigh();
		delay_us(1000);
	}
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
	adbms_wake_isospi();

	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
		adBmsWriteData(NUM_CHIPS, &chips[chip], command, type, group);
	}
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
	adbms_wake_isospi();

	for (uint8_t chip = 0; chip < NUM_CHIPS; chip++) {
		adBmsReadData(NUM_CHIPS, &chips[chip], command, type, group);
	}

	count_pec_errors(chips);
}

// --- BEGIN WRITE COMMANDS ---

void soft_reset_chips(cell_asic chips[NUM_CHIPS])
{
	write_adbms_data(chips, SRST, Comm, NONE);
	adbms_wake_core();
}

void write_config_regs(cell_asic chips[NUM_CHIPS])
{
	write_adbms_data(chips, WRCFGA, Config, A);
	write_adbms_data(chips, WRCFGB, Config, B);
}

void write_clear_flags(cell_asic chips[NUM_CHIPS])
{
	for (int chip = 0; chip < NUM_CHIPS; chip++) {
		chips[chip].clflag.cl_sleep = 1;
		chips[chip].clflag.cl_smed = 1;
		chips[chip].clflag.cl_sed = 1;
		chips[chip].clflag.cl_cmed = 1;
		chips[chip].clflag.cl_ced = 1;
		chips[chip].clflag.cl_vduv = 1;
		chips[chip].clflag.cl_vdov = 1;
		chips[chip].clflag.cl_vauv = 1;
		chips[chip].clflag.cl_vaov = 1;
		chips[chip].clflag.cl_oscchk = 1;
		chips[chip].clflag.cl_tmode = 1;
		chips[chip].clflag.cl_thsd = 1;
		chips[chip].clflag.cl_sleep = 1;
		chips[chip].clflag.cl_spiflt = 1;
		chips[chip].clflag.cl_vdel = 1;
		chips[chip].clflag.cl_vde = 1;
	}
	write_adbms_data(chips, CLRFLAG, Clrflag, NONE);
}

// --- END WRITE COMMANDS

// --- BEGIN READ COMMANDS ---

void read_filtered_voltage_registers(cell_asic chips[NUM_CHIPS])
{
	read_adbms_data(chips, RDFCALL, Rdfcall, ALL_GRP);
}

void adc_and_read_aux_registers(cell_asic chips[NUM_CHIPS])
{
	// TODO only poll correct GPIOs
	adbms_wake_isospi();
	adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
	adBmsPollAdc(PLAUX1);

	read_adbms_data(chips, RDAUXA, Aux, A);
	read_adbms_data(chips, RDAUXB, Aux, B);
	read_adbms_data(chips, RDAUXC, Aux, C);
	read_adbms_data(chips, RDAUXD, Aux, D);
}

void adc_and_read_aux2_registers(cell_asic chips[NUM_CHIPS])
{
	adbms_wake_isospi();
	adBms6830_Adax2(AUX_ALL);
	adBmsPollAdc(PLAUX2);

	read_adbms_data(chips, RDRAXA, RAux, A);
	read_adbms_data(chips, RDRAXB, RAux, B);
	read_adbms_data(chips, RDRAXC, RAux, C);
	read_adbms_data(chips, RDRAXD, RAux, D);
}

void read_status_registers(cell_asic chips[NUM_CHIPS])
{
	read_adbms_data(chips, RDSTATA, Status, A);
	read_adbms_data(chips, RDSTATB, Status, B);
	read_adbms_data(chips, RDSTATC, Status, C);
	read_adbms_data(chips, RDSTATD, Status, D);
	read_adbms_data(chips, RDSTATE, Status, E);
}

void read_status_register_c(cell_asic chips[NUM_CHIPS])
{
	read_adbms_data(chips, RDSTATC, Status, C);
}

void read_status_aux_registers(cell_asic chips[NUM_CHIPS])
{
	read_adbms_data(chips, RDASALL, Rdasall, ALL_GRP);
}

void read_serial_id(cell_asic chips[NUM_CHIPS])
{
	read_adbms_data(chips, RDSID, Sid, NONE);
}

// --- END READ COMMANDS ---

// --- BEGIN ADC POLL ---

void get_c_adc_voltages(cell_asic chips[NUM_CHIPS])
{
	adbms_wake_isospi();
	// Take single shot measurement
	adBms6830_Adcv(RD_OFF, SINGLE, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);
	read_adbms_data(chips, RDCVALL, Rdcvall, ALL_GRP);
}

void get_s_adc_voltages(cell_asic chips[NUM_CHIPS])
{
	write_config_regs(chips);
	adbms_wake_isospi();
	adBms6830_Adsv(SINGLE, DCP_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLSADC);

	adbms_wake_isospi();
	read_adbms_data(chips, RDSALL, Rdsall, ALL_GRP);
	// read_adbms_data(chip, RDSVA, S_volt, A);
	// read_adbms_data(chip, RDSVB, S_volt, B);
	// read_adbms_data(chip, RDSVC, S_volt, C);
	// read_adbms_data(chip, RDSVD, S_volt, D);
	// read_adbms_data(chip, RDSVE, S_volt, E);
	// read_adbms_data(chip, RDSVF, S_volt, F);
}

void get_avgd_cell_voltages(cell_asic chips[NUM_CHIPS])
{
	adbms_wake_isospi();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);

	adbms_wake_isospi();
	read_adbms_data(chips, RDACALL, Rdacall, ALL_GRP);
}

void get_filtered_cell_voltages(cell_asic chips[NUM_CHIPS])
{
	adbms_wake_isospi();
	read_adbms_data(chips, RDFCALL, Rdfcall, ALL_GRP);
}

void get_c_and_s_adc_voltages(cell_asic chips[NUM_CHIPS])
{
	adbms_wake_isospi();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc(PLCADC);

	adbms_wake_isospi();
	read_adbms_data(chips, RDCSALL, Rdcsall, ALL_GRP);
}

void start_c_adc_conv()
{
	adbms_wake_isospi();
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_ON, OW_OFF_ALL_CH);
}

// --- END ADC POLL ---
