#include "adi_interaction.h"
#include "adBms6830CmdList.h"
#include "adBms6830GenericType.h"
#include "can_messages.h"
#include "compute.h"
#include "mcuWrapper.h"

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

		// printf("1 %d\n", chips[chip].cccrc.cfgr_pec);
		// printf("b %d\n", chips[chip].cccrc.cell_pec);
		// printf("2 %d\n", chips[chip].cccrc.acell_pec);
		// printf("3 %d\n", chips[chip].cccrc.scell_pec);
		// printf("4 %d\n", chips[chip].cccrc.fcell_pec);
		// printf("5 %d\n", chips[chip].cccrc.aux_pec);
		// printf("6 %d\n", chips[chip].cccrc.raux_pec);
		// printf("7 %d\n", chips[chip].cccrc.stat_pec);
		// printf("8 %d\n", chips[chip].cccrc.comm_pec);
		// printf("9 %d\n", chips[chip].cccrc.pwm_pec);
		// printf("10 %d\n\n\n", chips[chip].cccrc.sid_pec);

		if (pec_error_count > 0) {
			printf("PEC Error: Chip %u, Count: %u\n", chip,
			       pec_error_count);

			// if only a few PEC errors happened, print which registers they came from
			if (pec_error_count < 10) {
				if (chips[chip].cccrc.cfgr_pec > 0) {
					printf("CFGR PEC %d, ",
					       chips[chip].cccrc.cfgr_pec);
				}
				if (chips[chip].cccrc.cell_pec > 0) {
					printf("CELL PEC %d, ",
					       chips[chip].cccrc.cell_pec);
				}
				if (chips[chip].cccrc.acell_pec > 0) {
					printf("ACELL PEC %d, ",
					       chips[chip].cccrc.acell_pec);
				}
				if (chips[chip].cccrc.scell_pec > 0) {
					printf("SCELL PEC %d, ",
					       chips[chip].cccrc.scell_pec);
				}
				if (chips[chip].cccrc.fcell_pec > 0) {
					printf("FCELL PEC %d, ",
					       chips[chip].cccrc.fcell_pec);
				}
				if (chips[chip].cccrc.aux_pec > 0) {
					printf("AUX PEC %d, ",
					       chips[chip].cccrc.aux_pec);
				}
				if (chips[chip].cccrc.raux_pec > 0) {
					printf("RAUX PEC %d, ",
					       chips[chip].cccrc.raux_pec);
				}
				if (chips[chip].cccrc.stat_pec > 0) {
					printf("STAT PEC %d, ",
					       chips[chip].cccrc.stat_pec);
				}
				if (chips[chip].cccrc.comm_pec > 0) {
					printf("COMM PEC %d, ",
					       chips[chip].cccrc.comm_pec);
				}
				if (chips[chip].cccrc.pwm_pec > 0) {
					printf("PWM PEC %d, ",
					       chips[chip].cccrc.pwm_pec);
				}
				if (chips[chip].cccrc.sid_pec > 0) {
					printf("SID PEC %d, ",
					       chips[chip].cccrc.sid_pec);
				}
				printf("\n");
			}

			send_pec_error_message(chip, pec_error_count);
		}

		memset(&(chips[chip].cccrc), 0, sizeof(chips[chip].cccrc));
	}
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

void set_diagnostic_flags(cell_asic *chip, FLAG_D config, CFGA_FLAG state)
{
	if (state == FLAG_SET) {
		chip->tx_cfga.flag_d |= ConfigA_Flag(config, state);
	} else {
		chip->tx_cfga.flag_d &= ~(1 << config);
	}
}
void clear_diagnostic_flags(cell_asic *chip)
{
	chip->tx_cfga.flag_d = 0;
}

void set_cell_discharge(cell_asic *chip, DCC cell, DCC_BIT discharge)
{
	if (discharge == DCC_BIT_SET) {
		chip->tx_cfgb.dcc |= ConfigB_DccBit(cell, discharge);
	} else {
		chip->tx_cfgb.dcc &= ~(1 << cell);
	}
}

void clear_cell_discharge(cell_asic *chip)
{
	chip->tx_cfgb.dcc = 0;
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

void set_gpio_pull(cell_asic *chip, GPO gpio, CFGA_GPO input)
{
	if (input == GPO_SET) {
		chip->tx_cfga.gpo |= ConfigA_Gpo(gpio, input);
	} else {
		chip->tx_cfga.gpo &= ~(1 << gpio);
	}
}

void set_iir_corner_freq(cell_asic *chip, IIR_FPA freq)
{
	chip->tx_cfga.fc = freq;
}

void set_comm_break(cell_asic *chip, COMM_BK is_break)
{
	chip->tx_cfga.comm_bk = is_break;
}

void set_discharge_timer_monitor(cell_asic *chip, DTMEN enabled)
{
	chip->tx_cfgb.dtmen = enabled;
}

void set_discharge_timer_range(cell_asic *chip, DTRNG range)
{
	chip->tx_cfgb.dtrng = range;
}

void set_discharge_timeout(cell_asic *chip, DCTO timeout)
{
	chip->tx_cfgb.dcto = timeout;
}

// --- END SET HELPERS ---

// void start_cell_voltages_adc(cell_asic chips[NUM_CHIPS])
// {
// 	adbms_wake_isospi();
// 	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
// 	adBmsPollAdc_indicator(PLCADC);
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
inline void delay_us(uint32_t us)
{
	uint32_t tickstart = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t wait = us;

	while ((__HAL_TIM_GET_COUNTER(&htim2) - tickstart) < wait) {
	}
}

/**
 * @brief Wake the isoSPI of every ADBMS6830 IC in the daisy chain. Blocking critical section wait for around 30us * NUM_CHIPS.
 * 
 * Takes in the SPI object as an onwership strategy, it is externed inside the driver
 * 
 */
void adbms_wake_isospi(SPI_HandleTypeDef *hspi)
{
	for (uint8_t ic = 0; ic < NUM_CHIPS; ic++) {
		adBmsCsLow();
		adBmsCsHigh();
		delay_us(500);
	}
}

/**
 * @brief Wake the chip of every ADBMS6830 IC.  Blocking critical section wait about 1ms * NUM_CHIPS
 * 
 */
void adbms_wake_core()
{
	for (uint8_t ic = 0; ic < NUM_CHIPS; ic++) {
		adBmsCsLow();
		delay_us(1000);
		adBmsCsHigh();
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
		      GRP group, SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);

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
		     GRP group, SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);

	adBmsReadData(NUM_CHIPS, chips, command, type, group);

	count_pec_errors(chips);
}

uint32_t adBmsPollAdc_indicator(uint8_t poll_type[2])
{
	set_debug_led_2(1);
	uint32_t result = adBmsPollAdc(poll_type);
	set_debug_led_2(0);
	return result;
}

// --- BEGIN WRITE COMMANDS ---

void soft_reset_chips(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	spiSendCmd(SRST);
	adbms_wake_core();
}

void mute_chips(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	spiSendCmd(MUTE);
}
void unmute_chips(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	spiSendCmd(UNMUTE);
}

void snap_chips(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	spiSendCmd(SNAP);
}

void unsnap_chips(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
  spiSendCmd(UNSNAP);
}

void write_config_regs(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	write_adbms_data(chips, WRCFGA, Config, A, hspi);
	write_adbms_data(chips, WRCFGB, Config, B, hspi);
}

void write_clear_flags(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
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
	write_adbms_data(chips, CLRFLAG, Clrflag, NONE, hspi);
}

// --- END WRITE COMMANDS

// --- BEGIN READ COMMANDS ---

void read_c_voltage_registers(cell_asic chips[NUM_CHIPS],
			      SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDCVA, Cell, A, hspi);
	read_adbms_data(chips, RDCVB, Cell, B, hspi);
	read_adbms_data(chips, RDCVC, Cell, C, hspi);
	read_adbms_data(chips, RDCVD, Cell, D, hspi);
	read_adbms_data(chips, RDCVE, Cell, E, hspi);
}

void read_average_voltage_registers(cell_asic chips[NUM_CHIPS],
				    SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDACA, AvgCell, A, hspi);
	read_adbms_data(chips, RDACB, AvgCell, B, hspi);
	read_adbms_data(chips, RDACC, AvgCell, C, hspi);
	read_adbms_data(chips, RDACD, AvgCell, D, hspi);
	read_adbms_data(chips, RDACE, AvgCell, E, hspi);
}

void read_filtered_voltage_registers(cell_asic chips[NUM_CHIPS],
				     SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDFCA, F_volt, A, hspi);
	read_adbms_data(chips, RDFCB, F_volt, B, hspi);
	read_adbms_data(chips, RDFCC, F_volt, C, hspi);
	read_adbms_data(chips, RDFCD, F_volt, D, hspi);
	read_adbms_data(chips, RDFCE, F_volt, E, hspi);
}

void read_s_voltage_registers(cell_asic chips[NUM_CHIPS],
			      SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDSVA, S_volt, A, hspi);
	read_adbms_data(chips, RDSVB, S_volt, B, hspi);
	read_adbms_data(chips, RDSVC, S_volt, C, hspi);
	read_adbms_data(chips, RDSVD, S_volt, D, hspi);
	read_adbms_data(chips, RDSVE, S_volt, E, hspi);
}

void adc_and_read_aux_registers(cell_asic chips[NUM_CHIPS],
				SPI_HandleTypeDef *hspi)
{
	// TODO only poll correct GPIOs
	adbms_wake_isospi(hspi);
	adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
	adBmsPollAdc_indicator(PLAUX1);

	read_adbms_data(chips, RDAUXA, Aux, A, hspi);
	read_adbms_data(chips, RDAUXB, Aux, B, hspi);
	read_adbms_data(chips, RDAUXC, Aux, C, hspi);
	read_adbms_data(chips, RDAUXD, Aux, D, hspi);
}

void adc_and_read_aux2_registers(cell_asic chips[NUM_CHIPS],
				 SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	adBms6830_Adax2(AUX_ALL);
	adBmsPollAdc_indicator(PLAUX2);

	read_adbms_data(chips, RDRAXA, RAux, A, hspi);
	read_adbms_data(chips, RDRAXB, RAux, B, hspi);
	read_adbms_data(chips, RDRAXC, RAux, C, hspi);
	read_adbms_data(chips, RDRAXD, RAux, D, hspi);
}

void read_status_registers(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDSTATA, Status, A, hspi);
	read_adbms_data(chips, RDSTATB, Status, B, hspi);
	read_adbms_data(chips, RDSTATC, Status, C, hspi);
	read_adbms_data(chips, RDSTATD, Status, D, hspi);
	read_adbms_data(chips, RDSTATE, Status, E, hspi);
}

void read_status_register_c(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDSTATC, Status, C, hspi);
}

void read_config_register_a(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDCFGA, Config, A, hspi);
}

void read_config_register_b(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDCFGB, Config, B, hspi);
}

void read_pwm_registers(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDPWM1, Pwm, A, hspi);
	read_adbms_data(chips, RDPWM2, Pwm, B, hspi);
}

void read_status_aux_registers(cell_asic chips[NUM_CHIPS],
			       SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDASALL, Rdasall, ALL_GRP, hspi);
}

void read_serial_id(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	read_adbms_data(chips, RDSID, Sid, NONE, hspi);
}

// --- END READ COMMANDS ---

// --- BEGIN ADC POLL ---

void get_c_adc_voltages(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	adBms6830_Adcv(RD_OFF, SINGLE, DCP_OFF, RSTF_ON, OW_OFF_ALL_CH);
	adBmsPollAdc_indicator(PLCADC);

	read_c_voltage_registers(chips, hspi);
}

void get_avgd_cell_voltages(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	adBms6830_Adcv(RD_OFF, SINGLE, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc_indicator(PLCADC);

	read_average_voltage_registers(chips, hspi);
}

// useless, as a filter needs to be continous
// void get_filtered_cell_voltages(cell_asic chips[NUM_CHIPS])
// {
// 	adbms_wake_isospi();
// 	read_adbms_data(chips, RDFCALL, Rdfcall, ALL_GRP);
// }

void get_s_adc_voltages(cell_asic chips[NUM_CHIPS], SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	adBms6830_Adsv(SINGLE, DCP_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc_indicator(PLSADC);

	read_s_voltage_registers(chips, hspi);
}

void get_c_and_s_adc_voltages(cell_asic chips[NUM_CHIPS],
			      SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	adBms6830_Adcv(RD_ON, SINGLE, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
	adBmsPollAdc_indicator(PLSADC);

	read_c_voltage_registers(chips, hspi);
	read_s_voltage_registers(chips, hspi);
}

void start_c_adc_conv(SPI_HandleTypeDef *hspi)
{
	adbms_wake_isospi(hspi);
	adBms6830_Adcv(RD_ON, CONTINUOUS, DCP_OFF, RSTF_ON, OW_OFF_ALL_CH);
}

// --- END ADC POLL ---
