#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

#define DEBUG_MODE_ENABLED true
#define DEBUG_STATS

// Hardware definition
#define NUM_SEGMENTS	5
#define NUM_CHIPS	NUM_SEGMENTS * 2
#define NUM_CELLS_ALPHA 14
#define NUM_CELLS_BETA	11
#define NUM_CELLS_SEG	NUM_CELLS_ALPHA + NUM_CELLS_BETA
#define NUM_CELLS                                \
	((NUM_CELLS_ALPHA * (NUM_CHIPS / 2.0)) + \
	 (NUM_CELLS_BETA * (NUM_CHIPS / 2.0)))
// only actual flexPCB therms counted
#define NUM_THERMS_ALPHA 7
#define NUM_THERMS_BETA	 6

// Firmware limits
#define MAX_TEMP    60 /* Celsius */
#define MIN_TEMP    -40 /* Celsius */
#define MAX_DELTA_V 0.010
#define BAL_MIN_V   4.00

/* Molicel P45B Cell Specifications */
#define TYP_CAPICITY_AH	    4.5 /* Amp-hours */
#define TYP_CAPACITY_WH	    16.2 /* Watt-hours */
#define MIN_CAPICITY_AH	    4.3 /* Amp-hours */
#define MIN_CAPACITY_WH	    15.5 /* Watt-hours */
#define MIN_VOLT	    2.5
#define NOM_VOLT	    3.6
#define MAX_VOLT	    4.2
#define MAX_CHARGE_VOLT	    4.19 // LOADED FAULT
#define MAX_CHARGE_VOLT_FLT 4.25 // LOADED FAULT
#define MAX_CHG_CURR	    13.5 /* Amps */
#define MAX_DISCHG_CURR	    45 /* Amps */
#define MIN_CHG_TEMP	    0 /* Celsius */
#define MIN_DISCHG_TEMP	    -40 /* Celsius */
#define MAX_CELL_TEMP	    60 /* Celsius (rules) */
#define TYP_IMPDNCE	    0.015 /* Ohms, DC, 50% SoC */

// Pack Limits
#define MAX_PACK_CHG_CURR \
	30 /* Pack-level charge limit: (MAX_CHG_CURR - 3.5A margin) × 3 cells in parallel */
#define MAX_PACK_DISCHG_CURR \
	(MAX_DISCHG_CURR *   \
	 3) /* Pack-level discharge limit: MAX_DISCHG_CURR × 3 cells in parallel */
#define MIN_DCL 30.0f

// ADBMS6830 limits
#define MAX_CHIP_TEMP 60

// Algorithm settings
#define VOLT_SAG_MARGIN \
	0.45 // Volts above the minimum cell voltage we would like to aim for
#define OCV_CURR_THRESH 0.5 /* in A */

// Charging settings
#define CHARGING_CURRENT    3.5

//Fault times
#define OVER_CURR_TIME \
	55000 //todo adjust these based on testing and/or counter values
#define OVER_CHG_CURR_TIME 55000
#define UNDER_VOLT_TIME	   55000
#define OVER_VOLT_CHG_TIME 15000
#define OVER_VOLT_TIME	   55000
#define LOW_CELL_TIME	   55000
#define HIGH_TEMP_TIME	   55000
#define MAX_CHIPTEMP_TIME  55000

// system wide base ADBMS sample rate
#define SAMPLE_RATE 2 /* Hz */

#endif