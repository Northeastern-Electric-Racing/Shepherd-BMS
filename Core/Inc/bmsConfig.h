#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

#define DEBUG_MODE_ENABLED true
#define DEBUG_STATS

// Hardware definition
#define NUM_SEGMENTS	1
#define NUM_CHIPS	1 //NUM_SEGMENTS * 2
#define NUM_CELLS_ALPHA 14
#define NUM_CELLS_BETA	11
#define NUM_CELLS_SEG	NUM_CELLS_ALPHA + NUM_CELLS_BETA
#define NUM_CELLS                                \
	((NUM_CELLS_ALPHA * (NUM_CHIPS / 2.0)) + \
	 (NUM_CELLS_BETA * (NUM_CHIPS / 2.0)))
#define NUM_THERMS_PER_CHIP 14

// Firmware limits
#define MAX_TEMP	  60 /* Celsius */
#define MIN_TEMP	  -40 /* Celsius */
#define MAX_CELL_TEMP_BAL 45 /* Celsius */
#define MAX_DELTA_V	  0.015
#define BAL_MIN_V	  4.00

// Boosting Parameters
#define BOOST_TIME	    5 // seconds
#define BOOST_RECHARGE_TIME 30 // seconds
#define CONTDCL_MULTIPLIER  3

/* Molicel P45B Cell Specifications */
#define TYP_CAPICITY_AH 4.5 /* Amp-hours */
#define TYP_CAPACITY_WH 16.2 /* Watt-hours */
#define MIN_CAPICITY_AH 4.3 /* Amp-hours */
#define MIN_CAPACITY_WH 15.5 /* Watt-hours */
#define MIN_VOLT	2.5
#define NOM_VOLT	3.6
#define MAX_VOLT	4.2
#define MAX_CHARGE_VOLT 4.205
#define MAX_CHG_CURR	13.5 /* Amps */
#define MAX_DISCHG_CURR 45 /* Amps */
#define MIN_CHG_TEMP	0 /* Celsius */
#define MIN_DISCHG_TEMP -40 /* Celsius */
#define MAX_CELL_TEMP	60 /* Celsius */
#define TYP_IMPDNCE	0.015 /* Ohms, DC, 50% SoC */

// Algorithm settings
#define CHARGE_SETL_TIMEOUT 60000 // 1 minute, may need adjustment
#define CHARGE_SETL_TIMEUP  300000 // 5 minutes, may need adjustment
#define CHARGE_VOLT_TIMEOUT 300000 // 5 minutes, may need adjustment
#define VOLT_SAG_MARGIN \
	0.45 // Volts above the minimum cell voltage we would like to aim for
#define OCV_CURR_THRESH 0.0015 /* 1.5 mA */

#define OCV_AVG 3

#define MAX_STANDARD_DEV 3 // only used for standard deviation for therms calc

//Fault times
#define OVER_CURR_TIME \
	5000 //todo adjust these based on testing and/or counter values
#define PRE_OVER_CURR_TIME  1000
#define OVER_CHG_CURR_TIME  1000
#define UNDER_VOLT_TIME	    45000
#define PRE_UNDER_VOLT_TIME 12000
#define OVER_VOLT_TIME	    45000
#define LOW_CELL_TIME	    45000
#define HIGH_TEMP_TIME	    60000
#define CURR_ERR_MARG	    1.1 // scaling factor, ie 1.1 = 10% error

#define DCDC_CURRENT_DRAW \
	0 // in A, was used because our DCDC was drawing current

#define CAN_MESSAGE_WAIT 5

#define CAN_DISPATCH_DELAY 5

// #define CHARGING

#define SAMPLE_RATE 2 /* Hz */

#endif