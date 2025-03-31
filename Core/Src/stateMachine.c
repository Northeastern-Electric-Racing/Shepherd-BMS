#include "stateMachine.h"

#include "can_messages.h"
#include "compute.h"
#include "segment.h"
#include "charging.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/// the current state of the BMS
BMSState_t current_state = BOOT_STATE;

// the countup timer for settling rest
nertimer_t charger_settle_countup = { .active = false };
// the countdown timer unitl setting rest
nertimer_t charger_settle_countdown = { .active = false };

nertimer_t charger_message_timer;

const bool valid_transition_from_to[NUM_STATES][NUM_STATES] = {
	/*   BOOT,     READY,      CHARGING,   FAULTED	*/
	{ true, true, false, true }, /* BOOT */
	{ false, true, true, true }, /* READY */
	{ false, true, true, true }, /* CHARGING */
	{ true, false, false, true } /* FAULTED */
};

typedef union _bms_fault_t {
	uint64_t all;
	struct {
		uint32_t fault_code_crit;
		uint32_t fault_code_noncrit;
	} fields;
} bms_fault_t;

/* private function prototypes */
void init_boot(acc_data_t *bmsdata);
void init_ready(acc_data_t *bmsdata);
void init_charging(acc_data_t *bmsdata);
void init_faulted(acc_data_t *bmsdata);
void handle_boot(acc_data_t *bmsdata);
void handle_ready(acc_data_t *bmsdata);
void handle_charging(acc_data_t *bmsdata);
void handle_faulted(acc_data_t *bmsdata);
void request_transition(acc_data_t *bmsdata, BMSState_t next_state);

typedef void (*HandlerFunction_t)(acc_data_t *bmsdata);
typedef void (*InitFunction_t)(acc_data_t *bmsdata);

const InitFunction_t init_LUT[NUM_STATES] = { &init_boot, &init_ready,
					      &init_charging, &init_faulted };

const HandlerFunction_t handler_LUT[NUM_STATES] = { &handle_boot, &handle_ready,
						    &handle_charging,
						    &handle_faulted };

void init_boot(acc_data_t *bmsdata)
{
	// useless, really since handle_boot always requests a transition anyways
	return;
}

void handle_boot(acc_data_t *bmsdata)
{
	segment_disable_balancing(bmsdata);
	// the charger could be connected on state machine boot, so lets not re-enter ready!
	if (bmsdata->is_charger_connected) {
		request_transition(bmsdata, CHARGING_STATE);
	} else {
		request_transition(bmsdata, READY_STATE);
	}
	return;
}

void init_ready(acc_data_t *bmsdata)
{
	segment_disable_balancing(bmsdata);
	return;
}

void handle_ready(acc_data_t *bmsdata)
{
	// send our DCL and CCL to motors
	send_mc_charge_message(bmsdata->cont_CCL);
	send_mc_discharge_message(bmsdata->cont_DCL);
}

void init_charging(acc_data_t *bmsdata)
{
	cancel_timer(&charger_settle_countup);
	return;
}

void handle_charging(acc_data_t *bmsdata)
{
	/* Check if we should charge */
	if (sm_charging_check(bmsdata)) {
		bmsdata->is_charging_enabled = true;

		/* Send CAN message, but not too often */
		if (is_timer_expired(&charger_message_timer) ||
		    !is_timer_active(&charger_message_timer)) {
			send_charging_message(
				(MAX_CHARGE_VOLT *
				 (NUM_CELLS_ALPHA + NUM_CELLS_BETA) *
				 NUM_CHIPS),
				5, true);
			start_timer(&charger_message_timer, 1000);
		}
	} else {
		bmsdata->is_charging_enabled = false;
		send_charging_message(0, 0, false);
	}

	/* Check if we should balance */
	if (sm_balancing_check(bmsdata))
		sm_balance_cells(bmsdata);
	else
		segment_disable_balancing(bmsdata);

	// disable discharge and charge from the MC
	send_mc_discharge_message(0);
	send_mc_charge_message(0);
}

void charger_message_recieved(acc_data_t *bmsdata)
{
	// this is irreversible, a LV power cycle occurs before re-connection to car
	bmsdata->is_charger_connected = true;
	request_transition(bmsdata, CHARGING_STATE);
}

void init_faulted(acc_data_t *bmsdata)
{
	// never balance when faulted
	segment_disable_balancing(bmsdata);
	// never charge when faulted
	bmsdata->is_charging_enabled = false;
	return;
}

void handle_faulted(acc_data_t *bmsdata)
{
	// leave faulted if all is well
	if (bmsdata->fault_code_crit == FAULTS_CLEAR) {
		compute_set_fault(false);
		request_transition(bmsdata, BOOT_STATE);
		return;
	}

	// not all is well, re-assert shutdown, turn our DCL and CCL to zero, turn off charging
	compute_set_fault(true);
	send_mc_charge_message(0);
	send_mc_discharge_message(0);
	if (bmsdata->is_charger_connected) {
		send_charging_message(0, 0, false);
	}

	return;
}

void sm_handle_state(acc_data_t *bmsdata)
{
	// always check for faults no matter the current state
	bms_fault_t faults = { .all = 0 };
	faults.all = sm_fault_return(bmsdata);

	bmsdata->fault_code_crit = faults.fields.fault_code_crit;
	bmsdata->fault_code_noncrit = faults.fields.fault_code_noncrit;

	if (bmsdata->fault_code_crit != FAULTS_CLEAR) {
		request_transition(bmsdata, FAULTED_STATE);
	}

	handler_LUT[current_state](bmsdata);
}

void request_transition(acc_data_t *bmsdata, BMSState_t next_state)
{
	if (current_state == next_state)
		return;
	if (!valid_transition_from_to[current_state][next_state])
		return;

	init_LUT[next_state](bmsdata);
	current_state = next_state;
}

uint64_t sm_fault_return(acc_data_t *bmsdata)
{
	/* FAULT CHECK (Check for fuckies) */

	static nertimer_t ovr_curr_timer = { 0 };
	static nertimer_t ovr_chgcurr_timer = { 0 };
	static nertimer_t undr_volt_timer = { 0 };
	static nertimer_t ovr_chgvolt_timer = { 0 };
	static nertimer_t ovr_volt_timer = { 0 };
	static nertimer_t low_cell_timer = { 0 };
	static nertimer_t high_temp_timer = { 0 };
	static nertimer_t die_overtemp_timer = { 0 };
	static fault_eval_t *fault_table = NULL;
	static acc_data_t *fault_data = NULL;

	static uint32_t fault_status_crit = 0;
	static uint32_t fault_status_noncrit = 0;

	if (!fault_data)
		fault_data = bmsdata;

	if (!fault_table) {
		/* Note that we are only allocating this table once at runtime, so there is
         * no need to free it */
		fault_table = (fault_eval_t *)malloc(NUM_FAULTS *
						     sizeof(fault_eval_t));
		// clang-format off
    											// ___________FAULT ID____________   __________TIMER___________   _____________DATA________________    __OPERATOR__   ____________________________________THRESHOLD____________________________  _______TIMER LENGTH_________  _____________FAULT CODE_________________    	___OPERATOR 2__ ________________________DATA 2______________   __THRESHOLD 2_____ ______CRITICAL________
        fault_table[0]  = (fault_eval_t) {.id = "Discharge Current Limit", .timer =       ovr_curr_timer, .data_1 =     fault_data->pack_current,  .optype_1 = GT, .lim_1 = fault_data->cont_DCL ,                                                .timeout =      OVER_CURR_TIME, .code = DISCHARGE_LIMIT_ENFORCEMENT_FAULT,  .optype_2 = NOP/* ------------------------------UNUSED-------------------------*/, .is_critical = true  };
        fault_table[1]  = (fault_eval_t) {.id = "Charge Current Limit",    .timer =    ovr_chgcurr_timer, .data_1 =     fault_data->pack_current,  .optype_1 = GT, .lim_1 =                                        fault_data->cont_CCL,          .timeout =  OVER_CHG_CURR_TIME, .code =    CHARGE_LIMIT_ENFORCEMENT_FAULT,  .optype_2 = LT,  .data_2 =         fault_data->pack_current,  .lim_2 =          0, .is_critical = true  };
        fault_table[2]  = (fault_eval_t) {.id = "Low Cell Voltage",        .timer =      undr_volt_timer, .data_1 =  fault_data->min_voltage.val,  .optype_1 = LT, .lim_1 =                                                     MIN_VOLT,         .timeout =     UNDER_VOLT_TIME, .code =              CELL_VOLTAGE_TOO_LOW,  .optype_2 = NOP/* ------------------------------UNUSED-------------------------*/, .is_critical = true  };
        fault_table[3]  = (fault_eval_t) {.id = "High Cell Voltage",       .timer =    ovr_chgvolt_timer, .data_1 =  fault_data->max_voltage.val,  .optype_1 = GT, .lim_1 =                                              MAX_CHARGE_VOLT,         .timeout =      OVER_VOLT_TIME, .code =             CELL_VOLTAGE_TOO_HIGH,  .optype_2 = NOP/* ------------------------------UNUSED-------------------------*/, .is_critical = true  };
        fault_table[4]  = (fault_eval_t) {.id = "High Cell Voltage",       .timer =       ovr_volt_timer, .data_1 =  fault_data->max_voltage.val,  .optype_1 = GT, .lim_1 =                                                     MAX_VOLT,         .timeout =      OVER_VOLT_TIME, .code =             CELL_VOLTAGE_TOO_HIGH,  .optype_2 = EQ,  .data_2 = fault_data->is_charger_connected,  .lim_2 =      false, .is_critical = true  };
        fault_table[5]  = (fault_eval_t) {.id = "High Temp",               .timer =      high_temp_timer, .data_1 =     fault_data->max_temp.val,  .optype_1 = GT, .lim_1 =                                                        MAX_CELL_TEMP, .timeout =      HIGH_TEMP_TIME, .code =                      PACK_TOO_HOT,  .optype_2 = NOP/* ------------------------------UNUSED-------------------------*/, .is_critical = true  };
    	fault_table[6]  = (fault_eval_t) {.id = "Extremely Low Voltage",   .timer =       low_cell_timer, .data_1 =  fault_data->min_voltage.val,  .optype_1 = LT, .lim_1 =                                                                  0.9, .timeout =       LOW_CELL_TIME, .code =                  LOW_CELL_VOLTAGE,  .optype_2 = NOP/* ------------------------------UNUSED-------------------------*/, .is_critical = true  };
		fault_table[7]  = (fault_eval_t) {.id = "Die Overtemp",            .timer =   die_overtemp_timer, .data_1 = fault_data->max_chiptemp.val,  .optype_1 = GT, .lim_1 = 													   MAX_CHIP_TEMP, .timeout =   MAX_CHIPTEMP_TIME, .code =            DIE_TEMP_MAXIMUM_FAULT,  .optype_2 = NOP/* ------------------------------UNUSED-------------------------*/, .is_critical = true  };
		fault_table[8]  = (fault_eval_t) {.id = NULL};

		cancel_timer(&ovr_curr_timer);
		cancel_timer(&ovr_chgcurr_timer);
		cancel_timer(&undr_volt_timer);
		cancel_timer(&ovr_chgvolt_timer);
		cancel_timer(&ovr_volt_timer);
		cancel_timer(&low_cell_timer);
		cancel_timer(&high_temp_timer);
		// clang-format on
	} else {
		fault_table[0].data_1 = fault_data->pack_current;
		fault_table[0].lim_1 = fault_data->cont_DCL;
		fault_table[1].data_1 = fault_data->pack_current;
		fault_table[1].lim_1 = fault_data->cont_CCL;
		fault_table[2].data_1 = fault_data->min_voltage.val;
		fault_table[3].data_1 = fault_data->max_voltage.val;
		fault_table[4].data_1 = fault_data->max_voltage.val;
		fault_table[4].data_2 = fault_data->is_charger_connected;
		fault_table[5].data_1 = fault_data->max_temp.val;
		fault_table[6].data_1 = fault_data->min_voltage.val;
		fault_table[7].data_1 = fault_data->max_chiptemp.val;
	}

	int incr = 0;
	while (fault_table[incr].id != NULL) {
		uint32_t item_code = fault_table[incr].code;
		if (sm_fault_eval(&fault_table[incr])) {
			if (fault_table[incr].is_critical) {
				fault_status_crit |= item_code;
			} else {
				fault_status_noncrit |= item_code;
			}
		} else {
			// Clear bit for non-critical faults
			if (!fault_table[incr].is_critical) {
				fault_status_noncrit &= ~item_code;
			}
		}
		incr++;
	}

	bms_fault_t return_faults = { .all = 0 };
	return_faults.fields.fault_code_crit = fault_status_crit;
	return_faults.fields.fault_code_noncrit = fault_status_noncrit;

	return return_faults.all;
}

bool sm_fault_eval(fault_eval_t *item)
{
	enum {
		FAULT_STAT_TIMER_START = 1,
		FAULT_STAT_FAULTED = 2,
	};

	bool condition1;
	bool condition2;

	// clang-format off
    switch (item->optype_1)
    {
        case GT: condition1 = item->data_1 > item->lim_1; break;
        case LT: condition1 = item->data_1 < item->lim_1; break;
        case GE: condition1 = item->data_1 >= item->lim_1; break;
        case LE: condition1 = item->data_1 <= item->lim_1; break;
        case EQ: condition1 = item->data_1 == item->lim_1; break;
		case NEQ: condition1 = item->data_1 != item->lim_1; break;
        case NOP: condition1 = false;
		default: condition1 = false;
    }

    switch (item->optype_2)
    {
        case GT: condition2 = item->data_2 > item->lim_2; break;
        case LT: condition2 = item->data_2 < item->lim_2; break;
        case GE: condition2 = item->data_2 >= item->lim_2; break;
        case LE: condition2 = item->data_2 <= item->lim_2; break;
        case EQ: condition2 = item->data_2 == item->lim_2; break;
		case NEQ: condition2 = item->data_2 != item->lim_2; break;
        case NOP: condition2 = false;
		default: condition2 = false;
    }
	// clang-format on

	bool fault_present = ((condition1 && condition2) ||
			      (condition1 && (item->optype_2 == NOP)));
	if ((!(is_timer_active(&item->timer))) && !fault_present) {
		return 0;
	}

	if (is_timer_active(&item->timer)) {
		if (!fault_present) {
			printf("\t\t\t*******Fault cleared: %s\r\n", item->id);
			cancel_timer(&item->timer);
			send_fault_timer_message(0, item->code, item->data_1);
			return 0;
		}

		if (is_timer_expired(&item->timer) && fault_present) {
			printf("\t\t\t*******Faulted: %s\r\n", item->id);
			send_fault_timer_message(2, item->code, item->data_1);
			return item->code;
		}

		else
			return 0;

	}

	else if (!is_timer_active(&item->timer) && fault_present) {
		printf("\t\t\t*******Starting fault timer: %s\r\n", item->id);
		start_timer(&item->timer, item->timeout);
		send_fault_timer_message(1, item->code, item->data_1);

		return 0;
	}
	/* if (item->code == CELL_VOLTAGE_TOO_LOW) {
          printf("\t\t\t*******Not fautled!!!!!\t%d\r\n",
  !is_timer_active(&item->timer) && condition1 && condition2); printf("More
  stats...\t:%d\t%d\r\n", is_timer_expired(&item->timer), item->timer.active);
  } */
	printf("err should not get here");
	return 0;
}

/* charger settle countup =  1 minute pause to let readings settle and get good
 * OCV */
/* charger settle countdown = 5 minute interval between 1 minute settle pauses */
bool sm_charging_check(acc_data_t *bmsdata)
{
	// samity check
	if (!bmsdata->is_charger_connected) {
		//printf("Charger not connected\r\n");
		return false;
	}

	// dont charge during the countup
	if (!is_timer_expired(&charger_settle_countup) &&
	    is_timer_active(&charger_settle_countup)) {
		//printf("Charger settle countup active\r\n");
		return false;
	}

	// if we are counting down (the normal charging time)
	if (is_timer_active(&charger_settle_countdown)) {
		// if we need to stop charging, start the pause timer and stop charging immediately
		if (is_timer_expired(&charger_settle_countdown)) {
			start_timer(&charger_settle_countup,
				    CHARGE_SETL_TIMEOUT);
			return false;
		} else
			return true;
	} else {
		// start the countdown timer if it is inactive, meaning we went from pause --> unpause
		start_timer(&charger_settle_countdown, CHARGE_SETL_TIMEUP);
		return true;
	}
}

// check if balancing is allowed
bool sm_balancing_check(acc_data_t *bmsdata)
{
	if (!bmsdata->is_charger_connected)
		return false;
	if (bmsdata->max_voltage.val <= BAL_MIN_V)
		return false;
	if (bmsdata->delt_voltage <= MAX_DELTA_V)
		return false;

	// do not balance either during the countup
	if (is_timer_active(&charger_settle_countup) &&
	    !is_timer_expired(&charger_settle_countup))
		return false;

	return true;
}

// balances cells using algorithm in charger.c
void sm_balance_cells(acc_data_t *bmsdata)
{
	handle_balance_cells(bmsdata);
}
