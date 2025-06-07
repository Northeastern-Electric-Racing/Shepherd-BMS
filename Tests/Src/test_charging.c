
#include "test_charging.h"
#include "mock_stub_functions.h"
#include "mock_segment.h"
#include "datastructs.h"
#include <stdbool.h>
#include <stdlib.h>

I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;

acc_data_t *bmsdata;

void setUp(void) {
    bmsdata = malloc(sizeof(acc_data_t));
}

void tearDown(void) {
    free(bmsdata);
}

// A simple random test
void test_segment_enable_calls(void) {
    bmsdata->delt_ocv = 0.02;
    segment_configure_balancing_ExpectAnyArgs();
    segment_enable_balancing_Expect(bmsdata);
    handle_balance_cells(bmsdata);  
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_segment_enable_calls);
    return UNITY_END();
}