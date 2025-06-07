
#include "mock_stub_functions.h"
#include "test_charging.h"
#include "datastructs.h"
#include <stdbool.h>
#include <stdlib.h>

I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;

acc_data_t *bmsdata;
chipdata_t *chipdata;


void setUp(void) {
    bmsdata = malloc(sizeof(acc_data_t));
    chipdata = malloc(sizeof(chipdata_t));
}

void tearDown(void) {
    free(bmsdata);
    free(chipdata);
}

// A simple random test
void test_charging(void) {
    
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_charging);
    return UNITY_END();
}