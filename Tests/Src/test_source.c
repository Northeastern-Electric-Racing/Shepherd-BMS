
#include "unity.h"
#include "mock_stub_functions.h"
#include <stdbool.h>
#include <stdlib.h>

I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;

void setUp(void) {
    
}

void tearDown(void) {

}

// A simple random test
void test_random(void) {
   
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_random);
    return UNITY_END();
}