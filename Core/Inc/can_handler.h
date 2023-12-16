#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "stm32f4xx_hal.h"

can_t can1;
can_t can2;

/* Shepherd hanbdles both can busses the same way */
void can_receive_callback(CAN_HandleTypeDef *hcan);

void get_can_msg();

#endif // CAN_HANDLER_H