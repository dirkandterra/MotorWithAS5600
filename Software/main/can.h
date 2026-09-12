#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAN_TX_GPIO     4
#define CAN_RX_GPIO     5
#define CAN_POSITION_INVALID 32767

int16_t can_get_PositionCommand(void);
void can_set_PositionActual(int16_t position);

void can_tx_task(void *arg);
void can_rx_task(void *arg);
void can_init(void);
    
#endif