#ifndef PACKET_TASK_PACKET_TASK_H
#define PACKET_TASK_PACKET_TASK_H

#include "packet_manager.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    packet_config_t config;
} packet_task_ctx_t;

atlas_err_t packet_task_initialize(packet_task_ctx_t* task_ctx);

void packet_task_slave_select_callback(void);
void packet_task_transfer_complete_callback(void);

#endif // PACKET_TASK_PACKET_TASK_H