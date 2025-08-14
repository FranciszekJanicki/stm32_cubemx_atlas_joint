#ifndef UART_TASK_UART_TASK_H
#define UART_TASK_UART_TASK_H

#include "bus_task.h"
#include "common.h"
#include "stm32l4xx_hal.h"

typedef struct {
    UART_HandleTypeDef* uart_bus;
} uart_task_ctx_t;

atlas_err_t uart_task_initialize(uart_task_ctx_t* task_ctx);

void uart_task_transmit_done_callback(void);

#endif // UART_TASK_UART_TASK_H