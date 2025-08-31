#ifndef LOG_TASK_LOG_TASK_H
#define LOG_TASK_LOG_TASK_H

#include "bus_task.h"
#include "common.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

typedef struct {
#ifdef LOG_VIA_UART
    UART_HandleTypeDef* log_bus;
#else
    USBD_ClassTypeDef* log_bus;
#endif
} log_task_ctx_t;

atlas_err_t log_task_initialize(log_task_ctx_t* task_ctx);

void log_task_transmit_done_callback(void);

#endif // LOG_TASK_LOG_TASK_H