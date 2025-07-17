#ifndef UART_TASK_UART_MANAGER_H
#define UART_TASK_UART_MANAGER_H

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "uart_config.h"
#include "usart.h"
#include <stddef.h>
#include <stdint.h>

typedef struct {
    StreamBufferHandle_t stream_buffer;
    UART_HandleTypeDef* uart;
    uint8_t* uart_buffer;
    size_t uart_buffer_size;
} uart_manager_t;

uart_err_t uart_manager_initialize(uart_manager_t* manager,
                                   StreamBufferHandle_t stream_buffer,
                                   UART_HandleTypeDef* uart,
                                   uint8_t* uart_buffer,
                                   size_t uart_buffer_size,
                                   uart_action_t action);
uart_err_t uart_manager_process(uart_manager_t* manager);

#endif // UART_TASK_UART_MANAGER_H