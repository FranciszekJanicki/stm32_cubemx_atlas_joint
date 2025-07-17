#ifndef UART_TASK_UART_TASK_H
#define UART_TASK_UART_TASK_H

#include "FreeRTOS.h"
#include "stm32l4xx_hal.h"
#include "stream_buffer.h"
#include "task.h"
#include "uart_config.h"
#include <stdint.h>

typedef struct {
    StreamBufferHandle_t stream_buffer;
    UART_HandleTypeDef* uart;
    size_t uart_buffer_size;
    uint8_t* uart_buffer;
    uart_action_t uart_action;
} uart_task_ctx_t;

TaskHandle_t uart_task_create_task(uart_task_ctx_t* task_ctx,
                                   char const* task_name,
                                   StaticTask_t* task_buffer,
                                   UBaseType_t task_priority,
                                   StackType_t* task_stack,
                                   UBaseType_t task_stack_size);
StreamBufferHandle_t uart_task_create_stream_buffer(StaticStreamBuffer_t* stream_buffer,
                                                    UBaseType_t stream_buffer_trigger,
                                                    UBaseType_t stream_buffer_storage_size,
                                                    uint8_t* stream_buffer_storage);

void uart_receive_complete_callback(TaskHandle_t uart_task);
void uart_transmit_complete_callback(TaskHandle_t uart_task);

#endif // UART_TASK_UART_TASK_H