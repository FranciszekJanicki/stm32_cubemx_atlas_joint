#include "uart_task.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"
#include "uart_manager.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

static void uart_task_func(void* ctx)
{
    uart_task_ctx_t* task_ctx = (uart_task_ctx_t*)ctx;

    uart_manager_t manager;
    uart_manager_initialize(&manager,
                            task_ctx->stream_buffer,
                            task_ctx->uart,
                            task_ctx->uart_buffer,
                            task_ctx->uart_buffer_size,
                            task_ctx->uart_action);

    while (1) {
        uart_manager_process(&manager);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

TaskHandle_t uart_task_create_task(uart_task_ctx_t* task_ctx,
                                   char const* task_name,
                                   StaticTask_t* task_buffer,
                                   UBaseType_t task_priority,
                                   StackType_t* task_stack,
                                   UBaseType_t task_stack_size)
{
    assert(task_ctx && task_name && task_buffer && task_stack);

    return xTaskCreateStatic(uart_task_func,
                             task_name,
                             task_stack_size,
                             task_ctx,
                             task_priority,
                             task_stack,
                             task_buffer);
}

StreamBufferHandle_t uart_task_create_stream_buffer(StaticStreamBuffer_t* stream_buffer,
                                                    UBaseType_t stream_buffer_trigger,
                                                    UBaseType_t stream_buffer_storage_size,
                                                    uint8_t* stream_buffer_storage)
{
    assert(stream_buffer && stream_buffer_storage);

    return xStreamBufferCreateStatic(stream_buffer_storage_size,
                                     stream_buffer_trigger,
                                     stream_buffer_storage,
                                     stream_buffer);
}

void uart_receive_complete_callback(TaskHandle_t uart_task)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(uart_task, UART_NOTIFY_RECEIVE_COMPLETE, eSetBits, &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

void uart_transmit_complete_callback(TaskHandle_t uart_task)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(uart_task, UART_NOTIFY_TRANSMIT_COMPLETE, eSetBits, &task_woken);

    portYIELD_FROM_ISR(task_woken);
}
