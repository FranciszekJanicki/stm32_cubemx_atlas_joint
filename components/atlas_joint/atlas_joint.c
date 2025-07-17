#include "atlas_rob.h"
#include "joints_task.h"
#include "packet_task.h"
#include "system_task.h"
#include "task.h"
#include "uart_config.h"
#include "uart_task.h"
#include <string.h>

#define UART_TASK_STACK_DEPTH (4096U / sizeof(StackType_t))
#define UART_TASK_PRIORITY (1U)
#define UART_TASK_NAME ("uart_task")

#define UART_BUFFER_STORAGE_SIZE (1024U)

#define UART_STREAM_BUFFER_STORAGE_SIZE (1024U)
#define UART_STREAM_BUFFER_TRIGGER (1U)

static void uart_task_initialize(uart_task_ctx_t* task_ctx)
{
    static StaticStreamBuffer_t uart_stream_buffer_buffer;
    static uint8_t uart_stream_buffer_storage[UART_STREAM_BUFFER_STORAGE_SIZE];

    stream_buffer_manager_set(STREAM_BUFFER_TYPE_UART,
                              uart_task_create_stream_buffer(&uart_stream_buffer_buffer,
                                                             UART_STREAM_BUFFER_TRIGGER,
                                                             UART_STREAM_BUFFER_STORAGE_SIZE,
                                                             uart_stream_buffer_storage));

    static StaticTask_t uart_task_buffer;
    static StackType_t uart_task_stack[UART_TASK_STACK_DEPTH];
    static uint8_t uart_buffer[UART_BUFFER_STORAGE_SIZE];

    task_ctx->uart_buffer = uart_buffer;
    task_ctx->uart_action = UART_ACTION_TRANSMIT;
    task_ctx->uart_buffer_size = UART_BUFFER_STORAGE_SIZE;
    task_ctx->stream_buffer = stream_buffer_manager_get(STREAM_BUFFER_TYPE_UART);

    task_manager_set(TASK_TYPE_UART,
                     uart_task_create_task(task_ctx,
                                           UART_TASK_NAME,
                                           &uart_task_buffer,
                                           UART_TASK_PRIORITY,
                                           uart_task_stack,
                                           UART_TASK_STACK_DEPTH));

    static StaticSemaphore_t uart_mutex_buffer;

    semaphore_manager_set(SEMAPHORE_TYPE_UART, xSemaphoreCreateMutexStatic(&uart_mutex_buffer));
}

void uart_tx_complete_callback(void)
{
    uart_transmit_complete_callback(task_manager_get(TASK_TYPE_UART));
}

void atlas_rob_initialize(atlas_rob_config_t const* config)
{
    ATLAS_ASSERT(config);

    system_task_initialize();
    uart_task_initialize(&config->uart_task_ctx);
    joints_task_initialize(&config->joints_task_ctx);
    packet_task_initialize(&config->packet_task_ctx);

    vTaskStartScheduler();
}

#undef UART_TASK_STACK_DEPTH
#undef UART_TASK_PRIORITY
#undef UART_TASK_NAME

#undef UART_BUFFER_STORAGE_SIZE

#undef UART_STREAM_BUFFER_STORAGE_SIZE
#undef UART_STREAM_BUFFER_TRIGGER
