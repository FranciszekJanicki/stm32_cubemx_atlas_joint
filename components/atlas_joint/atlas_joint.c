#include "atlas_joint.h"
#include "joint_task.h"
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

static atlas_err_t uart_task_initialize(uart_task_ctx_t* task_ctx)
{
    static StaticStreamBuffer_t uart_stream_buffer_buffer;
    static uint8_t uart_stream_buffer_storage[UART_STREAM_BUFFER_STORAGE_SIZE];

    StreamBufferHandle_t uart_stream_buffer =
        uart_task_create_stream_buffer(&uart_stream_buffer_buffer,
                                       UART_STREAM_BUFFER_TRIGGER,
                                       UART_STREAM_BUFFER_STORAGE_SIZE,
                                       uart_stream_buffer_storage);
    if (uart_stream_buffer == NULL) {
        return ATLAS_ERR_FAIL;
    }

    static StaticTask_t uart_task_buffer;
    static StackType_t uart_task_stack[UART_TASK_STACK_DEPTH];
    static uint8_t uart_buffer[UART_BUFFER_STORAGE_SIZE];

    task_ctx->uart_buffer = uart_buffer;
    task_ctx->uart_action = UART_ACTION_TRANSMIT;
    task_ctx->uart_buffer_size = UART_BUFFER_STORAGE_SIZE;
    task_ctx->stream_buffer = uart_stream_buffer;

    TaskHandle_t uart_task = uart_task_create_task(task_ctx,
                                                   UART_TASK_NAME,
                                                   &uart_task_buffer,
                                                   UART_TASK_PRIORITY,
                                                   uart_task_stack,
                                                   UART_TASK_STACK_DEPTH);
    if (uart_task == NULL) {
        return ATLAS_ERR_FAIL;
    }

    static StaticSemaphore_t uart_mutex_buffer;

    SemaphoreHandle_t uart_mutex =
        xSemaphoreCreateMutexStatic(&uart_mutex_buffer);
    if (uart_mutex == NULL) {
        return ATLAS_ERR_FAIL;
    }

    stream_buffer_manager_set(STREAM_BUFFER_TYPE_UART, uart_stream_buffer);
    semaphore_manager_set(SEMAPHORE_TYPE_UART, uart_mutex);
    task_manager_set(TASK_TYPE_UART, uart_task);

    return ATLAS_ERR_OK;
}

void uart_tx_complete_callback(void)
{
    uart_transmit_complete_callback(task_manager_get(TASK_TYPE_UART));
}

void atlas_joint_initialize(atlas_joint_config_t const* config)
{
    ATLAS_ASSERT(config);

    ATLAS_ERR_CHECK(system_task_initialize(&config->system_ctx));
    ATLAS_ERR_CHECK(uart_task_initialize(&config->uart_ctx));
    ATLAS_ERR_CHECK(joint_task_initialize(&config->joint_ctx));
    ATLAS_ERR_CHECK(packet_task_initialize(&config->packet_ctx));

    vTaskStartScheduler();
}

#undef UART_TASK_STACK_DEPTH
#undef UART_TASK_PRIORITY
#undef UART_TASK_NAME

#undef UART_BUFFER_STORAGE_SIZE

#undef UART_STREAM_BUFFER_STORAGE_SIZE
#undef UART_STREAM_BUFFER_TRIGGER
