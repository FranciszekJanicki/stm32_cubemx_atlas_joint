#include "uart_task.h"
#include "FreeRTOS.h"
#include "bus_task.h"
#include "common.h"
#include "manager.h"
#include "stream_buffer.h"
#include "task.h"
#include "usart.h"

#define UART_TASK_STACK_DEPTH (4096U / sizeof(StackType_t))
#define UART_TASK_PRIORITY (1U)
#define UART_TASK_NAME ("uart_task")

#define UART_BUFFER_STORAGE_SIZE (1024U)

#define UART_STREAM_BUFFER_STORAGE_SIZE (1024U)
#define UART_STREAM_BUFFER_TRIGGER (1U)

static bus_err_t bus_task_bus_transmit_data(void* user,
                                            uint8_t const* data,
                                            size_t data_size)
{
    return HAL_UART_Transmit_IT((UART_HandleTypeDef*)user, data, data_size) ==
                   HAL_OK
               ? BUS_ERR_OK
               : BUS_ERR_TRANSMIT;
}

atlas_err_t uart_task_initialize(uart_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

    static StaticStreamBuffer_t uart_stream_buffer_buffer;
    static uint8_t uart_stream_buffer_storage[UART_STREAM_BUFFER_STORAGE_SIZE];

    StreamBufferHandle_t uart_stream_buffer =
        bus_task_create_stream_buffer(&uart_stream_buffer_buffer,
                                      UART_STREAM_BUFFER_TRIGGER,
                                      UART_STREAM_BUFFER_STORAGE_SIZE,
                                      uart_stream_buffer_storage);
    if (uart_stream_buffer == NULL) {
        return ATLAS_ERR_FAIL;
    }

    static StaticTask_t uart_task_buffer;
    static StackType_t uart_task_stack[UART_TASK_STACK_DEPTH];
    static uint8_t uart_buffer[UART_BUFFER_STORAGE_SIZE];

    static bus_task_ctx_t bus_ctx;
    bus_ctx.config = (bus_config_t){.bus_buffer = uart_buffer,
                                    .bus_buffer_size = UART_BUFFER_STORAGE_SIZE,
                                    .stream_buffer = uart_stream_buffer};
    bus_ctx.interface =
        (bus_interface_t){.bus_user = task_ctx->uart_bus,
                          .bus_transmit_data = bus_task_bus_transmit_data};

    TaskHandle_t uart_task = bus_task_create_task(&bus_ctx,
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

void uart_task_transmit_done_callback(void)
{
    bus_task_transmit_done_callback(task_manager_get(TASK_TYPE_UART));
}

#undef UART_TASK_STACK_DEPTH
#undef UART_TASK_PRIORITY
#undef UART_TASK_NAME

#undef UART_BUFFER_STORAGE_SIZE

#undef UART_STREAM_BUFFER_STORAGE_SIZE
#undef UART_STREAM_BUFFER_TRIGGER