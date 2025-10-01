#include "log_task.h"
#include "FreeRTOS.h"
#include "bus_task.h"
#include "common.h"
#include "manager.h"
#include "stream_buffer.h"
#include "task.h"
#include "usart.h"
#include "usbd_cdc_if.h"

#define LOG_TASK_STACK_DEPTH (4096U / sizeof(StackType_t))
#define LOG_TASK_PRIORITY (1U)
#define LOG_TASK_NAME ("atlas_joint:log_task")

#define LOG_BUFFER_STORAGE_SIZE (1024U)

#define LOG_STREAM_BUFFER_STORAGE_SIZE (1024U)
#define LOG_STREAM_BUFFER_TRIGGER (1U)

static bus_err_t bus_task_bus_transmit_data(void* user,
                                            uint8_t const* data,
                                            size_t data_size)
{
#ifdef LOG_VIA_UART
    UART_HandleTypeDef* uart_bus = (UART_HandleTypeDef*)user;
    if (HAL_UART_Transmit_IT(uart_bus, data, data_size) != HAL_OK) {
        return BUS_ERR_TRANSMIT;
    }
#else
    if (CDC_Transmit_FS(data, data_size) != 0) {
        return BUS_ERR_TRANSMIT;
    }
#endif
    return BUS_ERR_OK;
}

atlas_err_t log_task_initialize(log_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

#ifdef USE_LOG_TASK
    static StaticStreamBuffer_t log_stream_buffer_buffer;
    static uint8_t log_stream_buffer_storage[LOG_STREAM_BUFFER_STORAGE_SIZE];

    StreamBufferHandle_t log_stream_buffer =
        bus_task_create_stream_buffer(&log_stream_buffer_buffer,
                                      LOG_STREAM_BUFFER_TRIGGER,
                                      LOG_STREAM_BUFFER_STORAGE_SIZE,
                                      log_stream_buffer_storage);
    if (log_stream_buffer == NULL) {
        return ATLAS_ERR_FAIL;
    }

    stream_buffer_manager_set(STREAM_BUFFER_TYPE_LOG, log_stream_buffer);

    static StaticTask_t log_task_buffer;
    static StackType_t log_task_stack[LOG_TASK_STACK_DEPTH];
    static uint8_t log_buffer[LOG_BUFFER_STORAGE_SIZE];

    static bus_task_ctx_t bus_ctx;
    bus_ctx.config = (bus_config_t){.bus_buffer = log_buffer,
                                    .bus_buffer_size = LOG_BUFFER_STORAGE_SIZE,
                                    .stream_buffer = log_stream_buffer};
    bus_ctx.interface =
        (bus_interface_t){.bus_user = task_ctx->log_bus,
                          .bus_transmit_data = bus_task_bus_transmit_data};

    TaskHandle_t log_task = bus_task_create_task(&bus_ctx,
                                                 LOG_TASK_NAME,
                                                 &log_task_buffer,
                                                 LOG_TASK_PRIORITY,
                                                 log_task_stack,
                                                 LOG_TASK_STACK_DEPTH);
    if (log_task == NULL) {
        return ATLAS_ERR_FAIL;
    }

    task_manager_set(TASK_TYPE_LOG, log_task);
#else
    static StaticSemaphore_t log_mutex_buffer;

    SemaphoreHandle_t log_mutex =
        xSemaphoreCreateMutexStatic(&log_mutex_buffer);
    if (log_mutex == NULL) {
        return ATLAS_ERR_FAIL;
    }

    semaphore_manager_set(SEMAPHORE_TYPE_LOG, log_mutex);
#endif
    return ATLAS_ERR_OK;
}

void log_task_transmit_done_callback(void)
{
#ifdef USE_LOG_TASK
    bus_task_transmit_done_callback(task_manager_get(TASK_TYPE_LOG));
#endif
}

#undef LOG_TASK_STACK_DEPTH
#undef LOG_TASK_PRIORITY
#undef LOG_TASK_NAME

#undef LOG_BUFFER_STORAGE_SIZE

#undef LOG_STREAM_BUFFER_STORAGE_SIZE
#undef LOG_STREAM_BUFFER_TRIGGER
