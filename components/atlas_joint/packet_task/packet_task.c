#include "packet_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "packet_manager.h"
#include "queue.h"
#include "task.h"
#include <stdint.h>

#define PACKET_TASK_STACK_DEPTH (5000U / sizeof(StackType_t))
#define PACKET_TASK_PRIORITY (1U)
#define PACKET_TASK_NAME ("packet_task")

#define PACKET_QUEUE_ITEMS (10U)
#define PACKET_QUEUE_ITEM_SIZE (sizeof(packet_event_t))
#define PACKET_QUEUE_STORAGE_SIZE (PACKET_QUEUE_ITEMS * PACKET_QUEUE_ITEM_SIZE)

static void packet_task_func(void* ctx)
{
    packet_task_ctx_t* task_ctx = (packet_task_ctx_t*)ctx;

    packet_manager_t manager;
    ATLAS_LOG_ON_ERR(PACKET_TASK_NAME, packet_manager_initialize(&manager, &task_ctx->config));

    while (1) {
        ATLAS_LOG_ON_ERR(PACKET_TASK_NAME, packet_manager_process(&manager));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static TaskHandle_t packet_task_create_task(packet_task_ctx_t* task_ctx)
{
    static StaticTask_t packet_task_buffer;
    static StackType_t packet_task_stack[PACKET_TASK_STACK_DEPTH];

    return xTaskCreateStatic(packet_task_func,
                             PACKET_TASK_NAME,
                             PACKET_TASK_STACK_DEPTH,
                             task_ctx,
                             PACKET_TASK_PRIORITY,
                             packet_task_stack,
                             &packet_task_buffer);
}

static QueueHandle_t packet_task_create_queue(void)
{
    static StaticQueue_t packet_queue_buffer;
    static uint8_t packet_queue_storage[PACKET_QUEUE_STORAGE_SIZE];

    return xQueueCreateStatic(PACKET_QUEUE_ITEMS,
                              PACKET_QUEUE_ITEM_SIZE,
                              packet_queue_storage,
                              &packet_queue_buffer);
}

void packet_task_initialize(packet_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

    queue_manager_set(QUEUE_TYPE_PACKET, packet_task_create_queue());
    task_manager_set(TASK_TYPE_PACKET, packet_task_create_task(task_ctx));
}

#undef PACKET_TASK_STACK_DEPTH
#undef PACKET_TASK_PRIORITY
#undef PACKET_TASK_NAME

#undef PACKET_QUEUE_ITEMS
#undef PACKET_QUEUE_ITEM_SIZE
#undef PACKET_QUEUE_STORAGE_SIZE