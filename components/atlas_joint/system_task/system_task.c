#include "system_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "system_manager.h"
#include "task.h"
#include <stdint.h>

#define SYSTEM_TASK_STACK_DEPTH (10000U / sizeof(StackType_t))
#define SYSTEM_TASK_PRIORITY (1U)
#define SYSTEM_TASK_NAME ("atlas_joint:system_task")

#define SYSTEM_QUEUE_ITEMS (10U)
#define SYSTEM_QUEUE_ITEM_SIZE (sizeof(system_event_t))
#define SYSTEM_QUEUE_STORAGE_SIZE (SYSTEM_QUEUE_ITEMS * SYSTEM_QUEUE_ITEM_SIZE)

static void system_task_func(void* ctx)
{
    system_task_ctx_t* task_ctx = (system_task_ctx_t*)ctx;

    system_manager_t manager;
    ATLAS_LOG_ON_ERR(SYSTEM_TASK_NAME,
                     system_manager_initialize(&manager, &task_ctx->config));

    while (1) {
        ATLAS_LOG_ON_ERR(SYSTEM_TASK_NAME, system_manager_process(&manager));
        ATLAS_DELAY(10);
    }
}

static TaskHandle_t system_task_create_task(system_task_ctx_t* task_ctx)
{
    static StaticTask_t system_task_buffer;
    static StackType_t system_task_stack[SYSTEM_TASK_STACK_DEPTH];

    return xTaskCreateStatic(system_task_func,
                             SYSTEM_TASK_NAME,
                             SYSTEM_TASK_STACK_DEPTH,
                             task_ctx,
                             SYSTEM_TASK_PRIORITY,
                             system_task_stack,
                             &system_task_buffer);
}

static QueueHandle_t system_task_create_queue(void)
{
    static StaticQueue_t system_queue_buffer;
    static uint8_t system_queue_storage[SYSTEM_QUEUE_STORAGE_SIZE];

    return xQueueCreateStatic(SYSTEM_QUEUE_ITEMS,
                              SYSTEM_QUEUE_ITEM_SIZE,
                              system_queue_storage,
                              &system_queue_buffer);
}

atlas_err_t system_task_initialize(system_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

    QueueHandle_t system_queue = system_task_create_queue();
    if (system_queue == NULL) {
        return ATLAS_ERR_FAIL;
    }

    queue_manager_set(QUEUE_TYPE_SYSTEM, system_queue);

    TaskHandle_t system_task = system_task_create_task(task_ctx);
    if (system_task == NULL) {
        return ATLAS_ERR_FAIL;
    }

    task_manager_set(TASK_TYPE_SYSTEM, system_task);

    return ATLAS_ERR_OK;
}

#undef SYSTEM_TASK_STACK_DEPTH
#undef SYSTEM_TASK_PRIORITY
#undef SYSTEM_TASK_NAME

#undef SYSTEM_QUEUE_ITEMS
#undef SYSTEM_QUEUE_ITEM_SIZE
#undef SYSTEM_QUEUE_STORAGE_SIZE
