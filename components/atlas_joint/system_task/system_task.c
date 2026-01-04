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

#ifndef USE_LOG_TASK
static SemaphoreHandle_t system_task_create_log_mutex(void)
{
    static StaticSemaphore_t log_mutex_buffer;

    return xSemaphoreCreateMutexStatic(&log_mutex_buffer);
}
#endif

#ifndef USE_WATCHDOG_TASK
atlas_err_t system_task_start_refresh_timer(system_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

    return HAL_TIM_Base_Start_IT(task_ctx->config.refresh_timer) == HAL_OK
               ? ATLAS_ERR_OK
               : ATLAS_ERR_FAIL;
}

atlas_err_t system_task_stop_refresh_timer(system_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

    return HAL_TIM_Base_Stop_IT(task_ctx->config.refresh_timer) == HAL_OK
               ? ATLAS_ERR_OK
               : ATLAS_ERR_FAIL;
}
#endif

atlas_err_t system_task_initialize(system_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

#ifndef USE_WATCHDOG_TASK
    ATLAS_RET_ON_ERR(system_task_start_refresh_timer(task_ctx));
#endif

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

#ifndef USE_LOG_TASK
    SemaphoreHandle_t log_mutex = system_task_create_log_mutex();
    if (log_mutex == NULL) {
        return ATLAS_ERR_FAIL;
    }
#endif

    semaphore_manager_set(SEMAPHORE_TYPE_LOG, log_mutex);

    return ATLAS_ERR_OK;
}

#ifndef USE_WATCHDOG_TASK
void system_task_refresh_timer_callback(void)
{
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_SYSTEM),
                       WATCHDOG_NOTIFY_WATCHDOG_TIMER,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}
#endif

#undef SYSTEM_TASK_STACK_DEPTH
#undef SYSTEM_TASK_PRIORITY
#undef SYSTEM_TASK_NAME

#undef SYSTEM_QUEUE_ITEMS
#undef SYSTEM_QUEUE_ITEM_SIZE
#undef SYSTEM_QUEUE_STORAGE_SIZE
